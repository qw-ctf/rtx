// SPDX-License-Identifier: AGPL-3.0-or-later

//! Fetching a map the machine doesn't have.
//!
//! QuakeWorld tries the community HTTP repository first, then asks the connected server if that
//! fails. HTTP stays on a worker thread so a slow mirror cannot freeze the netchan keepalives. The
//! server paths are driven from the session tick: stock servers send sequential `svc_download`
//! blocks, while `FTE_PEXT_CHUNKEDDOWNLOADS` servers answer many random-access requests at once.
//!
//! Both paths install the same way: write a unique file beside the destination, validate the Quake
//! BSP header, and rename only when complete. A failed or interrupted transfer therefore never
//! leaves a partial `maps/foo.bsp` for the next run to trust.

use std::fs::{File, OpenOptions};
use std::io::{Read, Seek, SeekFrom, Write};
use std::path::{Component, Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{self, Receiver};
use std::time::{Duration, Instant};

use rtx_proto::svc::DOWNLOAD_CHUNK_SIZE;

/// The community's primary map source — direct `.bsp` files, the URL shape most servers point real
/// clients at. See <https://maps.quakeworld.nu/>.
const MAP_REPO: &str = "https://maps.quakeworld.nu/all";

/// How long to give HTTP before asking the connected server instead.
const HTTP_TIMEOUT: Duration = Duration::from_secs(60);

/// FTE's useful upper bound: more logical requests only consume client memory while the server's
/// rate limiter catches up. This matches Qualia's deliberately wide window.
pub(crate) const MAX_IN_FLIGHT: usize = 75;

/// A request the server has acknowledged ten packets past without answering is a lost chunk. The
/// time fallback handles a quiet/choked netchan whose acknowledgement stops advancing.
const RETRY_ACK_DISTANCE: u32 = 10;

static TEMP_SEQUENCE: AtomicU64 = AtomicU64::new(1);

/// Shared progress reporting for every transport. Milestones are based on bytes accepted into the
/// transfer, while speed is the average since that transport started; crossing several milestones
/// in one read still emits each 5% line exactly once.
struct DownloadProgress {
    label: String,
    started: Instant,
    received: u64,
    next_percent: u8,
}

impl DownloadProgress {
    fn new(label: String, started: Instant) -> Self {
        DownloadProgress { label, started, received: 0, next_percent: 5 }
    }

    fn add(&mut self, bytes: u64, percent: u8, source: &str, now: Instant) {
        self.received = self.received.saturating_add(bytes);
        self.report(percent, source, now);
    }

    fn finish(&mut self, source: &str, now: Instant) {
        self.report(100, source, now);
    }

    fn report(&mut self, percent: u8, source: &str, now: Instant) {
        let milestones = self.milestones(percent);
        if milestones.is_empty() {
            return;
        }
        let speed = format_speed(self.received, now.saturating_duration_since(self.started));
        for milestone in milestones {
            eprintln!(
                "rtx-client: downloading {} {source}: {milestone}% at {speed}",
                self.label
            );
        }
    }

    fn milestones(&mut self, percent: u8) -> Vec<u8> {
        let mut crossed = Vec::new();
        while self.next_percent <= percent.min(100) && self.next_percent <= 100 {
            crossed.push(self.next_percent);
            self.next_percent += 5;
        }
        crossed
    }
}

fn format_speed(bytes: u64, elapsed: Duration) -> String {
    let bytes_per_second = bytes as f64 / elapsed.as_secs_f64().max(0.001);
    if bytes_per_second >= 1024.0 * 1024.0 {
        format!("{:.2} MiB/s", bytes_per_second / (1024.0 * 1024.0))
    } else if bytes_per_second >= 1024.0 {
        format!("{:.1} KiB/s", bytes_per_second / 1024.0)
    } else {
        format!("{bytes_per_second:.0} B/s")
    }
}

/// An HTTP download in flight: the channel its worker reports through.
pub(crate) struct Download {
    rx: Receiver<Result<PathBuf, String>>,
}

impl Download {
    /// Start fetching `<map>.bsp` into `<basedir>/<gamedir>/maps/`, in the background.
    pub(crate) fn start(basedir: PathBuf, gamedir: String, map: String) -> Download {
        let (tx, rx) = mpsc::channel();
        std::thread::spawn(move || {
            let _ = tx.send(fetch_http(&basedir, &gamedir, &map));
        });
        Download { rx }
    }

    /// `None` while the worker is running, then its one result.
    pub(crate) fn poll(&self) -> Option<Result<PathBuf, String>> {
        match self.rx.try_recv() {
            Ok(result) => Some(result),
            Err(mpsc::TryRecvError::Empty) => None,
            Err(mpsc::TryRecvError::Disconnected) => Some(Err("download worker vanished".to_string())),
        }
    }

    #[cfg(test)]
    pub(crate) fn completed(result: Result<PathBuf, String>) -> Download {
        let (tx, rx) = mpsc::channel();
        tx.send(result).unwrap();
        Download { rx }
    }
}

#[derive(Debug)]
struct DownloadPaths {
    destination: PathBuf,
    temporary: PathBuf,
}

impl DownloadPaths {
    fn new(basedir: &Path, gamedir: &str, map: &str) -> Result<Self, String> {
        validate_map_name(map)?;
        let root = basedir.join(if gamedir.is_empty() { "qw" } else { gamedir });
        let destination = root.join("maps").join(format!("{map}.bsp"));
        let parent = destination
            .parent()
            .ok_or_else(|| format!("{} has no parent directory", destination.display()))?;
        let file = destination.file_name().and_then(|n| n.to_str()).unwrap_or("map.bsp");
        let id = TEMP_SEQUENCE.fetch_add(1, Ordering::Relaxed);
        let temporary = parent.join(format!(".{file}.part-{}-{id}", std::process::id()));
        Ok(DownloadPaths { destination, temporary })
    }

    fn create(&self) -> Result<File, String> {
        let parent = self
            .temporary
            .parent()
            .ok_or_else(|| format!("{} has no parent directory", self.temporary.display()))?;
        std::fs::create_dir_all(parent).map_err(|e| format!("can't make {}: {e}", parent.display()))?;
        OpenOptions::new()
            .read(true)
            .write(true)
            .create_new(true)
            .open(&self.temporary)
            .map_err(|e| format!("can't create {}: {e}", self.temporary.display()))
    }

    fn install(&self, mut file: File) -> Result<PathBuf, String> {
        file.flush().map_err(|e| format!("can't flush {}: {e}", self.temporary.display()))?;
        file.seek(SeekFrom::Start(0))
            .map_err(|e| format!("can't seek {}: {e}", self.temporary.display()))?;
        let mut header = [0u8; 4];
        file.read_exact(&mut header)
            .map_err(|e| format!("can't validate {}: {e}", self.temporary.display()))?;
        if !is_bsp(&header) {
            return Err(format!("{} is not a Quake BSP", self.temporary.display()));
        }
        drop(file);

        // A squadmate may have installed the same map while this transfer was in flight. Keep the
        // first validated winner instead of replacing it with an equivalent copy.
        if is_bsp_file(&self.destination) {
            std::fs::remove_file(&self.temporary)
                .map_err(|e| format!("can't discard {}: {e}", self.temporary.display()))?;
            return Ok(self.destination.clone());
        }
        match std::fs::rename(&self.temporary, &self.destination) {
            Ok(()) => Ok(self.destination.clone()),
            // Close the check/rename race on platforms whose rename does not replace an existing
            // file. As above, trust the winner only after the same header validation.
            Err(_) if is_bsp_file(&self.destination) => {
                let _ = std::fs::remove_file(&self.temporary);
                Ok(self.destination.clone())
            }
            Err(e) => Err(format!(
                "can't install {} as {}: {e}",
                self.temporary.display(),
                self.destination.display()
            )),
        }
    }
}

impl Drop for DownloadPaths {
    fn drop(&mut self) {
        // Successful installs have already renamed this path away. Every other exit should discard
        // the private partial file, including an HTTP write/rename error before a ServerDownload
        // exists to own the cleanup.
        let _ = std::fs::remove_file(&self.temporary);
    }
}

fn validate_map_name(map: &str) -> Result<(), String> {
    let safe = !map.is_empty()
        && !map.contains(['\\', ':'])
        && Path::new(map).components().all(|c| matches!(c, Component::Normal(_)));
    if safe {
        Ok(())
    } else {
        Err(format!("refusing unsafe map name {map:?}"))
    }
}

/// A server transfer. Exactly one server-side file can be active on a QuakeWorld connection.
pub(crate) struct ServerDownload {
    expected_name: String,
    cookie: u32,
    paths: DownloadPaths,
    file: Option<File>,
    state: ServerState,
    progress: DownloadProgress,
}

enum ServerState {
    Awaiting,
    Legacy(LegacyState),
    Chunked(ChunkedState),
    Complete,
}

struct LegacyState {
    request: LegacyRequest,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum LegacyRequest {
    None,
    Queued,
    Sent(u32),
}

struct ChunkedState {
    size: u64,
    total_chunks: u32,
    received_bytes: u64,
    ranges: RangeContainer,
    rate: f32,
    credit: f32,
    last_poll: Instant,
}

impl ServerDownload {
    /// Prepare to receive `maps/<map>.bsp`. `cookie` must be non-zero so FTE can echo it in OOB
    /// replies and stale packets from an earlier transfer cannot be mistaken for this file.
    pub(crate) fn new(
        basedir: PathBuf,
        gamedir: &str,
        map: &str,
        cookie: u32,
    ) -> Result<Self, String> {
        if cookie == 0 {
            return Err("chunked download cookie must be non-zero".to_string());
        }
        let expected_name = format!("maps/{map}.bsp");
        Ok(ServerDownload {
            progress: DownloadProgress::new(expected_name.clone(), Instant::now()),
            expected_name,
            cookie,
            paths: DownloadPaths::new(&basedir, gamedir, map)?,
            file: None,
            state: ServerState::Awaiting,
        })
    }

    pub(crate) fn expected_name(&self) -> &str {
        &self.expected_name
    }

    pub(crate) fn cookie(&self) -> u32 {
        self.cookie
    }

    pub(crate) fn is_chunked(&self) -> bool {
        matches!(self.state, ServerState::Chunked(_))
    }

    /// Absorb one stock QuakeWorld block. The server sends the first block in response to
    /// `download`; each non-final block is followed by a client `nextdl`.
    pub(crate) fn receive_legacy(&mut self, data: &[u8], percent: u8) -> Result<Option<PathBuf>, String> {
        if percent > 100 {
            return Err(format!("invalid legacy download percentage {percent}"));
        }
        if matches!(self.state, ServerState::Awaiting) {
            self.file = Some(self.paths.create()?);
            self.state = ServerState::Legacy(LegacyState { request: LegacyRequest::None });
        }
        let ServerState::Legacy(active) = &mut self.state else {
            return Err("legacy block arrived during a different download method".to_string());
        };
        active.request = LegacyRequest::None;
        self.file
            .as_mut()
            .ok_or_else(|| "legacy download has no temporary file".to_string())?
            .write_all(data)
            .map_err(|e| format!("can't write {}: {e}", self.paths.temporary.display()))?;
        self.progress.add(
            data.len() as u64,
            percent,
            "from the server (regular QuakeWorld)",
            Instant::now(),
        );

        if percent == 100 {
            self.finish().map(Some)
        } else {
            Ok(None)
        }
    }

    /// A regular block is a response to the previous reliable `nextdl`. QuakeWorld can replay a
    /// reliable server payload while that request is still queued or in flight, so only accept the
    /// next block after the server has acknowledged the packet that actually carried `nextdl`.
    pub(crate) fn legacy_reply_ready(&self, acknowledged: u32, reliable_acked: bool) -> bool {
        match &self.state {
            ServerState::Awaiting => true,
            ServerState::Legacy(active) => match active.request {
                LegacyRequest::None => true,
                LegacyRequest::Queued => false,
                LegacyRequest::Sent(sequence) => {
                    reliable_acked && acknowledged.wrapping_sub(sequence) < (1 << 30)
                }
            },
            _ => false,
        }
    }

    pub(crate) fn mark_legacy_request_queued(&mut self) {
        if let ServerState::Legacy(active) = &mut self.state {
            active.request = LegacyRequest::Queued;
        }
    }

    /// Called when the netchan promotes its queued reliable bytes into this outgoing packet.
    pub(crate) fn mark_legacy_request_sent(&mut self, sequence: u32) {
        if let ServerState::Legacy(LegacyState { request: request @ LegacyRequest::Queued }) = &mut self.state {
            *request = LegacyRequest::Sent(sequence);
        }
    }

    /// Adopt the metadata FTE sends after `download <name>`.
    pub(crate) fn begin_chunked(&mut self, name: &str, size: u64, now: Instant) -> Result<(), String> {
        if name != self.expected_name {
            return Err(format!(
                "server offered {name:?} for requested {:?}",
                self.expected_name
            ));
        }
        if !matches!(self.state, ServerState::Awaiting) {
            return Err("server sent duplicate download metadata".to_string());
        }
        if size == 0 {
            return Err("server offered an empty map".to_string());
        }
        let chunks = size.div_ceil(DOWNLOAD_CHUNK_SIZE as u64);
        let chunks = u32::try_from(chunks).map_err(|_| format!("map is too large for FTE chunk numbers: {size} bytes"))?;
        let file = self.paths.create()?;
        file.set_len(size).map_err(|e| format!("can't size {}: {e}", self.paths.temporary.display()))?;
        self.file = Some(file);
        self.state = ServerState::Chunked(ChunkedState {
            size,
            total_chunks: chunks,
            received_bytes: 0,
            ranges: RangeContainer::new(chunks),
            rate: 1.0,
            credit: 0.0,
            last_poll: now,
        });
        Ok(())
    }

    /// Write one in-band or OOB FTE block. Missing and pending ranges are both accepted, so a reply
    /// that arrives just after its request was recycled still saves useful bandwidth.
    pub(crate) fn receive_chunk(
        &mut self,
        chunk: u32,
        data: &[u8; DOWNLOAD_CHUNK_SIZE],
    ) -> Result<Option<PathBuf>, String> {
        let ServerState::Chunked(active) = &mut self.state else {
            return Err("chunk arrived without active FTE metadata".to_string());
        };
        if chunk >= active.total_chunks {
            return Err(format!(
                "chunk {chunk} lies beyond the advertised {} chunks",
                active.total_chunks
            ));
        }
        if !active.ranges.contains(chunk) {
            return Ok(None); // duplicate
        }
        let offset = u64::from(chunk)
            .checked_mul(DOWNLOAD_CHUNK_SIZE as u64)
            .ok_or_else(|| format!("chunk {chunk} offset overflow"))?;
        if offset >= active.size {
            return Err(format!("chunk {chunk} lies beyond {} bytes", active.size));
        }
        let len = (active.size - offset).min(DOWNLOAD_CHUNK_SIZE as u64) as usize;
        let file = self.file.as_mut().ok_or_else(|| "chunked download has no temporary file".to_string())?;
        file.seek(SeekFrom::Start(offset))
            .and_then(|_| file.write_all(&data[..len]))
            .map_err(|e| format!("can't write {} at {offset}: {e}", self.paths.temporary.display()))?;

        if !active.ranges.complete(chunk) {
            return Ok(None);
        }
        active.received_bytes += len as u64;
        active.rate = (active.rate + 1.0).min(MAX_IN_FLIGHT as f32);
        let progress_percent = (active.received_bytes.saturating_mul(100) / active.size).min(100) as u8;
        let complete = active.ranges.is_empty();
        if complete {
            debug_assert_eq!(active.received_bytes, active.size);
        }
        self.progress.add(
            len as u64,
            progress_percent,
            "from the server (FTE chunked)",
            Instant::now(),
        );
        if complete {
            self.finish().map(Some)
        } else {
            Ok(None)
        }
    }

    /// How many new chunk commands this tick may carry. The scheduler begins with one request per
    /// tick, accelerates by one for every useful reply, and backs off slightly when requests expire.
    pub(crate) fn request_budget(&mut self, now: Instant, acknowledged: u32, rtt: f32) -> usize {
        let ServerState::Chunked(active) = &mut self.state else { return 0 };
        let retry_after = Duration::from_secs_f32(if rtt.is_finite() { (rtt * 4.0).max(1.0) } else { 1.0 });
        let expired = active.ranges.expire(acknowledged, now, retry_after);
        if expired > 0 {
            active.rate = (active.rate * 0.98).max(1.0);
        }

        let elapsed = now.duration_since(active.last_poll).min(Duration::from_millis(100));
        active.last_poll = now;
        active.credit += active.rate * elapsed.as_secs_f32();
        let mut budget = active.credit.floor() as usize;
        active.credit -= budget as f32;
        if budget == 0 && active.rate < MAX_IN_FLIGHT as f32 {
            budget = 1;
        }
        budget.min(MAX_IN_FLIGHT.saturating_sub(active.ranges.outstanding()))
    }

    pub(crate) fn next_missing_chunk(&self) -> Option<u32> {
        let ServerState::Chunked(active) = &self.state else { return None };
        active.ranges.next_missing()
    }

    pub(crate) fn mark_requested(&mut self, chunk: u32, sequence: u32, now: Instant) -> bool {
        let ServerState::Chunked(active) = &mut self.state else { return false };
        active.ranges.mark_requested(chunk, sequence, now)
    }

    pub(crate) fn percent(&self) -> u8 {
        let ServerState::Chunked(active) = &self.state else { return 0 };
        ((active.received_bytes.saturating_mul(100) / active.size).min(99)) as u8
    }

    fn finish(&mut self) -> Result<PathBuf, String> {
        let file = self.file.take().ok_or_else(|| "download completed without a file".to_string())?;
        let result = self.paths.install(file);
        self.state = ServerState::Complete;
        result
    }
}

impl Drop for ServerDownload {
    fn drop(&mut self) {
        self.file.take();
    }
}

#[derive(Clone, Copy, Debug)]
struct Range {
    start: u32,
    end: u32,
    state: RangeState,
}

#[derive(Clone, Copy, Debug)]
enum RangeState {
    Missing,
    Pending { sequence: u32, sent_at: Instant },
}

/// Compressed missing/pending chunk ranges. Completed chunks are absent. Unlike Qualia's original
/// exact-range removal, completion can carve a chunk out of a re-merged missing span, so a delayed
/// reply received just after timeout is still useful.
struct RangeContainer {
    ranges: Vec<Range>,
}

impl RangeContainer {
    fn new(chunks: u32) -> Self {
        RangeContainer {
            ranges: vec![Range { start: 0, end: chunks, state: RangeState::Missing }],
        }
    }

    fn contains(&self, chunk: u32) -> bool {
        self.ranges.iter().any(|r| r.start <= chunk && chunk < r.end)
    }

    fn is_empty(&self) -> bool {
        self.ranges.is_empty()
    }

    fn outstanding(&self) -> usize {
        self.ranges
            .iter()
            .filter(|r| matches!(r.state, RangeState::Pending { .. }))
            .count()
    }

    fn next_missing(&self) -> Option<u32> {
        self.ranges.iter().find(|r| matches!(r.state, RangeState::Missing)).map(|r| r.start)
    }

    fn mark_requested(&mut self, chunk: u32, sequence: u32, now: Instant) -> bool {
        let Some(i) = self.ranges.iter().position(|r| {
            matches!(r.state, RangeState::Missing) && r.start <= chunk && chunk < r.end
        }) else {
            return false;
        };
        let original = self.ranges[i];
        let pending = Range {
            start: chunk,
            end: chunk + 1,
            state: RangeState::Pending { sequence, sent_at: now },
        };
        let mut replacement = Vec::with_capacity(3);
        if original.start < chunk {
            replacement.push(Range { end: chunk, ..original });
        }
        replacement.push(pending);
        if chunk + 1 < original.end {
            replacement.push(Range { start: chunk + 1, ..original });
        }
        self.ranges.splice(i..=i, replacement);
        true
    }

    fn complete(&mut self, chunk: u32) -> bool {
        let Some(i) = self.ranges.iter().position(|r| r.start <= chunk && chunk < r.end) else {
            return false;
        };
        let original = self.ranges[i];
        let mut replacement = Vec::with_capacity(2);
        if original.start < chunk {
            replacement.push(Range { end: chunk, ..original });
        }
        if chunk + 1 < original.end {
            replacement.push(Range { start: chunk + 1, ..original });
        }
        self.ranges.splice(i..=i, replacement);
        self.merge_missing();
        true
    }

    fn expire(&mut self, acknowledged: u32, now: Instant, retry_after: Duration) -> usize {
        let mut expired = 0;
        for range in &mut self.ranges {
            let RangeState::Pending { sequence, sent_at } = range.state else { continue };
            let ack_distance = acknowledged.wrapping_sub(sequence);
            let ack_passed = (RETRY_ACK_DISTANCE..(1 << 30)).contains(&ack_distance);
            if ack_passed || now.duration_since(sent_at) >= retry_after {
                range.state = RangeState::Missing;
                expired += 1;
            }
        }
        if expired > 0 {
            self.merge_missing();
        }
        expired
    }

    fn merge_missing(&mut self) {
        let mut i = 1;
        while i < self.ranges.len() {
            let merge = matches!(self.ranges[i - 1].state, RangeState::Missing)
                && matches!(self.ranges[i].state, RangeState::Missing)
                && self.ranges[i - 1].end == self.ranges[i].start;
            if merge {
                self.ranges[i - 1].end = self.ranges[i].end;
                self.ranges.remove(i);
            } else {
                i += 1;
            }
        }
    }
}

fn fetch_http(basedir: &Path, gamedir: &str, map: &str) -> Result<PathBuf, String> {
    let url = format!("{MAP_REPO}/{map}.bsp");
    let label = format!("maps/{map}.bsp");
    let paths = DownloadPaths::new(basedir, gamedir, map)?;
    let bytes = http_get(&url, &label)?;
    if !is_bsp(&bytes) {
        return Err(format!("{url} did not return a Quake BSP (got {} bytes)", bytes.len()));
    }
    let mut file = paths.create()?;
    file.write_all(&bytes).map_err(|e| format!("can't write {}: {e}", paths.temporary.display()))?;
    paths.install(file)
}

fn http_get(url: &str, label: &str) -> Result<Vec<u8>, String> {
    let client = reqwest::blocking::Client::builder()
        .timeout(HTTP_TIMEOUT)
        .build()
        .map_err(|e| format!("couldn't build the http client: {e}"))?;
    let resp = client.get(url).send().map_err(|e| format!("fetching {url}: {e}"))?;
    let mut resp = resp.error_for_status().map_err(|e| format!("fetching {url}: {e}"))?;
    let expected = resp.content_length();
    // Content-Length is remote input. It is useful as an allocation hint, but never reserve an
    // attacker-sized sparse body up front; a genuinely larger map can grow the vector normally.
    let capacity = expected
        .and_then(|n| usize::try_from(n).ok())
        .unwrap_or_default()
        .min(64 * 1024 * 1024);
    let mut bytes = Vec::with_capacity(capacity);
    let mut progress = DownloadProgress::new(label.to_string(), Instant::now());
    let mut block = [0u8; 64 * 1024];
    loop {
        let read = resp.read(&mut block).map_err(|e| format!("reading {url}: {e}"))?;
        if read == 0 {
            break;
        }
        bytes.extend_from_slice(&block[..read]);
        let percent = expected
            .filter(|total| *total != 0)
            .map(|total| ((bytes.len() as u64).saturating_mul(100) / total).min(100) as u8)
            .unwrap_or(0);
        progress.add(read as u64, percent, "over HTTP", Instant::now());
    }
    progress.finish("over HTTP", Instant::now());
    Ok(bytes)
}

fn is_bsp(bytes: &[u8]) -> bool {
    bytes.len() >= 4 && i32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]) == 29
}

fn is_bsp_file(path: &Path) -> bool {
    let Ok(mut file) = File::open(path) else { return false };
    let mut header = [0u8; 4];
    file.read_exact(&mut header).is_ok() && is_bsp(&header)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn temp_root() -> PathBuf {
        let id = TEMP_SEQUENCE.fetch_add(1, Ordering::Relaxed);
        let root = std::env::temp_dir().join(format!("rtx-download-test-{}-{id}", std::process::id()));
        std::fs::create_dir_all(&root).unwrap();
        root
    }

    fn bsp_bytes(len: usize) -> Vec<u8> {
        let mut bytes = vec![0x5a; len.max(4)];
        bytes[..4].copy_from_slice(&29i32.to_le_bytes());
        bytes
    }

    #[test]
    fn progress_reports_every_five_percent_once_with_speed() {
        let now = Instant::now();
        let mut progress = DownloadProgress::new("maps/test.bsp".to_string(), now);
        assert!(progress.milestones(4).is_empty());
        assert_eq!(progress.milestones(5), vec![5]);
        assert!(progress.milestones(5).is_empty());
        assert_eq!(progress.milestones(17), vec![10, 15]);
        assert_eq!(progress.milestones(100), (20..=100).step_by(5).collect::<Vec<_>>());
        assert!(progress.milestones(100).is_empty());

        assert_eq!(format_speed(2 * 1024 * 1024, Duration::from_secs(1)), "2.00 MiB/s");
        assert_eq!(format_speed(1536, Duration::from_secs(1)), "1.5 KiB/s");
    }

    #[test]
    fn only_a_real_bsp_header_is_accepted() {
        assert!(is_bsp(&29i32.to_le_bytes()), "version 29 is the Quake BSP");
        assert!(!is_bsp(b"<!DOCTYPE html><title>404"), "a 404 page is not a map");
        assert!(!is_bsp(b""), "nor an empty file");
        assert!(!is_bsp(&30i32.to_le_bytes()), "nor a version we don't read");
    }

    #[test]
    fn unsafe_map_names_never_escape_the_gamedir() {
        for name in ["", "../config", "a/../../config", "/absolute", "a\\b", "C:drive"] {
            assert!(DownloadPaths::new(Path::new("/tmp/unused"), "qw", name).is_err(), "{name:?}");
        }
        assert!(DownloadPaths::new(Path::new("/tmp/unused"), "qw", "race/route1").is_ok());
    }

    #[test]
    fn range_container_accepts_out_of_order_late_and_duplicate_chunks() {
        let now = Instant::now();
        let mut ranges = RangeContainer::new(5);
        assert_eq!(ranges.next_missing(), Some(0));
        assert!(ranges.mark_requested(0, 10, now));
        assert!(ranges.mark_requested(1, 10, now));
        assert_eq!(ranges.outstanding(), 2);
        assert!(ranges.complete(1));
        assert!(!ranges.complete(1), "duplicate is absent from the range set");

        assert_eq!(ranges.expire(20, now, Duration::from_secs(10)), 1);
        assert_eq!(ranges.next_missing(), Some(0));
        assert!(ranges.complete(0), "a reply after expiry still carves the merged missing span");
        for chunk in [4, 2, 3] {
            assert!(ranges.complete(chunk));
        }
        assert!(ranges.is_empty());
    }

    #[test]
    fn chunk_scheduler_caps_the_wide_window_and_recycles_loss() {
        let root = temp_root();
        let now = Instant::now();
        let mut dl = ServerDownload::new(root.clone(), "qw", "wide", 1).unwrap();
        dl.begin_chunked("maps/wide.bsp", 200 * DOWNLOAD_CHUNK_SIZE as u64, now).unwrap();
        for sequence in 1..=MAX_IN_FLIGHT as u32 {
            let chunk = dl.next_missing_chunk().unwrap();
            assert!(dl.mark_requested(chunk, sequence, now));
        }
        assert_eq!(dl.request_budget(now, 0, 0.01), 0, "window is full");
        assert!(dl.request_budget(now, 100, 0.01) > 0, "advanced ack recycles unanswered chunks");
        drop(dl);
        let _ = std::fs::remove_dir_all(root);
    }

    #[test]
    fn chunked_download_installs_out_of_order_and_truncates_the_tail() {
        let root = temp_root();
        let now = Instant::now();
        let size = DOWNLOAD_CHUNK_SIZE * 2 + 17;
        let bytes = bsp_bytes(size);
        let mut dl = ServerDownload::new(root.clone(), "qw", "chunked", 7).unwrap();
        dl.begin_chunked("maps/chunked.bsp", size as u64, now).unwrap();
        let mut chunks = [[0u8; DOWNLOAD_CHUNK_SIZE]; 3];
        for (i, chunk) in chunks.iter_mut().enumerate() {
            let start = i * DOWNLOAD_CHUNK_SIZE;
            let end = bytes.len().min(start + DOWNLOAD_CHUNK_SIZE);
            chunk[..end - start].copy_from_slice(&bytes[start..end]);
        }
        assert!(dl.receive_chunk(2, &chunks[2]).unwrap().is_none());
        assert!(dl.receive_chunk(3, &chunks[2]).is_err(), "the advertised size bounds chunk ids");
        assert!(dl.receive_chunk(0, &chunks[0]).unwrap().is_none());
        let path = dl.receive_chunk(1, &chunks[1]).unwrap().unwrap();
        assert_eq!(std::fs::read(path).unwrap(), bytes);
        let _ = std::fs::remove_dir_all(root);
    }

    #[test]
    fn legacy_download_installs_only_the_final_valid_bsp() {
        let root = temp_root();
        let bytes = bsp_bytes(1500);
        let mut dl = ServerDownload::new(root.clone(), "qw", "legacy", 1).unwrap();
        assert!(dl.receive_legacy(&bytes[..768], 51).unwrap().is_none());
        dl.mark_legacy_request_queued();
        assert!(!dl.legacy_reply_ready(16, true), "a replay before nextdl is sent is not a new block");
        dl.mark_legacy_request_sent(17);
        assert!(!dl.legacy_reply_ready(16, true), "nor is a replay before the nextdl packet is acknowledged");
        assert!(!dl.legacy_reply_ready(18, false), "a later packet is insufficient without the reliable ack bit");
        assert!(dl.legacy_reply_ready(17, true));
        let path = dl.receive_legacy(&bytes[768..], 100).unwrap().unwrap();
        assert_eq!(std::fs::read(path).unwrap(), bytes);

        let mut bad = ServerDownload::new(root.clone(), "qw", "bad", 2).unwrap();
        assert!(bad.receive_legacy(b"not a bsp", 100).is_err());
        assert!(!root.join("qw/maps/bad.bsp").exists());
        let _ = std::fs::remove_dir_all(root);
    }
}
