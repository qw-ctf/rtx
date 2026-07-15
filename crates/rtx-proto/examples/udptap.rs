// SPDX-License-Identifier: AGPL-3.0-or-later

//! A UDP tap: sit between a QuakeWorld client and a server, forward everything, and write each
//! datagram to disk as a parser fixture.
//!
//! ```text
//! ezquake ──▶ 127.0.0.1:27600 ──[udptap]──▶ server:27500
//!         ◀──                            ◀──
//! ```
//!
//! The point is fixtures the parser can be tested against *without* a live server: point a real
//! client at the tap, play for a minute, and every datagram lands in `<dir>` as
//! `NNNNNN-{c2s,s2c}.bin`. `tests/fixtures.rs` then replays the `s2c` side through
//! [`svc::parse`](rtx_proto::svc::parse) and asserts the whole session decodes.
//!
//! It also injects impairment (`--loss`, `--dup`, `--delay`), because a client that only ever runs
//! on a flawless loopback has never had its retransmit or delta-invalidation paths executed. Losing
//! 20% of packets on purpose is the cheapest way to find out whether they work.
//!
//! ```sh
//! cargo run -p rtx-proto --example udptap -- --listen 27600 --server 127.0.0.1:27500 --out fixtures/
//! cargo run -p rtx-proto --example udptap -- --listen 27600 --server host:27500 --loss 20 --delay 150
//! ```
//!
//! Note the tap is deliberately single-client: it binds one upstream socket and pairs it with the
//! first client address it hears from. That's all a fixture capture needs.

use std::collections::VecDeque;
use std::net::{SocketAddr, ToSocketAddrs, UdpSocket};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

struct Args {
    listen: u16,
    server: SocketAddr,
    out: Option<PathBuf>,
    loss: u32,
    dup: u32,
    delay: Duration,
}

fn usage() -> ! {
    eprintln!(
        "usage: udptap --server <host:port> [--listen <port>] [--out <dir>]\n\
         \x20             [--loss <pct>] [--dup <pct>] [--delay <ms>]\n\n\
         Point a QuakeWorld client at 127.0.0.1:<listen> and it plays on <server>, while every\n\
         datagram is written to <dir> as a parser fixture.\n\n\
         \x20 --loss/--dup  drop or duplicate this %% of packets (both directions)\n\
         \x20 --delay       hold every packet this long before forwarding"
    );
    std::process::exit(2)
}

fn parse_args() -> Args {
    let mut a = Args {
        listen: 27600,
        server: ([127, 0, 0, 1], 27500).into(),
        out: None,
        loss: 0,
        dup: 0,
        delay: Duration::ZERO,
    };
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    let mut have_server = false;
    while i < argv.len() {
        let val = || argv.get(i + 1).cloned().unwrap_or_else(|| usage());
        match argv[i].as_str() {
            "--listen" => a.listen = val().parse().unwrap_or_else(|_| usage()),
            "--server" => {
                let s = val();
                let s = if s.contains(':') { s } else { format!("{s}:27500") };
                a.server = s
                    .to_socket_addrs()
                    .unwrap_or_else(|e| {
                        eprintln!("udptap: resolve {s}: {e}");
                        std::process::exit(1)
                    })
                    .next()
                    .unwrap_or_else(|| usage());
                have_server = true;
            }
            "--out" => a.out = Some(PathBuf::from(val())),
            "--loss" => a.loss = val().parse().unwrap_or_else(|_| usage()),
            "--dup" => a.dup = val().parse().unwrap_or_else(|_| usage()),
            "--delay" => a.delay = Duration::from_millis(val().parse().unwrap_or_else(|_| usage())),
            "-h" | "--help" => usage(),
            _ => usage(),
        }
        i += 2;
    }
    if !have_server {
        usage();
    }
    a
}

/// A tiny xorshift PRNG. The impairment doesn't need a good one, and this keeps the crate free of
/// a `rand` dependency it would otherwise pull in only for an example.
struct Rng(u64);

impl Rng {
    fn percent(&mut self, pct: u32) -> bool {
        if pct == 0 {
            return false;
        }
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 % 100) < pct as u64
    }
}

/// A packet held back by `--delay`, waiting for its release time.
struct Held {
    at: Instant,
    to_server: bool,
    data: Vec<u8>,
}

fn main() -> std::io::Result<()> {
    let args = parse_args();
    if let Some(dir) = &args.out {
        std::fs::create_dir_all(dir)?;
    }

    let down = UdpSocket::bind(("127.0.0.1", args.listen))?; // client side
    let up = UdpSocket::bind("0.0.0.0:0")?; // server side
    down.set_read_timeout(Some(Duration::from_millis(2)))?;
    up.set_read_timeout(Some(Duration::from_millis(2)))?;

    eprintln!(
        "udptap: 127.0.0.1:{} ⇄ {}{}{}",
        args.listen,
        args.server,
        args.out.as_ref().map(|d| format!("  → {}", d.display())).unwrap_or_default(),
        if args.loss > 0 || args.dup > 0 || !args.delay.is_zero() {
            format!("  [loss {}% dup {}% delay {:?}]", args.loss, args.dup, args.delay)
        } else {
            String::new()
        }
    );
    eprintln!("udptap: connect a client to 127.0.0.1:{} …", args.listen);

    let mut client: Option<SocketAddr> = None;
    let mut rng = Rng(0x2545_f491_4f6c_dd1d);
    let mut queue: VecDeque<Held> = VecDeque::new();
    let mut n = 0u32;
    let mut buf = [0u8; 8192];

    loop {
        // Client → server.
        if let Ok((len, from)) = down.recv_from(&mut buf) {
            if client != Some(from) {
                eprintln!("udptap: client {from}");
                client = Some(from);
            }
            n += 1;
            record(&args.out, n, true, &buf[..len])?;
            forward(&args, &mut rng, &mut queue, true, &buf[..len]);
        }

        // Server → client.
        if let Ok(len) = up.recv(&mut buf) {
            n += 1;
            record(&args.out, n, false, &buf[..len])?;
            forward(&args, &mut rng, &mut queue, false, &buf[..len]);
        }

        // Release anything whose delay has elapsed. The queue is in send order, and a uniform
        // delay preserves it — so a front-to-back drain is enough.
        let now = Instant::now();
        while queue.front().is_some_and(|h| h.at <= now) {
            let h = queue.pop_front().unwrap();
            if h.to_server {
                let _ = up.send_to(&h.data, args.server);
            } else if let Some(c) = client {
                let _ = down.send_to(&h.data, c);
            }
        }
    }
}

/// Queue a packet for forwarding, applying loss and duplication.
fn forward(args: &Args, rng: &mut Rng, queue: &mut VecDeque<Held>, to_server: bool, data: &[u8]) {
    if rng.percent(args.loss) {
        return;
    }
    let at = Instant::now() + args.delay;
    queue.push_back(Held { at, to_server, data: data.to_vec() });
    if rng.percent(args.dup) {
        queue.push_back(Held { at, to_server, data: data.to_vec() });
    }
}

/// Write one datagram as a fixture.
fn record(out: &Option<PathBuf>, n: u32, to_server: bool, data: &[u8]) -> std::io::Result<()> {
    let Some(dir) = out else { return Ok(()) };
    let dir: &Path = dir;
    let dir = dir.join(format!("{n:06}-{}.bin", if to_server { "c2s" } else { "s2c" }));
    std::fs::write(dir, data)
}
