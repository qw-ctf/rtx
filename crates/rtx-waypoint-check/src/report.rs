// SPDX-License-Identifier: AGPL-3.0-or-later

//! Formatting the coverage report: one line per authored path, a per-map roll-up, and a grand total.

use glam::Vec3;
use rtx_nav::navmesh::LinkKind;

use crate::botfile::{MarkerPos, ResolvedPath};
use crate::check::{Family, Verdict};

/// Per-family verdict counts.
#[derive(Default, Clone, Copy)]
pub struct Tally {
    pub matched: u32,
    pub jump: u32,
    pub route: u32,
    pub unreach: u32,
    pub unsnap: u32,
}

impl Tally {
    pub fn add(&mut self, v: &Verdict) {
        match v {
            Verdict::Matched(_) => self.matched += 1,
            Verdict::JumpConnected(_) => self.jump += 1,
            Verdict::RouteConnected { .. } => self.route += 1,
            Verdict::Unreachable { .. } => self.unreach += 1,
            Verdict::Unsnapped { .. } => self.unsnap += 1,
        }
    }

    pub fn total(&self) -> u32 {
        self.matched + self.jump + self.route + self.unreach + self.unsnap
    }

    /// Paths that are a genuine blind spot (no route at all, or an endpoint off the mesh).
    pub fn holes(&self) -> u32 {
        self.unreach + self.unsnap
    }

    pub fn merge(&mut self, o: &Tally) {
        self.matched += o.matched;
        self.jump += o.jump;
        self.route += o.route;
        self.unreach += o.unreach;
        self.unsnap += o.unsnap;
    }

    /// One-line summary, e.g. `23: 14 matched, 3 jump, 5 route, 1 unreach`.
    pub fn line(&self) -> String {
        let mut parts = Vec::new();
        if self.matched > 0 {
            parts.push(format!("{} matched", self.matched));
        }
        if self.jump > 0 {
            parts.push(format!("{} jump", self.jump));
        }
        if self.route > 0 {
            parts.push(format!("{} route", self.route));
        }
        if self.unreach > 0 {
            parts.push(format!("{} unreach", self.unreach));
        }
        if self.unsnap > 0 {
            parts.push(format!("{} unsnap", self.unsnap));
        }
        if parts.is_empty() {
            parts.push("none".into());
        }
        format!("{}: {}", self.total(), parts.join(", "))
    }
}

/// One report line for a classified path.
pub fn path_line(fam: Family, p: &ResolvedPath, v: &Verdict) -> String {
    let fam_s = match fam {
        Family::RocketJump => "rj",
        Family::Curl => "curl",
    };
    let params = match fam {
        Family::RocketJump => {
            let rj = p.rj.unwrap_or(crate::botfile::RjFields {
                pitch: 0.0,
                yaw: 0.0,
                delay: 0.0,
            });
            // yaw <= 0 is KTX's "keep the current yaw" sentinel, not an angle — leave it be.
            let yaw = if rj.yaw <= 0.0 {
                "keep-yaw".to_string()
            } else {
                format!("y{:.0}", anglemod(rj.yaw))
            };
            format!("p{:.1} {} d{}", anglemod(rj.pitch), yaw, rj.delay as i32)
        }
        Family::Curl => format!("hint {}", p.angle_hint),
    };
    format!(
        "  {:<4} {:>3}->{:<3} {} -> {}   {:<20}  {}",
        fam_s,
        p.src,
        p.dst,
        endpoint(&p.from),
        endpoint(&p.to),
        params,
        verdict(v),
    )
}

fn endpoint(m: &MarkerPos) -> String {
    match m {
        MarkerPos::Entity { classname, brush, pos } => {
            format!("{}{}{}", classname, if *brush { "~" } else { "" }, pos_str(*pos))
        }
        MarkerPos::Created(p) => pos_str(*p),
    }
}

fn pos_str(v: Vec3) -> String {
    format!("({} {} {})", v.x as i32, v.y as i32, v.z as i32)
}

/// Wrap an angle to `[0, 360)`, mirroring KTX's `anglemod` (`mathlib.c`) — the view angle a bot
/// actually holds when firing. An authored pitch of e.g. `770` reads as its effective `50°` rather
/// than raw garbage; a real yaw is shown as-turned. The `.bot` file may store the un-wrapped value.
fn anglemod(a: f32) -> f32 {
    a.rem_euclid(360.0)
}

fn verdict(v: &Verdict) -> String {
    match v {
        Verdict::Matched(n) => format!(
            "MATCHED  {} link {}  {:.0}/{:.0}",
            kind_short(n.kind),
            n.link,
            n.d_src,
            n.d_tgt
        ),
        Verdict::JumpConnected(n) => {
            format!(
                "JUMP     {} link {}  {:.0}/{:.0}",
                kind_short(n.kind),
                n.link,
                n.d_src,
                n.d_tgt
            )
        }
        Verdict::RouteConnected {
            cost,
            legs,
            jump_legs,
            degenerate,
        } => {
            if *degenerate {
                "ROUTE*   penalty-priced (plat/gate/chained only)".to_string()
            } else {
                format!("ROUTE    {cost:.1}s, {legs} legs ({jump_legs} air)")
            }
        }
        Verdict::Unreachable { nearest_kindred } => match nearest_kindred {
            Some(n) => format!(
                "UNREACH  nearest {} d {:.0}/{:.0}",
                kind_short(n.kind),
                n.d_src,
                n.d_tgt
            ),
            None => "UNREACH  (no same-kind links on map)".to_string(),
        },
        Verdict::Unsnapped { end, dist } => {
            if dist.is_finite() {
                format!("UNSNAP   {end} off mesh by {dist:.0}")
            } else {
                format!("UNSNAP   {end} off mesh")
            }
        }
    }
}

fn kind_short(k: LinkKind) -> &'static str {
    match k {
        LinkKind::Walk => "walk",
        LinkKind::Step => "step",
        LinkKind::Drop => "drop",
        LinkKind::JumpGap => "jump",
        LinkKind::DoubleJump => "djump",
        LinkKind::SpeedJump => "sjump",
        LinkKind::Plat => "plat",
        LinkKind::Teleport => "tele",
        LinkKind::Hook => "hook",
        LinkKind::RocketJump => "rj",
    }
}
