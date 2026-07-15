// Temporary: how much of the navmesh build is the curl-jump pass?
use rtx_nav::navmesh::{build_navmesh, SpeedJumpParams};

fn params(curl: bool) -> SpeedJumpParams {
    SpeedJumpParams { gravity: 800.0, accel: 10.0, maxspeed: 320.0, friction: 4.0, stopspeed: 100.0, curl }
}

#[test]
fn curl_pass_cost() {
    for map in ["bravado", "aerowalk", "race2"] {
        let Ok(bytes) = std::fs::read(format!("../../playground/qw/maps/{map}.bsp")) else {
            continue;
        };
        let mut times = Vec::new();
        for curl in [false, true] {
            let t = std::time::Instant::now();
            let (_b, g) = build_navmesh(bytes.clone(), vec![], vec![], vec![], None, true, Some(params(curl)), None)
                .expect("built");
            times.push((t.elapsed().as_secs_f64(), g.summary().speed_jump));
        }
        let (off, sj_off) = times[0];
        let (on, sj_on) = times[1];
        eprintln!(
            "{map:10} curl OFF {off:6.2}s ({sj_off} sj) | curl ON {on:6.2}s ({sj_on} sj) | curl pass = {:5.2}s ({:.1}x)",
            on - off,
            on / off.max(0.001)
        );
    }
}
