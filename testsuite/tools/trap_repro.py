#!/usr/bin/env python3
"""Deterministic recovery drill for un-carved standable surfaces (`nav_patch` acceptance).

One trial: teleport a puppet bot onto the surface, hand it a `goto` to a floor target, and watch
at ~10 Hz whether it gets off (``escaped``, with time-to-escape) or stands and grinds displacement
stalls until the window closes (``stuck``). The drill is the paired on/off acceptance test for a
`nav_patch` entry — run it twice against the same binary, once with the patch applied and once
with the server started under `rtx_nav_patch 0`:

    python3 tools/trap_repro.py --port 27994 --trials 8

Exit status is non-zero when any trial sticks, so `patch on` is a green run and `patch off` is the
control that proves the trap still exists without it. The drill never touches `rtx_nav_patch`
itself — which graph is live is the rig operator's choice, and the server log's
``rtx: navpatch ...`` line is the ground truth for it.

Defaults target the dm3 west shelf (see `nav_patch::PATCHES` in `rtx-game`), measured on the
patch branch's acceptance run, same binary both arms: patch off, 4/4 trials stuck over 40 s
windows with 5-8 displacement stalls each on the floor walk links 104 u below
(769/771/1029/1032); patch on, 0/8 stuck with a median 0.63 s escape (span 0.51-1.00).
`--surf`/`--target` retarget any other patched surface.

Rig prep mirrors `runner/t2.py`: `rtx_telemetry 1` (BotStall events), `rtx_bot_pacifist 1`,
`rtx_bot_count 1` (one puppet, no interference), all restored on exit. Events piggyback on the
`status` polls via `runner.control.Control`, the same client the rest of the suite uses.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from runner.control import Control, ControlError  # noqa: E402


def hdist(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def bot_row(status: dict, ent: int) -> dict | None:
    return next((b for b in status.get("bots", []) if b.get("ent") == ent), None)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, required=True, help="control-channel TCP port")
    ap.add_argument("--trials", type=int, default=8)
    ap.add_argument("--secs", type=float, default=40.0, help="watch window per trial")
    ap.add_argument("--surf", type=float, nargs=3, default=[-865.0, -48.0, 90.0],
                    help="teleport aim point on the surface under test")
    ap.add_argument("--target", type=float, nargs=3, default=[-864.0, -96.0, -16.0],
                    help="goto target on the carved floor")
    ap.add_argument("--escape-z", type=float, default=40.0,
                    help="a bot below this z has left the surface")
    ap.add_argument("--out", default="/tmp/trap-repro.jsonl")
    args = ap.parse_args()

    ctl = Control(args.host, args.port)

    surf = list(args.surf)
    target = list(args.target)
    results = []
    saved: dict[str, str | None] = {}
    try:
        # Rig prep inside the same try as the run: a failure on the second or third `set` must
        # still restore the first one.
        for name, value in [("rtx_telemetry", "1"), ("rtx_bot_pacifist", "1"),
                            ("rtx_bot_count", "1")]:
            try:
                saved[name] = str(ctl.request(f"get {name}")["data"].get("value"))
            except (ControlError, AttributeError, KeyError):
                saved[name] = None
            ctl.request(f"set {name} {value}")
        run_trials(ctl, args, surf, target, results)
    finally:
        # The docstring promises restoration on exit — including sys.exit mid-run, a lost
        # control connection, or ^C. Each cvar is restored independently so one failure
        # doesn't strand the rest.
        for name, value in saved.items():
            if value is not None:
                try:
                    ctl.request(f"set {name} {value}")
                except (ControlError, OSError):
                    continue

    stuck = sum(1 for r in results if not r["verdict"].startswith("escaped"))
    print(f"SUMMARY: {stuck}/{len(results)} not escaped; "
          f"verdicts={[r['verdict'] for r in results]}", file=sys.stderr)
    sys.exit(1 if stuck or not results else 0)


def run_trials(ctl: Control, args, surf: list, target: list, results: list) -> None:
    for _ in range(120):
        status = ctl.request("status")["data"]
        if status.get("navmesh") == "ready" and status.get("bots"):
            break
        time.sleep(1.0)
    else:
        sys.exit("navmesh/bot never ready")
    ent = status["bots"][0]["ent"]

    with open(args.out, "a") as outf:
        for i in range(args.trials):
            deadline = time.monotonic() + 20
            while True:
                b = bot_row(ctl.request("status")["data"], ent)
                if b and b.get("alive") and (b.get("health") or 0) > 0:
                    break
                if time.monotonic() > deadline:
                    sys.exit(f"bot {ent} never came alive")
                time.sleep(0.5)
            ctl.request(f"prep {ent} 100 0")
            ctl.request(f"teleport {ent} {surf[0]} {surf[1]} {surf[2]}")
            time.sleep(0.6)
            settle = list(bot_row(ctl.request("status")["data"], ent)["origin"])
            if settle[2] < args.escape_z + 15 or hdist(settle, surf) > 80:
                row = {"trial": i, "verdict": "no_stand", "settle": settle}
                print(json.dumps(row))
                outf.write(json.dumps(row) + "\n")
                outf.flush()
                results.append(row)
                continue

            ctl.events.clear()
            ctl.request(f"goto {ent} {target[0]} {target[1]} {target[2]}")
            t0 = time.monotonic()
            stalls, verdict, t_escape, traj = [], "stuck", None, []
            while time.monotonic() - t0 < args.secs:
                b = bot_row(ctl.request("status")["data"], ent)
                now = time.monotonic() - t0
                o = b["origin"]
                traj.append([round(now, 2), round(o[0], 1), round(o[1], 1), round(o[2], 1),
                             round(b.get("speed", 0), 1), b.get("on_ground")])
                arrived = False
                for ev in ctl.events:
                    if ev.get("bot") not in (None, ent):
                        continue
                    if ev.get("ev") == "bot_stall":
                        stalls.append({**ev, "rel_t": round(now, 2)})
                    elif ev.get("ev") == "arrived":
                        arrived = True
                ctl.events.clear()
                if arrived:
                    verdict, t_escape = "escaped_arrived", now
                    break
                if o[2] < args.escape_z:
                    verdict, t_escape = "escaped", now
                    break
                time.sleep(0.08)
            ctl.request(f"stop {ent}")
            row = {"trial": i, "verdict": verdict, "t_escape": t_escape, "settle": settle,
                   "n_stalls": len(stalls),
                   "stall_reasons": sorted({s.get("reason") for s in stalls}),
                   "stall_links": sorted({s.get("link") for s in stalls}),
                   "end": traj[-1] if traj else None, "stalls": stalls, "traj_tail": traj[-8:]}
            print(json.dumps({k: row[k] for k in
                              ("trial", "verdict", "t_escape", "settle", "n_stalls",
                               "stall_reasons", "stall_links", "end")}))
            outf.write(json.dumps(row) + "\n")
            outf.flush()
            results.append(row)
            time.sleep(1.0)


if __name__ == "__main__":
    main()
