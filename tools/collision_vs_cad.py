#!/usr/bin/env python3
"""Self-collision capsules (software/deskarm/collision.py) against Fusion's interference check on DA_ARM (docs/21 §5).

    python tools/collision_vs_cad.py [n_random] [seed]     # Fusion with the MCP add-in running

Poses: n random ones inside the joint limits plus a few folded ones. Truth = Fusion finds an overlap over 1 mm3.
A hit on the base board counts as caught if the planner's desk check (ik.workspace_violations) flags the pose.
A miss (CAD hits, capsules clear) is the dangerous kind; a false alarm only costs workspace. Results are written
to tools/collision_vs_cad.json and summarised in a few lines.
"""
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "software"))
from deskarm import load_arm  # noqa: E402
from deskarm.collision import self_collisions  # noqa: E402
from deskarm.ik import workspace_violations  # noqa: E402

PY = ROOT / "software" / ".venv" / "Scripts" / "python.exe"
FOLDED = [[0, -30, 0, 120, 0, 90, 0], [0, 0, 0, 120, 0, 120, 0], [0, 0, 0, 120, 0, -120, 0], [0, 90, 0, -120, 0, 0, 0],
          [0, 60, 90, 120, 0, 90, 0], [0, -60, -90, 110, 90, 100, 0], [0, 120, 0, 0, 0, 0, 0], [0, -120, 0, 0, 0, 0, 0]]
BATCH = 2                                              # the add-in's HTTP call times out after ~130 s
CACHE = ROOT / "tools" / "collision_vs_cad_cache.json"   # Fusion results by pose, so a timeout loses one batch


def cad(poses):
    cache = json.loads(CACHE.read_text()) if CACHE.exists() else {}
    key = lambda q: json.dumps([round(v, 1) for v in q])
    todo = [q for q in poses if key(q) not in cache]
    for i in range(0, len(todo), BATCH):
        js = json.dumps([{f"J{k + 1}": round(v, 1) for k, v in enumerate(q) if v} for q in todo[i:i + BATCH]])
        r = subprocess.run([str(PY), str(ROOT / "tools" / "fusion_run.py"), "sweep", js], capture_output=True, text=True)
        lines = [ln for ln in r.stdout.splitlines() if ln.startswith("{")]
        if len(lines) != len(todo[i:i + BATCH]):
            print("  batch failed (re-run to retry):", r.stdout[-200:].strip().splitlines()[-1:])
            continue
        for q, ln in zip(todo[i:i + BATCH], lines):
            hits = ln.split("} ", 1)[1]
            d = {} if hits == "clear" else eval(hits)              # Fusion prints a dict repr
            cache[key(q)] = {k: v for k, v in d.items() if v > 1.0}
        CACHE.write_text(json.dumps(cache))
    return [cache.get(key(q)) for q in poses]


def main(n=48, seed=1):
    arm = load_arm()
    rng = np.random.default_rng(seed)
    lo, hi = np.degrees(arm.lower), np.degrees(arm.upper)
    hi[6] = lo[6] = 0                                              # J7 spins nothing into anything
    poses = [list(np.round(rng.uniform(lo, hi), 1)) for _ in range(n)] + FOLDED
    truth = cad(poses)
    rows, miss, alarm = [], 0, 0
    for q, t in zip(poses, truth):
        if t is None:
            continue
        c = self_collisions(arm, np.radians(q), margin=0.0)
        if any("DA_B_Board" in k for k in t):          # the desk / board: the planner's desk check, not the capsules
            c = c + [f"desk: {w}" for w in workspace_violations(arm, np.radians(q))]
        miss += bool(t) and not c
        alarm += bool(c) and not t
        rows.append({"q": q, "cad": t, "capsules": c})
    (ROOT / "tools" / "collision_vs_cad.json").write_text(json.dumps(rows, indent=0))
    both = sum(bool(r["cad"]) and bool(r["capsules"]) for r in rows)
    print(f"{len(rows)} poses: CAD hits {sum(bool(r['cad']) for r in rows)}, caught {both}, MISSED {miss}, "
          f"false alarms {alarm}")
    for r in rows:
        if r["cad"] and not r["capsules"]:
            print("  missed", r["q"], list(r["cad"])[:3])


if __name__ == "__main__":
    main(*[int(a) for a in sys.argv[1:]])
