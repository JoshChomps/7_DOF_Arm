#!/usr/bin/env python3
"""First-order strength of the printed links (docs/21 §6): bending stress at each link's weakest section.

    python tools/link_strength.py

Load: the gravity moment of everything a link carries (arm.toml masses + 1 kg at the TCP) about the link's root,
worst over random poses, split into the component about the plate's stiff in-plane axis and the rest (out of
plane), x 1.5 for acceleration. Stress = M / Z with Z = b h^2 / 6 of the section named in the table (mm, from
tools/fusion_arm_links.py). Allowable for printed PETG: 15 MPa (across layers, about half the in-layer
strength); "check" if over. Beam theory on plates with holes and no stress concentration: a screening, not FEA.
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "software"))
from deskarm import load_arm  # noqa: E402
from deskarm.kinematics import G  # noqa: E402

DYN, ALLOW, PAYLOAD = 1.5, 15.0, 1.0
# link, carried by (joint index), root point at zero pose (m, base frame), plate normal at zero pose,
# weakest section (width b along the plate, thickness h) mm, what it is
LINKS = [
    ("link 2 side plate", 1, (0, -0.048, 0.267), (0, 1, 0), (120, 8), "x -52..-44 above the J2 flange disc"),
    ("link 3 arm plate", 2, (0, -0.044, 0.3215), (0, 0, 1), (60, 12), "z 317.5..329.5, y +-30, x -43 (bolt circle)"),
    ("link 3 U-collar", 2, (0, -0.0845, 0.3535), (0, 1, 0), (124, 8), "below the J4 housing passage"),
    ("forearm side plate", 3, (0, -0.047, 0.420), (0, 1, 0), (86, 10), "x -52..-42 above the J4 flange"),
    ("link 5 arm", 4, (0, -0.040, 0.498), (0, 0, 1), (64, 8), "z 494..502 at the riser"),
    ("link 5 collar", 4, (0, -0.063, 0.520), (0, 1, 0), (92, 6), "x -66..-60 below the J6 passage"),
    ("link 6 side plate", 5, (0, -0.033, 0.600), (0, 1, 0), (64, 8), "x -37..-29 above the J6 flange"),
]


def moments(arm, link, root, normal, n=3000, seed=0):
    """Worst |M| about the plate's in-plane stiff axis and out of plane (N m), over random poses."""
    rng = np.random.default_rng(seed)
    L0 = arm.link_frames(np.zeros(arm.n))
    lumps = [(m.mass, m.link, np.append(m.pos, 1.0)) for m in arm.masses if m.link >= link]
    best = np.zeros(2)
    for _ in range(n):
        q = rng.uniform(arm.lower, arm.upper)
        Lq = arm.link_frames(q)
        T = lambda k: Lq[k] @ np.linalg.inv(L0[k])
        R = T(link)
        r0, nrm = (R @ np.append(root, 1.0))[:3], R[:3, :3] @ np.asarray(normal, float)
        M = np.zeros(3)
        for m, k, p in lumps:
            M += np.cross((T(k) @ p)[:3] - r0, [0, 0, -m * G])
        M += np.cross(arm.fk(q)[:3, 3] - r0, [0, 0, -PAYLOAD * G])
        out_of_plane = np.linalg.norm(M - (M @ nrm) * nrm)          # bends the plate across its thickness
        best = np.maximum(best, [abs(M @ nrm), out_of_plane])
    return best


def main():
    arm = load_arm()
    print(f"{'member':20s} {'M in / out (N m)':>17s} {'stress (MPa)':>13s}  section")
    for name, link, root, normal, (b, h), where in LINKS:
        m_in, m_out = moments(arm, link, root, normal)
        z_weak, z_strong = b * h * h / 6, h * b * b / 6
        s = DYN * (m_out * 1e3 / z_weak + m_in * 1e3 / z_strong)
        print(f"{name:20s} {m_in:7.1f} / {m_out:5.1f}   {s:9.1f} {'ok' if s <= ALLOW else 'CHECK':>5s}  {b} x {h}, {where}")


if __name__ == "__main__":
    main()
