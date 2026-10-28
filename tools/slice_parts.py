#!/usr/bin/env python3
"""Slice every printed part with the Bambu Studio CLI and report time and filament (docs/21 §4).

    python tools/slice_parts.py [part ...]      # STLs from stl/da/ (Fusion: tools/fusion_run.py sweep stl)

Settings as docs/19 §9 (6 walls, 40 % gyroid, 6 top / bottom layers, tree supports at 40 deg, PETG HF), A1 mini for
parts within 170 mm, the H2D profile as a stand-in for the CORE One L otherwise. The stock profiles inherit from
parents, which the CLI doesn't resolve, so they are flattened into stl/da/profiles/ first. Results go to
stl/da/slice.json; the .3mf files are deleted again.
"""
import json
import re
import struct
import subprocess
import sys
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DA = ROOT / "stl" / "da"
BBL = Path(r"C:\Program Files\Bambu Studio\resources\profiles\BBL")
EXE = Path(r"C:\Program Files\Bambu Studio\bambu-studio.exe")
PROC = {"wall_loops": "6", "sparse_infill_density": "40%", "sparse_infill_pattern": "gyroid", "top_shell_layers": "6",
        "bottom_shell_layers": "6", "enable_support": "1", "support_type": "tree(auto)", "support_threshold_angle": "40",
        "curr_bed_type": "Textured PEI Plate"}
PRINTERS = {"A1M": ("Bambu Lab A1 mini 0.4 nozzle", "0.20mm Standard @BBL A1M", "Bambu PETG HF @BBL A1M", 170),
            "H2D": ("Bambu Lab H2D 0.4 nozzle", "0.20mm Standard @BBL H2D", "Bambu PETG HF @BBL H2D 0.4 nozzle", 300)}


def flat(kind, name):
    d = json.loads((BBL / kind / f"{name}.json").read_text(encoding="utf-8"))
    if "inherits" in d:
        base = flat(kind, d.pop("inherits"))
        base.update(d)
        d = base
    return d


def profiles(printer):
    out = DA / "profiles"
    out.mkdir(exist_ok=True)
    m, p, f, _ = PRINTERS[printer]
    paths = []
    for kind, name, extra in (("machine", m, {}), ("process", p, PROC), ("filament", f, {})):
        d = flat(kind, name)
        d.update(extra)
        d.pop("instantiation", None)
        path = out / f"{printer}_{kind}.json"
        path.write_text(json.dumps(d, indent=1), encoding="utf-8")
        paths.append(path)
    return paths


def extent(stl):
    """Bounding-box size of a binary STL, mm."""
    b = stl.read_bytes()
    n = struct.unpack_from("<I", b, 80)[0]
    lo, hi = [1e9] * 3, [-1e9] * 3
    for i in range(n):
        v = struct.unpack_from("<12f", b, 84 + 50 * i)[3:]
        for k in range(9):
            lo[k % 3], hi[k % 3] = min(lo[k % 3], v[k]), max(hi[k % 3], v[k])
    return sorted(hi[i] - lo[i] for i in range(3))


def slice_one(stl, printer, orient="1"):
    m, p, f = profiles(printer)
    out = DA / "out"
    out.mkdir(exist_ok=True)
    cmd = [str(EXE), "--orient", orient, "--arrange", "1", "--load-settings", f"{m};{p}", "--load-filaments", str(f),
           "--slice", "0", "--export-3mf", f"{stl.stem}.3mf", "--outputdir", str(out), str(stl)]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=900)
    mf = out / f"{stl.stem}.3mf"
    if not mf.exists():
        return {"error": (r.stdout + r.stderr).strip().splitlines()[-1:]}
    with zipfile.ZipFile(mf) as z:
        info = z.read("Metadata/slice_info.config").decode()
    mf.unlink()
    get = lambda key: re.search(rf'key="{key}" value="([^"]*)"', info)
    warn = re.findall(r'warning msg="([^"]+)"', info)
    return {"printer": printer, "time_h": round(float(get("prediction").group(1)) / 3600, 2),
            "grams": float(get("weight").group(1)), "supports": get("support_used").group(1) == "true",
            "warnings": [w for w in warn if w != "bed_temperature_too_high_than_filament"]}


def main(names):
    res_path = DA / "slice.json"
    res = json.loads(res_path.read_text()) if res_path.exists() else {}
    for stl in sorted(DA.glob("*.stl")):
        if names and stl.stem not in names:
            continue
        size = extent(stl)
        printer = "A1M" if size[-1] <= PRINTERS["A1M"][3] else "H2D"
        r = slice_one(stl, printer)
        for p, o in ((printer, "0"), ("H2D", "1")):     # auto-orient can tilt a part out of the A1 mini's volume
            if "error" not in r:
                break
            r = dict(slice_one(stl, p, o), note=f"auto-orient failed; {p}, orient {o}")
        res[stl.stem] = dict(r, size=[round(v) for v in size])
        print(stl.stem, res[stl.stem], flush=True)
        res_path.write_text(json.dumps(res, indent=1))


if __name__ == "__main__":
    main(sys.argv[1:])
