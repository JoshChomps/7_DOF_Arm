"""Split a whole-board Kairo design into per-sheet designs.

usage: split.py whole.json spec.json outdir
spec: {"sheet_name": {"refs": [...], "blocks": [...], "requirements": [indices], "loads": [indices]}}
Signal nets must stay inside one sheet (Kairo draws them as local labels); power and
ground nets may cross (global labels / power symbols).
"""
import json, sys, os

whole = json.load(open(sys.argv[1]))
spec = json.load(open(sys.argv[2]))
out = sys.argv[3]

owner = {}
for sheet, s in spec.items():
    for r in s["refs"]:
        assert r not in owner, f"{r} on two sheets"
        owner[r] = sheet
missing = set(whole["components"]) - set(owner)
assert not missing, f"unplaced: {missing}"

for net, pins in whole["connections"].items():
    sheets = {owner[p.split(".")[0]] for p in pins}
    if len(sheets) > 1 and whole["nets"][net]["role"] == "signal":
        raise SystemExit(f"signal net {net} crosses sheets {sheets}")

for sheet, s in spec.items():
    refs = set(s["refs"])
    conns = {}
    for net, pins in whole["connections"].items():
        mine = [p for p in pins if p.split(".")[0] in refs]
        if mine:
            conns[net] = mine
    d = {
        "design": f"{whole['design']}_{sheet}",
        "policy": whole.get("policy", {}),
        "nets": {n: whole["nets"][n] for n in conns},
        "components": {r: whole["components"][r] for r in whole["components"] if r in refs},
        "connections": conns,
        "no_connect": [p for p in whole.get("no_connect", []) if p.split(".")[0] in refs],
        "loads": [whole["loads"][i] for i in s.get("loads", [])],
        "requirements": [whole["requirements"][i] for i in s.get("requirements", [])],
        "blocks": {b: whole["blocks"][b] for b in s["blocks"]},
        "placement": {"flow": "left_to_right", "order": s["blocks"]},
    }
    for b, members in d["blocks"].items():
        assert set(members) <= refs, (sheet, b)
    json.dump(d, open(os.path.join(out, f"{sheet}.json"), "w"))
    print(sheet, len(d["components"]), "parts,", len(json.dumps(d)), "chars")
