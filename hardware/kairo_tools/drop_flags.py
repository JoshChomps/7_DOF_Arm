"""Remove top-level symbols (by reference) from a KiCad schematic copy.

usage: drop_flags.py file.kicad_sch REF [REF ...]
Used to drop a PWR_FLAG that a second Kairo sheet also places on a shared rail.
"""
import re, sys

path, refs = sys.argv[1], set(sys.argv[2:])
text = open(path, encoding="utf-8").read()
out, i, dropped = [], 0, []
for m in re.finditer(r"\n\t\(symbol\n", text):
    start = m.start()
    if start < i:
        continue
    depth, j = 0, start + 1
    while True:
        c = text[j]
        if c == "(":
            depth += 1
        elif c == ")":
            depth -= 1
            if depth == 0:
                break
        j += 1
    block = text[start:j + 1]
    ref = re.search(r'\(reference "([^"]+)"\)', block)
    if ref and ref.group(1) in refs:
        out.append(text[i:start])
        i = j + 1
        dropped.append(ref.group(1))
out.append(text[i:])
open(path, "w", encoding="utf-8", newline="\n").write("".join(out))
print("dropped", dropped)
assert set(dropped) == refs, refs - set(dropped)
