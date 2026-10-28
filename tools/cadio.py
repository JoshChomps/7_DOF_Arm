"""Tiny file writers shared by the design tools: DXF (R12 entities) and binary STL."""
import struct
from pathlib import Path


def write_dxf(path, entities):
    """entities: list of (layer, kind, data). kind 'polyline' -> list of (x, y) (closed),
    kind 'circle' -> ((x, y), r). Units are mm."""
    out = ["0", "SECTION", "2", "ENTITIES"]
    for layer, kind, data in entities:
        if kind == "polyline":
            out += ["0", "POLYLINE", "8", layer, "66", "1", "70", "1", "10", "0", "20", "0", "30", "0"]
            for x, y in data:
                out += ["0", "VERTEX", "8", layer, "10", f"{x:.5f}", "20", f"{y:.5f}", "30", "0"]
            out += ["0", "SEQEND", "8", layer]
        elif kind == "circle":
            (x, y), r = data
            out += ["0", "CIRCLE", "8", layer, "10", f"{x:.5f}", "20", f"{y:.5f}", "30", "0", "40", f"{r:.5f}"]
        else:
            raise ValueError(kind)
    out += ["0", "ENDSEC", "0", "EOF"]
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(out) + "\n")


def write_stl(path, manifold, name="deskarm"):
    """Binary STL from a manifold3d.Manifold."""
    mesh = manifold.to_mesh()
    verts = mesh.vert_properties[:, :3]
    tris = mesh.tri_verts
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "wb") as f:
        f.write(name.encode()[:80].ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for a, b, c in tris:
            p0, p1, p2 = verts[a], verts[b], verts[c]
            ux, uy, uz = p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]
            vx, vy, vz = p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]
            n = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx)
            f.write(struct.pack("<12fH", *n, *p0, *p1, *p2, 0))
