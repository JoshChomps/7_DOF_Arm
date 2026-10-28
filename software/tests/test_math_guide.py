"""Runs every ``python`` example in docs/14_math_guide.md, so the guide can't drift from the code."""
import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
GUIDE = ROOT / "docs" / "14_math_guide.md"
TEXT = GUIDE.read_text(encoding="utf-8")
BLOCKS = re.findall(r"^## (\d+)\. [^\n]*\n(?:(?!^## ).)*?```python\n(.*?)```", TEXT, flags=re.S | re.M)


def test_guide_has_an_example_per_math_section():
    assert [int(n) for n, _ in BLOCKS] == list(range(1, 22))


@pytest.mark.parametrize("section, code", BLOCKS, ids=[f"section-{n}" for n, _ in BLOCKS])
def test_example_runs(section, code, monkeypatch, capsys):
    if "deskarm.physics" in code:
        pytest.importorskip("mujoco")
    monkeypatch.chdir(ROOT / "software")           # the examples are written to run from software/
    exec(compile(code, f"{GUIDE.name} section {section}", "exec"), {"__name__": "__guide__"})


def test_every_figure_the_guide_shows_exists():
    figures = re.findall(r"!\[[^\]]*\]\((img/math/[^)]+)\)", TEXT)
    assert len(figures) >= 20
    for rel in figures:
        assert (GUIDE.parent / rel).is_file(), rel


def test_modules_point_back_to_the_guide():
    """Each module with math names its section, and that section exists."""
    sections = {int(n) for n in re.findall(r"^## (\d+)\. ", TEXT, flags=re.M)}
    pkg = ROOT / "software" / "deskarm"
    for name in ("kinematics", "ik", "trajectory", "cartesian", "topp", "motor", "physics",
                 "counterbalance", "collision", "urdf", "backends", "bench", "encoders", "control"):
        refs = re.findall(r"docs/14_math_guide\.md §(\d+)", (pkg / f"{name}.py").read_text(encoding="utf-8"))
        assert refs, f"{name}.py does not reference the math guide"
        assert {int(r) for r in refs} <= sections


def test_math_survives_github_markdown():
    r"""GitHub strips a backslash before punctuation (so a double backslash ends no matrix row) and double-escapes < inside math.

    Use \cr, \thinspace, \lbrace, \lt instead. Checked against GitHub's renderer on 14 Oct 2026.
    """
    body = re.sub(r"```.*?```", "", TEXT, flags=re.S)
    display = re.findall(r"\$\$(.+?)\$\$", body, flags=re.S)
    inline = re.findall(r"(?<!\$)\$(?!\$)([^$\n]+?)\$(?!\$)", re.sub(r"\$\$.+?\$\$", "", body, flags=re.S))
    bad = [m for s in display + inline for m in re.findall(r"\[\,;!{}|_:>]|[<>]", s)]
    assert not bad, f"{len(bad)} GitHub-unsafe sequences in math, e.g. {bad[:5]}"
    assert all(not s.startswith(" ") and not s.endswith(" ") for s in inline), "inline math must not start or end with a space"
