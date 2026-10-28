"""Join Kairo-built one-sheet projects into one hierarchical KiCad project.

usage: combine.py out_dir project_name title sheetname=path/to/sheet.kicad_sch ...

Each sub-sheet is copied next to the top sheet, its symbol instance paths are rewritten
for the new project, and power-symbol references (#PWR, #FLG) are renumbered per sheet
so they stay unique. Kairo's own copies are left untouched.
"""
import re, sys, uuid, shutil, os

out, project, title = sys.argv[1:4]
sheets = [a.split("=", 1) for a in sys.argv[4:]]
os.makedirs(out, exist_ok=True)
top_uuid = str(uuid.uuid5(uuid.NAMESPACE_URL, f"deskarm/{project}/top"))

blocks = []
x = 30.0
for page, (name, path) in enumerate(sheets, start=2):
    text = open(path, encoding="utf-8").read()
    sheet_uuid = str(uuid.uuid5(uuid.NAMESPACE_URL, f"deskarm/{project}/{name}"))
    fname = f"{name}.kicad_sch"
    old_project = re.search(r'\(project "([^"]+)"', text).group(1)
    root = re.search(r'\(kicad_sch\s*\(version \d+\)\s*\(generator "[^"]*"\)\s*\(generator_version "[^"]*"\)\s*\(uuid "([^"]+)"\)', text).group(1)
    text = text.replace(f'(project "{old_project}"', f'(project "{project}"')
    text = text.replace(f'(path "/{root}"', f'(path "/{top_uuid}/{sheet_uuid}"')
    n = [0]
    def renum(m):
        n[0] += 1
        return f'"{m.group(1)}{page - 1}{n[0]:02d}"'
    text = re.sub(r'"(#PWR|#FLG)0*\d+"', renum, text)
    # the top sheet's sheet_instances is the only one KiCad reads; drop the copy's
    text = re.sub(r'\n\t\(sheet_instances\n\t\t\(path "/"\n\t\t\t\(page "1"\)\n\t\t\)\n\t\)', "", text)
    open(os.path.join(out, fname), "w", encoding="utf-8", newline="\n").write(text)
    blocks.append(f"""	(sheet
		(at {x} 40)
		(size 60 30)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(stroke
			(width 0.1524)
			(type solid)
		)
		(fill
			(color 0 0 0 0.0000)
		)
		(uuid "{sheet_uuid}")
		(property "Sheetname" "{name}"
			(at {x} 39.2 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify left bottom)
			)
		)
		(property "Sheetfile" "{fname}"
			(at {x} 70.6 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify left top)
			)
		)
		(instances
			(project "{project}"
				(path "/{top_uuid}"
					(page "{page}")
				)
			)
		)
	)
""")
    x += 80

top = f"""(kicad_sch
	(version 20260306)
	(generator "eeschema")
	(generator_version "10.0")
	(uuid "{top_uuid}")
	(paper "A4")
	(title_block
		(title "{title}")
		(comment 1 "Sub-sheets drawn and checked by Kairo one sheet at a time; joined by power nets (global)")
	)
	(lib_symbols)
{''.join(blocks)}	(sheet_instances
		(path "/"
			(page "1")
		)
	)
	(embedded_fonts no)
)
"""
open(os.path.join(out, f"{project}.kicad_sch"), "w", encoding="utf-8", newline="\n").write(top)
pro_src = sheets[0][1].replace(".kicad_sch", ".kicad_pro")
pro = open(pro_src, encoding="utf-8").read().replace(os.path.basename(pro_src), f"{project}.kicad_pro")
open(os.path.join(out, f"{project}.kicad_pro"), "w", encoding="utf-8", newline="\n").write(pro)
print("top", top_uuid, "sheets", [s[0] for s in sheets])
