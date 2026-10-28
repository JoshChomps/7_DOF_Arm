# Accuracy report: deskarm_board_a_power

Status: **verified**. 0 pass, 0 fail, 0 unverified.

A result is `pass` only when its worst case over every input's tolerance is proven. `unverified` means an input was missing; it is never a pass.

## To resolve

Nothing: every result is established.

## Results

| Check | Status | Found |
|---|---|---|

## Tolerances in use

| Part | Parameter | Tolerance | Source |
|---|---|---|---|
| R1 | value | 1% | generic 0603 1% thick film |
| R2 | value | 1% | generic 0603 1% thick film |
| C1 | value | 10% | user-stated; LCSC's catalogue entry C20616238 for AM03B104K500NT (read 2026-10-11, not checked against the datasheet): tolerance, v_rating, dielectric |
| R3 | value | 1% | generic 0603 1% thick film |
| R4 | value | 1% | generic 0603 1% thick film |
| R6 | value | 1% | generic 1206 1% thick film |
| D4 | Vf | 14.3% | user-stated; LCSC's catalogue entry C19273152 for YLED0603Y (read 2026-10-11, not checked against the datasheet): v_f, v_f_tolerance |
| R7 | value | 1% | 2512 2 W 1% current-sense resistor |
| C2 | value | 10% | user-stated; LCSC's catalogue entry C20616238 for AM03B104K500NT (read 2026-10-11, not checked against the datasheet): tolerance, v_rating, dielectric |
| C3 | value | 20% | user-stated |
| R8 | value | 1% | generic 1206 1% thick film |
| D6 | Vf | 12.2% | user-stated; LCSC's catalogue entry C7371900 for XL-1608SURC-06FJ (read 2026-10-11, not checked against the datasheet): v_f, v_f_tolerance |

## Project policies in use

- Margins: only tolerances the design states were used; Kairo's default margins are turned off.
