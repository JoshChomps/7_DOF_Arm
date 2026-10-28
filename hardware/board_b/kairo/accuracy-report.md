# Accuracy report: deskarm_board_b

Status: **unverified**. 3 pass, 0 fail, 3 unverified.

A result is `pass` only when its worst case over every input's tolerance is proven. `unverified` means an input was missing; it is never a pass.

## To resolve

- Run simulate with an op analysis for the current through R1 (resolves 2 results: requirement 2: max_junction_temp (U4), regulator_load (U4))
- Run simulate with an op analysis for the current through R2 (resolves 2 results: requirement 2: max_junction_temp (U4), regulator_load (U4))
- Run simulate with an op analysis for the current through R7 (resolves 2 results: requirement 2: max_junction_temp (U4), regulator_load (U4))
- Run simulate with an op analysis for the current through R9 (resolves 2 results: requirement 2: max_junction_temp (U4), regulator_load (U4))
- Run simulate with an op analysis for the current through R10 (resolves 2 results: requirement 2: max_junction_temp (U4), regulator_load (U4))
- Run simulate with an op analysis for the current through R11 (resolves 2 results: requirement 2: max_junction_temp (U4), regulator_load (U4))
- Run simulate with an op analysis for the current through R12 (resolves 2 results: requirement 2: max_junction_temp (U4), regulator_load (U4))
- Run simulate with an op analysis for the current through R13 (resolves 2 results: requirement 2: max_junction_temp (U4), regulator_load (U4))
- Run simulate with an op analysis for the current through R14 (resolves 2 results: requirement 2: max_junction_temp (U4), regulator_load (U4))
- Run simulate with an op analysis for the current through R15 (resolves 2 results: requirement 2: max_junction_temp (U4), regulator_load (U4))
- Give the tolerance of Iout, with where it comes from (resolves 1 result: requirement 2: max_junction_temp (U4))
- If U1's i_supply is its datasheet maximum, list it under limits as max; if not, give the maximum (resolves 1 result: requirement 2: max_junction_temp (U4))
- Give U4's least output capacitance (c_out_min) and the lowest and highest ESR it is stable with (c_out_esr_min, c_out_esr_max) from its datasheet (resolves 1 result: regulator_stability (U4))

## Results

| Check | Status | Found |
|---|---|---|
| requirement 1: rail_voltage (+3V3) | pass | +3V3 is 3.25 V to 3.349 V (U4: 3.3 V +/-1.5%); allowed is 3.135 V to 3.465 V |
| requirement 2: max_junction_temp (U4) | unverified | U4's junction is at least 62.1 degC; a lower bound cannot show the 125 degC limit is respected |
| requirement 3: min_capacitance (+3V3) | pass | C5, C1, C2, C6 give 20.2 uF nominal (marked values, as asked) between +3V3 and GND; 1 uF is required |
| requirement 4: max_component_power (R16) | pass | R16 dissipates 0.302 mW at nominal; the limit is 50 mW (50% of 100 mW) |
| regulator_load (U4) | unverified | U4 supplies at least 135.4 mA on +3V3; it is rated for 600 mA |
| regulator_stability (U4) | unverified | U4's stable region is not given, so its output capacitor cannot be checked against it |

## Tolerances in use

| Part | Parameter | Tolerance | Source |
|---|---|---|---|
| R1 | value | 1% | generic 0603 1% thick film |
| C3 | value | 10% | user-stated; LCSC's catalogue entry C33546005 for HGC0603R7105K250NTHJ (read 2026-10-11, not checked against the datasheet): tolerance, v_rating, dielectric |
| R2 | value | 1% | generic 0603 1% thick film |
| R3 | value | 1% | generic 0603 1% thick film |
| R4 | value | 1% | generic 0603 1% thick film |
| R6 | value | 1% | generic 1206 1% thick film |
| R7 | value | 1% | generic 0603 1% thick film |
| R8 | value | 1% | generic 0603 1% thick film |
| C7 | value | 10% | user-stated; LCSC's catalogue entry C20616238 for AM03B104K500NT (read 2026-10-11, not checked against the datasheet): tolerance, v_rating, dielectric |
| R9 | value | 1% | generic 0603 1% thick film |
| R10 | value | 1% | generic 0603 1% thick film |
| R11 | value | 1% | generic 0603 1% thick film |
| R12 | value | 1% | generic 0603 1% thick film |
| R13 | value | 1% | generic 0603 1% thick film |
| R14 | value | 1% | generic 0603 1% thick film |
| R15 | value | 1% | generic 0603 1% thick film |
| U4 | Vout | 1.5% | Diodes AP2112 datasheet DS39724 Rev. 2-2 (June 2017): p.1 output accuracy 1.5% and stable with 1.0 uF ceramic; p.8 (3.3 V) maximum output current 600 mA min, dropout 400 mV max at 600 mA, quiescent current 80 uA max, thetaJA 96 degC/W (SOT25) |
| C4 | value | 10% | user-stated; LCSC's catalogue entry C19103847 for HGC0805R5106K250NSLJ (read 2026-10-11, not checked against the datasheet): tolerance, v_rating, dielectric |
| C5 | value | 10% | user-stated; LCSC's catalogue entry C19103847 for HGC0805R5106K250NSLJ (read 2026-10-11, not checked against the datasheet): tolerance, v_rating, dielectric |
| C1 | value | 10% | user-stated; LCSC's catalogue entry C19103847 for HGC0805R5106K250NSLJ (read 2026-10-11, not checked against the datasheet): tolerance, v_rating, dielectric |
| C2 | value | 10% | user-stated; LCSC's catalogue entry C20616238 for AM03B104K500NT (read 2026-10-11, not checked against the datasheet): tolerance, v_rating, dielectric |
| C6 | value | 10% | user-stated; LCSC's catalogue entry C20616238 for AM03B104K500NT (read 2026-10-11, not checked against the datasheet): tolerance, v_rating, dielectric |
| C8 | value | 10% | user-stated; LCSC's catalogue entry C20616238 for AM03B104K500NT (read 2026-10-11, not checked against the datasheet): tolerance, v_rating, dielectric |
| R16 | value | 1% | generic 0603 1% thick film |
| D4 | Vf | 5.46% | user-stated; LCSC's catalogue entry C7496818 for F.0603.00025/P2-0603G1TS2-06T-002 (read 2026-10-11, not checked against the datasheet): v_f, v_f_tolerance |

## Project policies in use

- Margins: only tolerances the design states were used; Kairo's default margins are turned off.
- Thermal: a junction temperature is reckoned with the datasheet's theta_JA, a typical value on the JEDEC test board, plus 20%, a margin this project chose. A pass under it is not a pass against a datasheet limit.
