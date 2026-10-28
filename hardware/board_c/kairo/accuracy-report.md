# Accuracy report: deskarm_board_c

Status: **unverified**. 4 pass, 0 fail, 8 unverified.

A result is `pass` only when its worst case over every input's tolerance is proven. `unverified` means an input was missing; it is never a pass.

## To resolve

- Run simulate with an op analysis for the current through R1 (resolves 2 results: requirement 7: max_junction_temp (U4), regulator_load (U4))
- Run simulate with an op analysis for the current through R2 (resolves 2 results: requirement 7: max_junction_temp (U4), regulator_load (U4))
- Run simulate with an op analysis for the current through R3 (resolves 2 results: requirement 7: max_junction_temp (U4), regulator_load (U4))
- Run simulate with an op analysis for the current through R4 (resolves 2 results: requirement 7: max_junction_temp (U4), regulator_load (U4))
- Run simulate with an op analysis for the current through R10 (resolves 2 results: requirement 7: max_junction_temp (U4), regulator_load (U4))
- Run simulate with an op analysis for the current through R11 (resolves 2 results: requirement 7: max_junction_temp (U4), regulator_load (U4))
- Run simulate with an op analysis for the current through R12 (resolves 2 results: requirement 7: max_junction_temp (U4), regulator_load (U4))
- Say what sets +5V, with its output voltage and tolerance (resolves 1 result: requirement 1: rail_voltage (+5V))
- Connect L1 from the regulator's switch pin to its output rail, and set the regulator's regulator parameter to switching (resolves 1 result: requirement 3: max_inductor_current (L1))
- Add the manufacturer's DC-bias curve for HHV1206R7475K101NSPJ to the parts folder (resolves 1 result: requirement 4: min_capacitance (VIN_C))
- Add the manufacturer's DC-bias curve for AM03B104K500NT to the parts folder (resolves 1 result: requirement 4: min_capacitance (VIN_C))
- Give the tolerance of Iout, with where it comes from (resolves 1 result: requirement 7: max_junction_temp (U4))
- If U1's i_supply is its datasheet maximum, list it under limits as max; if not, give the maximum (resolves 1 result: requirement 7: max_junction_temp (U4))
- Set U5's regulator parameter to linear, or state its dissipation (resolves 1 result: requirement 8: max_junction_temp (U5))
- Kairo works out a resistor's power when it runs from a rail to ground, alone or through an LED; state R5's dissipation otherwise (resolves 1 result: requirement 9: max_component_power (R5))
- Give U4's least output capacitance (c_out_min) and the lowest and highest ESR it is stable with (c_out_esr_min, c_out_esr_max) from its datasheet (resolves 1 result: regulator_stability (U4))

## Results

| Check | Status | Found |
|---|---|---|
| requirement 1: rail_voltage (+5V) | unverified | nothing in the design sets +5V's voltage |
| requirement 2: rail_voltage (+3V3) | pass | +3V3 is 3.25 V to 3.349 V (U4: 3.3 V +/-1.5%); allowed is 3.135 V to 3.465 V |
| requirement 3: max_inductor_current (L1) | unverified | L1 is not the inductor of a step-down stage Kairo recognises |
| requirement 4: min_capacitance (VIN_C) | unverified | the effective capacitance between VIN_C and GND is not established |
| requirement 5: min_capacitance (+5V) | pass | C11, C12 give 20 uF nominal (marked values, as asked) between +5V and GND; 10 uF is required |
| requirement 6: min_capacitance (+3V3) | pass | C7, C2, C3, C4, C5 give 20.3 uF nominal (marked values, as asked) between +3V3 and GND; 1 uF is required |
| requirement 7: max_junction_temp (U4) | unverified | U4's junction is at least 51.4 degC; a lower bound cannot show the 125 degC limit is respected |
| requirement 8: max_junction_temp (U5) | unverified | U5 is not a linear regulator Kairo can work out the dissipation of |
| requirement 9: max_component_power (R5) | unverified | R5's dissipation is not established |
| requirement 10: max_component_power (R14) | pass | R14 dissipates 0.302 mW at nominal; the limit is 50 mW (50% of 100 mW) |
| regulator_load (U4) | unverified | U4 supplies at least 69.55 mA on +3V3; it is rated for 600 mA |
| regulator_stability (U4) | unverified | U4's stable region is not given, so its output capacitor cannot be checked against it |

## Tolerances in use

| Part | Parameter | Tolerance | Source |
|---|---|---|---|
| C8 | value | 10% | user-stated; LCSC's catalogue entry C22367839 for HHV1206R7475K101NSPJ (read 2026-10-11, not checked against the datasheet): tolerance, v_rating, dielectric |
| C9 | value | 10% | user-stated; LCSC's catalogue entry C20616238 for AM03B104K500NT (read 2026-10-11, not checked against the datasheet): tolerance, v_rating, dielectric |
| R13 | value | 1% | generic 0603 1% thick film; value from TI SNVSAC1 Figure 7 |
| U5 | Vout | 3% | TI LMR16006Y-Q1 datasheet SNVSAC1 (June 2015): p.4 VIN 4-40 V recommended (65 V absolute), operating junction 125 degC max, RthetaJA 102 degC/W (DDC); p.5 fixed 5 V output 4.85-5.15 V, switching frequency 1785-2415 kHz; p.1 and p.10 600 mA maximum output current |
| C10 | value | 10% | user-stated; LCSC's catalogue entry C20616238 for AM03B104K500NT (read 2026-10-11, not checked against the datasheet): tolerance, v_rating, dielectric |
| L1 | value | 20% | user-stated; LCSC's catalogue entry C6808095 for ANR4030T100M (read 2026-10-11, not checked against the datasheet): tolerance, i_sat, i_rating |
| C11 | value | 10% | user-stated; LCSC's catalogue entry C19103847 for HGC0805R5106K250NSLJ (read 2026-10-11, not checked against the datasheet): tolerance, v_rating, dielectric |
| C12 | value | 10% | user-stated; LCSC's catalogue entry C19103847 for HGC0805R5106K250NSLJ (read 2026-10-11, not checked against the datasheet): tolerance, v_rating, dielectric |
| R6 | value | 1% | generic 0603 1% thick film |
| R7 | value | 1% | generic 0603 1% thick film |
| U4 | Vout | 1.5% | Diodes AP2112 datasheet DS39724 Rev. 2-2 (June 2017): p.1 output accuracy 1.5% and stable with 1.0 uF ceramic; p.8 (3.3 V) maximum output current 600 mA min, dropout 400 mV max at 600 mA, quiescent current 80 uA max, thetaJA 96 degC/W (SOT25) |
| C6 | value | 10% | user-stated; LCSC's catalogue entry C19103847 for HGC0805R5106K250NSLJ (read 2026-10-11, not checked against the datasheet): tolerance, v_rating, dielectric |
| C7 | value | 10% | user-stated; LCSC's catalogue entry C19103847 for HGC0805R5106K250NSLJ (read 2026-10-11, not checked against the datasheet): tolerance, v_rating, dielectric |
| R14 | value | 1% | generic 0603 1% thick film |
| D7 | Vf | 5.46% | user-stated; LCSC's catalogue entry C7496818 for F.0603.00025/P2-0603G1TS2-06T-002 (read 2026-10-11, not checked against the datasheet): v_f, v_f_tolerance |
| R1 | value | 1% | generic 0603 1% thick film; ESP32-C3-WROOM-02 datasheet v1.7 p.34: EN RC delay 10 k / 1 uF |
| C1 | value | 10% | user-stated; LCSC's catalogue entry C33546005 for HGC0603R7105K250NTHJ (read 2026-10-11, not checked against the datasheet): tolerance, v_rating, dielectric |
| R2 | value | 1% | generic 0603 1% thick film |
| R3 | value | 1% | generic 0603 1% thick film; GPIO8 must be 1 for download boot (ESP32-C3-WROOM-02 datasheet v1.7 Table 4-3 p.13) |
| R4 | value | 1% | generic 0603 1% thick film; GPIO2 pull-up recommended (ESP32-C3-WROOM-02 datasheet v1.7 Table 4-3 note 2 p.13) |
| C2 | value | 10% | user-stated; LCSC's catalogue entry C19103847 for HGC0805R5106K250NSLJ (read 2026-10-11, not checked against the datasheet): tolerance, v_rating, dielectric |
| C3 | value | 10% | user-stated; LCSC's catalogue entry C20616238 for AM03B104K500NT (read 2026-10-11, not checked against the datasheet): tolerance, v_rating, dielectric |
| R5 | value | 1% | generic 0603 1% thick film |
| D6 | Vf | 5.46% | user-stated; LCSC's catalogue entry C7496818 for F.0603.00025/P2-0603G1TS2-06T-002 (read 2026-10-11, not checked against the datasheet): v_f, v_f_tolerance |
| C4 | value | 10% | user-stated; LCSC's catalogue entry C20616238 for AM03B104K500NT (read 2026-10-11, not checked against the datasheet): tolerance, v_rating, dielectric |
| R9 | value | 1% | generic 1206 1% thick film |
| C5 | value | 10% | user-stated; LCSC's catalogue entry C20616238 for AM03B104K500NT (read 2026-10-11, not checked against the datasheet): tolerance, v_rating, dielectric |
| R10 | value | 1% | generic 0603 1% thick film; MODE high = I2C/SSI (MT6701 v1.9 p.3) |
| R11 | value | 1% | generic 0603 1% thick film |
| R12 | value | 1% | generic 0603 1% thick film |

## Project policies in use

- Margins: only tolerances the design states were used; Kairo's default margins are turned off.
- Thermal: a junction temperature is reckoned with the datasheet's theta_JA, a typical value on the JEDEC test board, plus 20%, a margin this project chose. A pass under it is not a pass against a datasheet limit.
