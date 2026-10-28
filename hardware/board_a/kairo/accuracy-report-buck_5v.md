# Accuracy report: deskarm_board_a_buck

Status: **unverified**. 4 pass, 0 fail, 4 unverified.

A result is `pass` only when its worst case over every input's tolerance is proven. `unverified` means an input was missing; it is never a pass.

## To resolve

- Simulate +5V's ripple, or use one output capacitor (resolves 1 result: requirement 3: max_ripple (+5V))
- Add the manufacturer's DC-bias curve for HHV1206R7475K101NSPJ to the parts folder (resolves 1 result: requirement 4: min_capacitance (24V_LOGIC))
- Add the manufacturer's DC-bias curve for AM03B104K500NT to the parts folder (resolves 1 result: requirement 4: min_capacitance (24V_LOGIC))
- Give the tolerance of V(+5V) for U1, with where it comes from (resolves 1 result: requirement 5: max_component_power (R15))
- Measure U1's loop gain with its maker's model (simulate with loop), or follow the compensation its datasheet gives (resolves 1 result: regulator_stability (U1))

## Results

| Check | Status | Found |
|---|---|---|
| requirement 1: rail_voltage (U1) | pass | +5V is 4.871 V to 5.14 V from U1's reference and its divider (R13, R14), with the feedback pin's own current taken as nothing, as the datasheet's formula takes it; allowed is 4.75 V to 5.25 V |
| requirement 2: max_inductor_current (L1) | pass | L1 carries at most 2.83 A at its peak (the load's most plus half of 1.654 A of ripple); it saturates at 7 A |
| requirement 3: max_ripple (+5V) | unverified | +5V's ripple is worked out only for one output capacitor; it has 3 |
| requirement 4: min_capacitance (24V_LOGIC) | unverified | the effective capacitance between 24V_LOGIC and GND is not established |
| requirement 5: max_component_power (R15) | unverified | R15 dissipates 5.06 mW at nominal; the limit is 50 mW (50% of 100 mW) |
| regulator_load (U1) | pass | U1 supplies 2002 mA on +5V; it is rated for 5000 mA |
| switch_current_limit (U1) | pass | U1's switch carries at most 2.83 A at its peak; its current limit is at least 6.3 A |
| regulator_stability (U1) | unverified | U1 is a switching regulator, whose stability is a matter of its control loop and not of a capacitor's ESR range |

## Tolerances in use

| Part | Parameter | Tolerance | Source |
|---|---|---|---|
| C4 | value | 10% | user-stated; LCSC's catalogue entry C22367839 for HHV1206R7475K101NSPJ (read 2026-10-11, not checked against the datasheet): tolerance, v_rating, dielectric |
| C5 | value | 10% | user-stated; LCSC's catalogue entry C22367839 for HHV1206R7475K101NSPJ (read 2026-10-11, not checked against the datasheet): tolerance, v_rating, dielectric |
| C6 | value | 10% | user-stated; LCSC's catalogue entry C20616238 for AM03B104K500NT (read 2026-10-11, not checked against the datasheet): tolerance, v_rating, dielectric |
| R9 | value | 1% | generic 0603 1% thick film; value from TI SLVSF00 Figure 32 |
| R10 | value | 1% | generic 0603 1% thick film; value from TI SLVSF00 Figure 32 |
| R11 | value | 1% | user-stated; LCSC's catalogue entry C43720265 for ABD03K2003FT (read 2026-10-11, not checked against the datasheet): tolerance, p_rating, tempco |
| C7 | value | 10% | user-stated; LCSC's catalogue entry C20616238 for AM03B104K500NT (read 2026-10-11, not checked against the datasheet): tolerance, v_rating, dielectric |
| L1 | value | 20% | user-stated; LCSC's catalogue entry C41415571 for ZEMS0730-6R8M (read 2026-10-11, not checked against the datasheet): tolerance, i_sat, i_rating |
| R12 | value | 1% | generic 0603 1% thick film; value from TI SLVSF00 Figure 32 |
| C8 | value | 10% | user-stated; LCSC's catalogue entry C7503499 for CGA0603X7R472K500JT (read 2026-10-11, not checked against the datasheet): tolerance, v_rating, dielectric |
| C9 | value | 5% | user-stated; LCSC's catalogue entry C18220914 for CSA0603C0G470J500JT (read 2026-10-11, not checked against the datasheet): tolerance, v_rating, dielectric |
| R13 | value | 1% | generic 0603 1% thick film; value from TI SLVSF00 Figure 32 |
| R14 | value | 1% | generic 0603 1% thick film; value from TI SLVSF00 Figure 32 |
| C10 | value | 20% | user-stated; LCSC's catalogue entry C7432791 for HGC1210R5476M250NSVK (read 2026-10-11, not checked against the datasheet): tolerance, v_rating, dielectric |
| C11 | value | 20% | user-stated; LCSC's catalogue entry C7432791 for HGC1210R5476M250NSVK (read 2026-10-11, not checked against the datasheet): tolerance, v_rating, dielectric |
| C12 | value | 20% | user-stated; LCSC's catalogue entry C7432791 for HGC1210R5476M250NSVK (read 2026-10-11, not checked against the datasheet): tolerance, v_rating, dielectric |
| R15 | value | 1% | generic 0603 1% thick film |
| D8 | Vf | 5.46% | user-stated; LCSC's catalogue entry C7496818 for F.0603.00025/P2-0603G1TS2-06T-002 (read 2026-10-11, not checked against the datasheet): v_f, v_f_tolerance |
| U1 | V(+5V) | none given | missing |

## Project policies in use

- Margins: only tolerances the design states were used; Kairo's default margins are turned off.
