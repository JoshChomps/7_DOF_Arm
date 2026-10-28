# Custom PCBs: boards A and B (order 1) and C (order 1 or 2), rev A drafts

Schematics only, A and B drawn on 14 Oct 2026 and C on 15 Oct with **Kairo** (Josh's KiCad schematic MCP, `<path-to>\Kairo`, M1.9).
**Nothing here is laid out, ordered, built or powered.** Layout and routing are Josh's, in KiCad, with board outlines
from the Fusion base-board layout. The plan behind the boards is [docs/10](../docs/10_drivers_and_custom_pcbs.md)
§3-4 and [docs/04](../docs/04_electronics_control.md).

| Folder | Board | Open in KiCad 10 |
|---|---|---|
| [board_a/](board_a/) | A: 24 V power, protection, E-stop contactor drive, current monitor, 5 V buck | `board_a/deskarm_board_a.kicad_pro` (two sub-sheets) |
| [board_b/](board_b/) | B: base controller, ESP32-S3 USB↔CAN bridge (`firmware/base_controller`) | `board_b/deskarm_board_b.kicad_pro` (one A0 sheet) |
| [board_c/](board_c/) | C: joint encoder CAN node, ESP32-C3 + MT6701 over SSI (D33, [docs/16](../docs/16_packaging_and_routing.md) §3.7-3.8) | `board_c/deskarm_board_c.kicad_pro` (one A0 sheet) |
| [bom_order1.csv](bom_order1.csv) | Parts list for all three boards, per board (LCSC numbers where Kairo's catalogue had the part) | |
| `*/kairo/` | The Kairo design files each sheet was built from, and Kairo's accuracy reports | |
| [kairo_tools/](kairo_tools/) | The three small scripts used to split board A and join its sheets (see "How board A was built") | |

Kairo's working copies stay in `D:\KiCad\projects\deskarm\` (its only allowed root). **These repo copies are the
master from now on**: edit here in KiCad. Kairo can still read and ERC-check them if they are copied back under
`D:\KiCad`, or if `<path-to>\7_DOF_Arm\hardware` is added to Kairo's `--allowed-root`.

## Board A: power and E-stop

```
PSU 24 V ─XT30 J1─► Q1‖Q2 (P-FET reverse-polarity, AOD4185 ×2) ─► +24V ─┬─ TVS D2, 100 nF
                                                                         ├─ F1 2 A ─► 24V_LOGIC ─► TPS54560B buck ─► +5V (to board B, 5 V aux)
                                                                         ├─ F2 1 A ─► J2 E-stop NC loop ─► COIL_FEED ─► J3 contactor coil ─► Q3 / R5 0 Ω ─► GND
                                                                         └─ J4 pin 1 ─► external contactor ─► J4 pin 2 ─► 2 mΩ shunt (INA226) ─► MOTOR_RAIL
MOTOR_RAIL ─┬─ TVS D5, 1000 µF, red LED
            └─ F3..F6 blade fuses ─► J5 (J1+J2), J6 (J3+J4), J7 (J5-J7), J8 (aux/tool), XT30 female
```

- **E-stop (D32, option A chosen 14 Oct):** the mushroom button's NC main contact sits in series with a 24 V
  contactor coil, so pressing it (or cutting the wire) drops the contactor and the motor rail. The motor current never
  passes a PCB relay. The button's **auxiliary** NC contact goes to board B's J4 so the firmware reports it (as now).
- **Software motor enable:** Q3 (DMN6075S) is in series with the coil, driven by `MOTOR_EN` from board B (GPIO14),
  100 kΩ pull-down. **R5 (0 Ω) bypasses it and is fitted**, so rev A works with today's firmware, which does not drive
  GPIO14. Remove R5 only after the firmware drives the pin.
- **5 V:** boards only (board B, the INA226 rail via board B, the 5 V aux header), sized for 2 A; the Raspberry Pi
  keeps its own USB-C supply (chosen 14 Oct by Josh). The buck is TI's own 5 V design example
  (SLVSF00 Figure 32: 16.9 k / 4.7 nF / 47 pF compensation, 53.6 k / 10.2 k divider, 442 k / 90.9 k UVLO, B560C),
  except **RT = 200 kΩ** (≈500 kHz, the only frequency TI gives min/max for: 450-550 kHz) and a 6.8 µH, 7 A inductor.
- **INA226** (address 0x40, A0 = A1 = GND) measures the motor rail's current and voltage *after* the contactor, so
  software sees the cut. It is powered from board B's 3.3 V through J9, and board B carries the I²C pull-ups.

Board A ↔ B cable (J9 on A, J8 on B, JST-XH 8, pin for pin): 1 +5V (A→B), 2 GND, 3 +3V3 (B→A), 4 SDA, 5 SCL,
6 INA_ALERT, 7 MOTOR_EN, 8 GND.

## Board B: base controller

ESP32-S3-WROOM-1 (N8, no PSRAM pins used), native USB-C with USBLC6 ESD and 5.1 k CC resistors, SN65HVD230 CAN
transceiver with a 120 Ω terminator on jumper JP1 (fit it: the bridge is a bus end), two CAN connectors, RESET and
BOOT buttons, AP2112K-3.3 LDO fed from USB or board A's 5 V through two SS14 diodes (`+5VD`).

| GPIO | Use | Matches |
|---|---|---|
| IO4 / IO5 | CAN TX → transceiver D, CAN RX ← R | `PIN_CAN_TX 4`, `PIN_CAN_RX 5` |
| IO6 | E-stop sense: J4 aux NC contact to GND, 10 k pull-up + 1 k / 100 nF filter (pressed or broken = HIGH) | `PIN_ESTOP 6` |
| IO48 | WS2812B-2020 status LED (VDD from `+5VD`: it needs 3.7-5.3 V; its DIN accepts ≥ 2.7 V, so 3.3 V logic drives it) | `PIN_RGB_LED 48` |
| IO10 / IO12 / IO13 | Bench MT6701 SSI: CSN / CLK / DO on J7 (10 k pull-ups on CSN and DO) | `PIN_ENC_CSN 10`, `_CLK 12`, `_DO 13` |
| IO8 / IO9 | I²C to board A's INA226 (4.7 k pull-ups) | new, firmware does not use it yet |
| IO7 | INA226 ALERT (10 k pull-up) | new |
| IO14 | MOTOR_EN to board A | new (R5 on board A bypasses it) |
| IO1 / IO2 | Hall inputs J5 / J6 (5 V supply for A3144-type, open-collector output pulled up to 3.3 V) | new: the halls of D33 go to the MKS drivers' IN_1; these are spares |
| TXD0 / RXD0 | UART0 header J9 | |
| IO11, 15-18, 21, 38-42, 47 | Spare header J10 (with 3V3 and GND) | |
| IO3, IO45, IO46 | Strapping pins, left open | |
| IO35-37 | Left open (octal PSRAM on R8 modules) | |

## What was checked, and what was not

| Check | Result |
|---|---|
| Every pin name and number | From Kairo's `get_symbol_pins` (KiCad 10 libraries); nets read back with `trace_net` |
| Board A, joined project, KiCad ERC (`run_erc`, KiCad 10.0.6) | **Clean**, 0 errors, 0 warnings |
| Board B, KiCad ERC | **Clean**, 0 errors, 0 warnings |
| Kairo verifier on each Kairo-built sheet (A power, A buck, B): pins, structure, drawing, KiCad netlist, ERC | Pass (one warning each: a hidden power-symbol value on the A↔B connector) |
| Firmware pins on board B | `trace_net`: IO4→U3 D, IO5←U3 R, IO6 E-stop, IO10/12/13 → J7, IO48 → D1 DIN |
| Cross-sheet nets on board A | `trace_net`: 24V_LOGIC joins F1/TP7 to U1/C4/C5; +5V joins J9/TP4 to L1/C10-C12 |
| 5 V rail | **Pass**: 4.871-5.14 V (TPS54560B 0.792-0.808 V reference, 1 % divider) against 4.75-5.25 V |
| Buck inductor and switch | **Pass**: peak 2.83 A at 2 A load (1.654 A ripple) against L1's 7 A saturation and U1's ≥ 6.3 A limit |
| Buck load | **Pass**: 2 A against 5 A |
| 3.3 V rail | **Pass**: 3.25-3.349 V (AP2112 ±1.5 %) against ±5 % |
| 3.3 V decoupling, LED resistor | **Pass** (20.2 µF nominal; R16 0.3 mW) |

Kairo results that are **unverified** (its `to_resolve` lines, unchanged):

- 5 V ripple: "Simulate +5V's ripple, or use one output capacitor".
- Buck input capacitance (TI asks for ≥ 3 µF effective): "Add the manufacturer's DC-bias curve for HHV1206R7475K101NSPJ
  to the parts folder"; "Add the manufacturer's DC-bias curve for AM03B104K500NT to the parts folder".
- Buck loop stability: "Measure U1's loop gain with its maker's model (simulate with loop), or follow the compensation
  its datasheet gives". The compensation *is* TI's, but RT was moved from 243 k to 200 k, so TI's design is followed
  at a different switching frequency; the compensation was not re-derived.
- Board A R15 power (5.06 mW nominal of a 50 mW limit): "Give the tolerance of V(+5V) for U1, with where it comes from".
- AP2112 junction temperature (at least 62.1 °C; limit 125 °C) and its total load (at least 135.4 mA of 600 mA): the
  pull-up currents ("Run simulate with an op analysis for the current through R1" … R15), "Give the tolerance of
  Iout", and "If U1's i_supply is its datasheet maximum, list it under limits as max; if not, give the maximum"
  (Espressif publishes only a typical 107.9 mA). The θJA margin (20 %) is a choice made here, not a datasheet figure.
- AP2112 output-capacitor stability: "Give U4's least output capacitance (c_out_min) and the lowest and highest ESR it
  is stable with (c_out_esr_min, c_out_esr_max) from its datasheet". The datasheet says only "stable with 1.0 µF
  flexible cap: ceramic, tantalum and aluminium electrolytic".

Worked out with Kairo's `calc`, outside a requirement:

- Reverse-polarity FETs: at the PSU's 14.6 A ceiling each AOD4185 (23 mΩ max at 125 °C) dissipates **1.23 W** if the two
  share equally (one alone would be 4.9 W). AOS gives 50 °C/W max steady state (on its footnoted test board), so this needs copper;
  Kairo has no generic junction-temperature calculation, so Tj is not computed.
- Gate divider: at the PSU's 33.6 V over-voltage ceiling the gate sees 16.8 V (AOD4185: ±20 V) and each 10 k
  dissipates 28 mW. The 15 V zener D1 clamps faster edges.
- Shunt: 0.43 W at 14.6 A in 2 mΩ (2 W part). The INA226's shunt input range is ±81.92 mV (SBOS547 p.5).
- Trace width for 14.6 A at 20 °C rise (IPC-2221, outer layer): **7.96 mm on 1 oz, 3.98 mm on 2 oz**. Use pours.

Not checked at all: the layout (none exists), footprints against the real parts' drawings, the pad-count match of the
XT30, blade-holder, USB-C and button footprints, the contactor and E-stop parts themselves, thermal behaviour, EMC,
anything on hardware.

## Open choices for Josh

1. **Contactor or relay part**: a 24 V-coil contactor (DIN rail) rated for DC at ≥ 15 A, and its coil current (F2 is
   1 A; Q3 is 2.5 A, 60 V). DC breaking matters: AC-rated contacts are often derated heavily at 24 V DC.
2. **Fuse ratings**: F1 (logic, 2 A placeholder), F2 (coil, 1 A placeholder), F3-F6 blade fuses for the J1+J2,
   J3+J4, J5-J7 and aux branches (no values chosen; they wait for the MKS drivers' measured supply current and the PSU's
   14.6 A).
3. **Connectors**: XT30 for 24 V in and the four motor branches (check the XT30's continuous rating against 14.6 A); screw terminals
   for E-stop, coil and contactor main (MKDS-3, check its current rating); JST-XH for the A↔B cable, CAN, halls, E-stop
   sense and MT6701. What the MKS SERVO42D/57D actually take for power and CAN was **not checked**; match J2/J3 on
   board B and J5-J8 on board A to them.
4. **Reverse-polarity FETs**: two AOD4185 in parallel, or one FET with lower RDS(on), or rely on the keyed XT30 alone.
5. **TVS**: SMCJ26CA (26 V stand-off). It conducts if the PSU's trim pot is turned up past ~26 V (the LRS-350-24
   adjusts 21.6-28.8 V): keep the PSU at 24 V, or choose SMCJ30CA and accept a higher clamp at the drivers.
6. **Bulk capacitor** C3 (1000 µF 35 V, through-hole, hand-fit) on the motor rail: fit, and how big (docs/04 says "a few
   thousand µF" if the PSU trips during fast stops).
7. **Board B's CAN slope** (R5 0 Ω = fastest edges; 10-100 k slows them) and whether both CAN connectors are needed.
8. **JLC basic parts**: Kairo's catalogue mostly returned extended parts; swap for basic equivalents when placing the order.

## Board C: joint encoder node (drawn 15 Oct)

One per encoded joint: J2, J3 and J4 at first (D33), the wrist later. It sits on the actuator's stationary housing
with the MT6701 on the joint axis, under the Ø8 × 2.5 mm magnet on the through-rod (J2/J4) or the pickup stub (J3),
~1 mm away. It reads the chip over SSI and puts the joint-angle frame of D34 on the CAN bus (ID `0x10` + joint, the
frame the base controller already sends for the bench MT6701; [firmware README](../firmware/base_controller/README.md)).
The host does the coarse + fine sum; board C only reports the raw angle.

```
24 V tap ─J1─► D1 SS16 (reverse polarity) ─► VIN_C ─┬─ D2 SMAJ36A, C8 4.7 µF 100 V, C9 100 nF
                                                    └─ U5 LMR16006YQ5 (fixed 5 V, 2.1 MHz) ─ L1 10 µH ─► +5V ─ D5 SS14 ─┐
USB-C J4 (flashing) ─ U2 USBLC6 ─ VBUS ─ D4 SS14 ────────────────────────────────────────────────────────────────────┴─► +5VD ─► U4 AP2112K-3.3 ─► +3V3
+3V3 ─► U1 ESP32-C3-WROOM-02, U6 MT6701CT (SOIC-8, on board), U3 SN65HVD230 (120 Ω on JP1, left open: mid-bus)
CAN IN J2 ═ CAN OUT J3 (H, L, GND; daisy-chained through the board, no stub)
```

| GPIO | Use | Note |
|---|---|---|
| IO4 / IO5 | CAN TX → U3 D, CAN RX ← U3 R | same numbers as board B |
| IO10 / IO6 / IO7 | MT6701 CSN (pin Z) / CLK (B) / DO (A); 10 k pull-ups on CSN and DO | board B uses IO10/12/13; the C3 has no IO12/13. Change `PIN_ENC_*` in board C's `config.h` |
| - | MT6701 MODE tied high through 10 k (I²C/SSI mode; it also has a 200 k internal pull-up, datasheet v1.9 p.3); OUT and PUSH open | |
| IO18 / IO19 | USB D− / D+ (the C3's USB Serial/JTAG: flashing and serial with no USB-UART chip) | |
| IO9 | BOOT button, 10 k pull-up | strapping pin: low at reset = download |
| IO8, IO2 | 10 k pull-ups | strapping: IO8 must be high for download boot, IO2 pull-up recommended (WROOM-02 datasheet v1.7 Table 4-3) |
| EN | 10 k / 1 µF delay + RESET button | Espressif's values (datasheet p.34) |
| IO3 | Green status LED, 1 k | |
| IO0, IO1, TXD0, RXD0 | J5 AUX header (3V3, IO0, IO1, TXD, RXD, GND), fit only if needed | IO0/IO1 could be joint-ID straps or a hall input |

Choices made in the draft (all open to Josh; see the list below):

- **24 V → 5 V → 3.3 V**, not straight to 3.3 V, so the board can be flashed and its sensor read from USB alone on the
  desk: the same diode-OR and AP2112 as board B. The LDO's extra loss is small at the ~70 mA the 3.3 V rail draws.
- **LMR16006YQ5** buck (TI SNVSAC1): 4-40 V operating, **65 V absolute** on VIN, so hot-plug ringing on the harness
  has room; fixed 5 V output (4.85-5.15 V, p.5), so no divider; SOT-23-6 (DDC). The circuit is TI's fixed-output one
  (Figure 14: 100 nF boot, Schottky catch diode, 100 k on SHDN) except **L1 = 10 µH**: Kairo's `calc buck_inductor`
  gives **9.645 µH minimum** at 28.8 V in (the PSU's top trim), 1785 kHz (the datasheet's lowest frequency) and
  0.24 A ripple (40 % of 0.6 A; TI says 30-40 %). TI's 6.8 µH example is for a 16 V maximum input.
- **SMAJ36A** after the diode: its 36 V stand-off is above the PSU's 28.8 V trim and 33.6 V over-voltage ceiling.
  Its clamping voltage was not read (Littelfuse refused the download), so whether it keeps a surge under U5's 65 V is
  unverified.
- **SSI timing:** DO changes on CLK's rising edge and is sampled after the falling edge (datasheet v1.9 p.26), which
  is what `main.cpp`'s bit-bang read already does.

### What was checked on board C, and what was not

| Check | Result |
|---|---|
| Every pin name and number | From `get_symbol_pins` (KiCad 10 libraries: `RF_Module:ESP32-C3-WROOM-02`, `Sensor_Magnetic:MT6701CT`, `Regulator_Switching:LMR16006YQ5`, …) |
| KiCad ERC (`run_erc`, KiCad 10.0.6) | **Clean**, 0 errors, 0 warnings |
| Kairo verifier (pins, structure, drawing, KiCad netlist against the design, ERC) | **Pass**, all five layers, no findings |
| Nets | `read_schematic`: 41 nets, 59 parts; pin counts match the design (`/ENC_CSN` 4, `/ENC_CLK` 3, `/ENC_DO` 4, `/CAN_TX` 3, `/CAN_RX` 3, `/USB_DP` and `/USB_DN` 5 each). `trace_net` was **not** run pin by pin: it locked for the session after the first calls used the names without KiCad's `/` prefix, so the per-pin check rests on the verifier's netlist layer |
| 3.3 V rail | **Pass**: 3.25-3.349 V (AP2112 ±1.5 %) against ±5 % |
| +5V output capacitance | **Pass** (nominal): 20 µF against TI's 10 µF |
| +3V3 capacitance | **Pass** (nominal): 20.3 µF against 1 µF |
| R14 (power LED) | **Pass**: 0.302 mW against 50 mW |
| Kairo review (`analyze_power_tree`) | 0 faults, 0 suggestions |

Worked out with Kairo's `calc`, outside a requirement (Kairo can't check this buck, see below):

- Inductor: minimum 9.645 µH (above). With 10 µH at −20 % (8 µH), 28.8 V in, 5.15 V out, 1785 kHz and ideal
  efficiency (duty 0.1788), the ripple is **0.2961 A** and the peak at the full 600 mA rating **0.748 A**. L1
  (ANR4030T100M) saturates at 2.4 A by LCSC's listing, above U5's 1.7 A maximum peak current limit (SNVSAC1 p.5, given
  at 12 V and 25 °C only), so a short should not saturate it. The board itself draws ~70 mA, far below the rating.

Kairo results that are **unverified** (its `to_resolve` lines, unchanged):

- +5V rail: "Say what sets +5V, with its output voltage and tolerance". Inductor current: "Connect L1 from the
  regulator's switch pin to its output rail, and set the regulator's regulator parameter to switching". U5 junction
  temperature: "Set U5's regulator parameter to linear, or state its dissipation". All three are one Kairo limit:
  KiCad's `LMR16006YQ5` symbol types its SW pin as `output`, and Kairo only recognises a buck whose switch pin is
  `power_out`. The circuit *is* wired SW → L1 → +5V, and `regulator` *is* `switching`. The rail's range is the
  chip's own 4.85-5.15 V (datasheet p.5), not checked by Kairo against the ±5 % requirement.
- Buck input capacitance (TI asks for 1-10 µF): "Add the manufacturer's DC-bias curve for HHV1206R7475K101NSPJ to the
  parts folder"; "Add the manufacturer's DC-bias curve for AM03B104K500NT to the parts folder". C8 is a 100 V part, so
  it loses less at 24 V than a 50 V one, but no curve was read.
- AP2112 junction temperature (at least 51.4 °C; limit 125 °C) and its total load (at least 69.55 mA of 600 mA):
  "Run simulate with an op analysis for the current through R1" (and R2, R3, R4, R10, R11, R12), "Give the tolerance
  of Iout, with where it comes from", "If U1's i_supply is its datasheet maximum, list it under limits as max; if not,
  give the maximum" (Espressif publishes only a typical 28 mA with Wi-Fi off).
- R5 (status LED) power: "Kairo works out a resistor's power when it runs from a rail to ground, alone or through an
  LED; state R5's dissipation otherwise". It is the same 1 k and LED as R14, driven from IO3 instead of +3V3.
- AP2112 output-capacitor stability: "Give U4's least output capacitance (c_out_min) and the lowest and highest ESR it
  is stable with (c_out_esr_min, c_out_esr_max) from its datasheet" (as on board B).

Not checked at all: the 2.1 MHz buck's minimum on-time at high input (80 ns typical, SNVSAC1 p.5; at 24-28.8 V the
on-time is near it, so the chip may skip pulses: harmless for a rail this lightly loaded, but untested), the TVS
clamp, CAN bus ESD beyond the SN65HVD230's own rating, the layout (none exists: the MT6701 must sit centred on the
joint axis, the module's antenna needs its keep-out even though Wi-Fi is unused), footprints against the real parts,
whether the MT6701 datasheet's magnet figures changed between Rev 0.1 and v1.9, anything on hardware.

**Found while reading the MT6701 datasheet:** the newest datasheet (v1.9, 2024.05) gives INL **±1.0° typical at
25 °C, ±1.5° max over −40…125 °C**. The repo had ±0.5° typical from the 2020 Rev 0.1. The coarse error budget in
[docs/16](../docs/16_packaging_and_routing.md) §3.3 is updated: J2 ~3.5° against ±3.75° (was ~3.1°).

### Open choices for Josh (board C)

1. **How many boards**: D33 says J2, J3, J4 now and two for the wrist later. JLC's minimum is 5, which covers both.
2. **Order 1 or 2**: it is drawn in time for order 1, but its connectors and size depend on the J2-J4 CAD (mounting,
   where the board sits behind the actuator). Ordering in January after the CAD is frozen is the safer choice.
3. **Connectors**: drawn as a separate 24 V tap (J1, JST-XH 2) and CAN in/out (J2/J3, JST-XH 3 H/L/GND, the same as
   board B), so the motor current never crosses board C. The alternative is XT30(2+2) in and out with 24 V passing
   through, which needs wide copper (~6 A on the J2→J3 segment, docs/16 §2). The MKS drivers' own connector types are
   still unchecked.
4. **Programming route**: a USB-C on every board (as drawn) or a smaller 4-pin USB header/pads (saves the connector
   and the USBLC6). Later, firmware updates could go over CAN.
5. **Module**: ESP32-C3-WROOM-02 (18 × 20 mm), the only C3 module in KiCad's library here. The ESP32-C3-MINI-1
   (13.2 × 16.6 mm) is smaller but needs Espressif's symbol and footprint added.
6. **Joint ID**: compiled in, stored in flash, or set by straps on IO0/IO1 (J5).
7. **TVS and reverse diode**: SMAJ36A + SS16 as drawn, or rely on the keyed connector and U5's 65 V rating.
8. **CAN slope** (R8 0 Ω = fastest edges, as on board B) and **JLC basic parts** (swap extended parts when ordering).

## How board A was built (and why it is two sheets)

Kairo writes one sheet per project and places blocks in a single row of A4-height columns, so board A's 64 parts
needed a 1219 mm-wide sheet, wider than A0. It was split along power nets only (Kairo draws power and ground nets as
global labels or power symbols and signal nets as local labels, so only power nets may cross): `split.py` cut the one
design into the `power_estop` and `buck_5v` designs; Kairo built and verified each in its own folder; `combine.py`
made the top sheet with two hierarchical sheets, rewrote each copy's instance paths for the joined project and
renumbered the power symbols; `drop_flags.py` removed the second PWR_FLAG that each sheet had put on the shared
24V_LOGIC and +5V rails (KiCad flagged them as two power outputs). The joined project then ran clean in KiCad ERC.
Board B (52 parts) fitted one A0 sheet and was built directly.
