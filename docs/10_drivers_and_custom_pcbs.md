# 10: Choosing Motor Drivers & What Circuits to Design Yourself

## Summary

- **Picking a driver is not hard.** Once you know which motor you're using, 6 questions settle it (below). For
  this arm the answer is **MKS SERVO42D / 57D (CAN)**, which you could choose in 30 minutes.
- **Designing your own *driver* is hard** and slow: high-current switching, current sensing,
  thermal design, FOC firmware, and usually 2-3 board revisions. It **doesn't save money** at quantities of 1-6
  (a JLC-assembled board costs about the same as an MKS driver once shipping is included). What you get from it is learning.
- **Recommended split for an April deadline:** *buy* the joint drivers (everything else waits on them) and *design* the
  support boards around them (power/E-stop, base controller, tool boards, encoder boards).
  Do a custom FOC driver as a controls side project that nothing else waits on. It can replace MKS drivers later if it works.

---

## 1. How to choose a driver: 6 questions

| # | Question | For this arm |
|---|---|---|
| 1 | **Motor type?** Stepper (2-phase) or BLDC (3-phase) | Stepper |
| 2 | **Current:** the driver's *continuous RMS* rating must be ≥ the motor's rated phase current, with margin | NEMA17 ≈ 1.5-2.1 A, NEMA23 ≈ 2.8 A (SERVO42D: up to 3.0 A, SERVO57D: up to 5.2 A) |
| 3 | **Voltage:** the driver must handle the supply (+ back-EMF spikes). Higher voltage = more torque at speed: above its corner speed a stepper's power scales with the supply ([03](03_actuators_gearboxes.md) §1b) | 24 V system, which is also the MKS drivers' maximum (12-24 V) |
| 4 | **Feedback:** open loop (can miss steps silently) or closed loop (encoder, stall detection) | Closed loop |
| 5 | **Interface:** STEP/DIR (you generate real-time pulses) vs UART/SPI (configuration) vs **CAN** (send targets, the driver runs the loop) | CAN, one 4-wire bus |
| 6 | **Control modes you need:** position only? velocity? **current/torque**? | Position + velocity now. Torque later (controls track) |

Then check the tie-breakers: documentation quality, community, open firmware, AliExpress availability, and size (does it mount on the back of the motor?).

## 2. Options compared

| Driver | Motor | Current | Interface | Closed loop | Torque mode | ~Price | Notes |
|---|---|---|---|---|---|---|---|
| **TMC2209** module | Stepper | ~1.4 A RMS practical (2 A max) | STEP/DIR + UART | No | No | US$3 | Silent and cheap, but open loop. Needs an MCU generating pulses |
| **TMC5160** module | Stepper | ~3 A RMS+ (external MOSFETs) | SPI (built-in motion controller) or STEP/DIR | No (stallGuard only) | No | US$10-15 | For NEMA23 on a DIY board |
| **MKS SERVO42D** | NEMA17 | ~3 A peak | CAN / RS485 / STEP-DIR | **Yes** (on-motor magnetic encoder) | Position/speed modes. Check the manual for current limit settings | US$20-25 | **Recommended.** Mounts on the back of the motor |
| **MKS SERVO57D** | NEMA23 | higher | CAN / RS485 / STEP-DIR | **Yes** | same | US$28-35 | For J2 |
| **SimpleFOC Mini** + MCU | Gimbal BLDC (or stepper via SimpleFOC) | < ~2.5 A | Your MCU decides | Yes (add encoder) | **Yes** | €7-10 | A good **FOC learning board**. Too weak for arm joints |
| **Spectral Micro** (Source Robotics / PCrnjak) | BLDC | small-medium | CAN 1 Mbit | Yes (14-bit encoder) | **Yes**: position, velocity, torque, impedance | Tindie | Used in their SSG-48 gripper. Has a Python lib and a ROS 2 package |
| **moteus-c1** (mjbots) | BLDC | 20 A peak / 5-14 A cont. | CAN-FD | Yes | **Yes** | US$69 | Open HW (KiCad) + FW. Well documented |
| **ODrive S1** | BLDC | 40 A cont. | CAN / USB / UART | Yes | **Yes** | US$149 | Overkill for a desk arm |
| **GIM6010-8** (Steadywin) | Integrated BLDC + 8:1 planetary + driver | 5 N·m rated / 11 N·m peak, 388 g | CAN | Yes | **Yes** | ~US$72 | A complete QDD actuator. 15 arcmin (0.25°) backlash ≈ 2 mm at 500 mm reach on J1, so it's better suited to a controls testbed or a v2 joint |

**Manual and protocol:** [makerbase-motor/MKS-SERVO42D-57D](https://github.com/makerbase-motor/MKS-SERVO42D-57D)
(user manual with the CAN command set). Community implementations to cross-check against are listed in
[02](02_prior_art_research.md) §5b-2. Buy the **CAN** variant: the RS485 one is a different board.

**What MKS doesn't give you:** a clean *torque command*. That's fine for position-controlled
pick-and-place, but compliant/impedance control and computed-torque control need current-controlled
FOC drivers. That's the reason for the controls track below.

## 3. What to design yourself (ranked by value vs risk)

| Board | Difficulty | Risk to April | What you learn | When |
|---|---|---|---|---|
| **A. Power & E-stop board**: 24 V in, fuse per branch, E-stop relay/contactor drive, TVS/reverse-polarity protection, 5 V buck, voltage/current monitor (INA226) | ★★☆ | Low (hand-wired fallback exists) | Power design, protection, safety interlocks | Rev A in Oct |
| **B. Base controller**: ESP32-S3 (USB + built-in CAN/TWAI) + transceiver + 120 Ω jumper, E-stop sense input, status LEDs, hall-sensor inputs. Acts as the **USB-CAN bridge** and safety watchdog. Firmware: [`firmware/base_controller`](../firmware/base_controller/README.md) | ★★☆ | Low (dev board + modules as fallback) | MCU design, CAN, USB, firmware | Rev A in Oct |
| **C. Joint encoder boards**: tiny MT6701 board at the back of J2-J4 (through-rod or through-tube, wrist later), hall sensors on J1 and J7. Which joint gets what, where the magnet goes on a hollow joint, and how the reading reaches the host (board C as a CAN node): [16](16_packaging_and_routing.md) §3 | ★☆☆ | Very low | Sensor integration, dual-loop control | Nov |
| **D. Tool-changer board pair**: pogo-pin carrier (arm side) + tool MCU board (ESP32-C3 + CAN + small motor driver + TOOL_DETECT) | ★★☆ | Low | Connector design, a tool protocol | Dec |
| **E. Custom FOC joint driver** (stepper or BLDC): STM32G4 + gate driver/MOSFETs or DRV8316, inline current sense, MT6701, CAN-FD | ★★★★ | **High**, so nothing else should wait on it | FOC, current loops: what motor control is built on | Controls track, prototype Dec-Feb, v2 in summer |

**Reference designs to learn from (don't start from scratch):**
- **moteus-c1**: open hardware in KiCad, production-quality BLDC FOC driver.
- **SimpleFOC** boards and library: the easiest way to get FOC running, and it supports steppers too.
- **PCrnjak's S-Drive** (L6234 BLDC driver firmware, GPLv3) and **Spectral Micro** docs.
- **Dummy (Peng Zhihui)**: a custom closed-loop stepper driver on CAN with STM32. This is exactly design E for steppers.
- **MKS SERVO42B/C**: older MKS closed-loop stepper boards with open firmware.

## 4. PCB workflow
- **Status (15 Oct 2026):** boards A and B (14 Oct) and C (15 Oct) are drawn (schematics only, ERC clean) in
  [hardware/](../hardware/README.md), with the checks, what is unverified, the open choices and the parts list.
- Tools: **KiCad** (free) + JLCPCB/PCBWay. Use JLC's parts library so they can assemble the boards (PCBA). Hand-solder only
  connectors.
- **Batch designs into one order** because shipping to Canada is the main fixed cost. Plan **2 orders total** before
  **mid-January**, since Chinese New Year (6 Feb 2027) shuts down fabs and AliExpress sellers for ~2-3 weeks.
- Rev A = breakout-style with test points, 0 Ω jumpers, and spare GPIO. Only compact it in Rev B.
- Makerspace: check for a hot-air station, a microscope, an **oscilloscope** (needed for debugging CAN and PWM), and a bench PSU
  with a current limit. **Always power new boards from a current-limited supply first.**
- Budget ~US$25-35 per order including shipping (a few small 2-layer designs + simple PCBA). Canadian duties/GST apply.
