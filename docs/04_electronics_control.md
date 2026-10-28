# 04: Electronics & Control Hardware

> How to pick drivers and which boards we design ourselves: see [10](10_drivers_and_custom_pcbs.md).

## Architecture

```
  Laptop / Raspberry Pi 4 B  (Python digital twin → later ROS 2 + MoveIt 2)
          │ USB
   Base controller: ESP32-S3 + CAN transceiver (our PCB B; acts as USB-CAN bridge + E-stop watchdog)          24 V PSU ──► E-STOP ──► motor power rail
          │                                                         │
          ▼   CAN bus @ 500 kbit-1 Mbit, 120 Ω at both ends         │
  ┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬───────────┐   │
  J1     J2     J3     J4     J5     J6     J7    Tool MCU (on │ ◄─┘
 drv    drv    drv    drv    drv    drv    drv    the gripper, via the slip ring + pogo pins)
 (each driver = closed-loop stepper board on the back of its motor; node IDs 1-7 = J1-J7)

  Through the arm: ONLY 4 wires → 24 V, GND, CAN-H, CAN-L (twisted pair)
```

Why a CAN daisy chain:
- Adding a joint or tool is just another node on the same 4 wires.
- A **hollow center in each gearbox** (≥ 10-12 mm) carries the harness straight through the joint.
- It's what Atlas, Dummy, and OpenArm do.

## Components

| Function | Recommended | Budget alternative | Advanced alternative |
|---|---|---|---|
| Joint drivers | **MKS SERVO42D (NEMA17) / SERVO57D (NEMA23)**, **CAN variants** (the RS485 version is a different board). Manual + CAN protocol: [makerbase-motor/MKS-SERVO42D-57D](https://github.com/makerbase-motor/MKS-SERVO42D-57D) | TMC2209 open-loop + separate encoders + limit switches, on an Arduino Mega/ESP32 CNC shield (early Arctos / CyBot style) | BLDC: **moteus-c1** ($69, open HW/FW), ODrive S1 ($149), SimpleFOC boards |
| Host | Your laptop (free) | - | The Raspberry Pi 4 B you already have, on the base board, for a standalone arm (D17) |
| CAN adapter | **ESP32-S3 base controller** (built-in TWAI CAN) as USB-CAN bridge, first on a dev board, then on our PCB B | **Also buy a CANable 2.0-class adapter (~US$12)** as a known-good reference: the bridge firmware has never run on real hardware, and you don't want to debug it and an unfamiliar driver at the same time | - |
| Real-time bridge (optional) | Not needed at first, since the drivers do their own control loops | ESP32-S3 (has a CAN/TWAI controller built in) | Teensy 4.1 for high-rate trajectory streaming |
| Power | **24 V, 10-15 A** (e.g. Mean Well LRS-350-24) | Used laptop brick (limited current) | - |
| E-stop | Latching mushroom button in series with the motor power rail (use a relay/contactor for high current) | - | Also connect it to a digital input so the software knows the E-stop was pressed |
| Homing | Hall-effect sensor + magnet per joint (the AR4 Mk5 approach) | Microswitches | **Output-side absolute encoder** per joint (AS5600 cheap, MT6701/AS5048A better), so no homing is needed |
| Tool MCU | ESP32-C3/S3 + SN65HVD230 CAN transceiver, or RP2040 + MCP2515 | - | STM32G4 (FD-CAN) |
| 5 V supply | 24 V → 5 V buck module per MCU / Pi until the custom PCBs carry their own regulator | - | - |
| Connectors | XT30 (power), JST-GH or Molex Micro-Fit (CAN), pogo pins at the tool flange | - | - |
| Wire | Silicone 18 AWG (power), twisted pair 24 AWG (CAN), high-flex cable at joints | - | - |

## Important details

- **Motor-side vs joint-side encoders:** the MKS drivers read the *motor* shaft. After a 48:1
  gearbox the joint angle is known only relative to where it started, and gearbox backlash
  isn't measured at all. So you need either **homing sensors** (cheap, required at minimum) or
  **output encoders** (better, and they let you measure backlash directly). The motor encoder is absolute within one
  motor turn, so after a power cut only the turn count is lost, and a coarse output reading (±3.75° on J2-J4) recovers
  it. Which joints get which sensor, and how a magnet fits a hollow joint: [16](16_packaging_and_routing.md) §3.
- **Holding current and heat:** steppers draw current even when standing still. Set the holding current around 50-70%.
  Check how hot the housings get, because PETG softens at ~70-80 °C. Put mounting standoffs or an air gap between the motor and the printed housing.
- **Driver facts from the MKS manual:** supply **12-24 V**; working current settable up to 3.0 A (SERVO42D) and 5.2 A
  (SERVO57D); CAN at 125 k / 250 k / 500 k / 1 M bit/s with standard 11-bit frames, default 500 k; default work mode is the
  closed-loop FOC mode. 24 V is therefore the ceiling for this arm.
- **Power sizing:** 7 steppers (6 × NEMA17 at 1.5-2.1 A per phase, 1 × NEMA23 at 2.8 A), plus the tool, give a peak of
  about 250-300 W: the drivers chop the supply, so supply current is well below the sum of the phase currents.
  A 350 W PSU leaves headroom. Decelerating joints push energy back into the 24 V rail; if the PSU trips on
  over-voltage during fast stops, add a bulk capacitor (a few thousand µF) on the motor rail.
- **Grounding and EMI:** use a single star ground at the PSU. Terminate the CAN bus correctly with 120 Ω resistors
  at the host end and the last node. Keep power wires and CAN wires in separate bundles where possible.
- **Wire twist at joints:** hollow shafts plus limited joint ranges (±170°) keep wires from twisting
  too far. J7 is continuous, so it gets the slip ring (D11). Which joints are hollow and where each motor sits:
  [03](03_actuators_gearboxes.md), "Hollow shafts and where the motors go".
- **E-stop cuts motor power, and the drivers forget where they are.** The MKS encoder is on the motor shaft and only
  knows one turn absolutely, so after any power cut the arm must re-home (hall sensors) or read its output encoders
  ([16](16_packaging_and_routing.md) §3.3: coarse output reading + motor encoder = exact angle).
  **Do not count on the arm holding its pose unpowered.** With the efficiencies in `arm.toml` (0.63-0.72) a simple
  friction model (reverse efficiency 2 − 1/η) says J2-J6 can backdrive at their worst poses with 1 kg: the load reflected
  to the motor (0.06-0.13 N·m) exceeds a stepper's detent torque (typically 0.02-0.07 N·m), the wrist first. The
  counterbalance spring brings J2 to about the detent level. Expect sag or a slow drop on E-stop and on a driver stall
  (the MKS driver releases the shaft); `tests/actuator_bench.md` §3 measures the real figure. The physics sim's default
  (`backdrivable=False`) is the optimistic case.
- **CAN bus load (estimate, check frame sizes against the MKS manual):** per joint and control cycle, one position command
  (8 data bytes), its acknowledge (3), one encoder read request (2) and its reply (8) come to ~430 bits with worst-case
  bit stuffing. Seven joints: ~3,000 bits per cycle. At **50 Hz** that is 30% of a 500 kbit/s bus; at **100 Hz** it is 60%,
  which is too full, so run **1 Mbit/s** for 100 Hz streaming (30%), or drop the per-cycle acknowledges.
- **The ESP32-S3's USB port can reset the chip when a PC opens it.** Its built-in USB-serial interface treats the
  RTS/DTR modem lines as reset signals, and serial libraries toggle them on open (and on close). python-can never checks
  the bridge's replies, so a bridge that was still rebooting would look like a dead bus. `deskarm.bridge.open_bus`
  therefore waits until the firmware answers a version request and then re-sends "set bitrate" and "open".
- **Streaming setpoints is the first thing to test on the bench.** The MKS drivers plan their own motion (speed +
  acceleration per position command). The twin sends a new setpoint every cycle, so check how the driver behaves when a
  new target arrives mid-move (does it blend or stop and restart?). If it stutters, the fallback is speed-mode commands
  with the position loop closed in Python.

## Bring-up order

1. One motor + driver + USB-CAN on the bench → send position commands from Python with `deskarm.backends.CanBackend`
   (driver settings it assumes: [software README](../software/README.md), "Driving the real arm"). Use the CANable
   first, then swap in the ESP32-S3 bridge and confirm it behaves the same. Other implementations of the MKS CAN protocol,
   to cross-check frames against:
   [mks_servo_can](https://github.com/ricardodeazambuja/mks_servo_can) (Python, has a simulator you can develop
   `CanBackend` against before the parts arrive) and [MKSServoCAN](https://github.com/TheSpaceEgg/MKSServoCAN) (ESP32).
2. Add the E-stop and the PSU with a fuse. Test that pressing the E-stop actually stops motion.
3. Chain two drivers and address them individually on the CAN bus.
4. Mount the motor in the test-bench gearbox and add a homing sensor or output encoder.
5. Scale up to the full arm.
