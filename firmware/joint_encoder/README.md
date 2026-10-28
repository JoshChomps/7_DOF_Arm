# Joint encoder firmware (board C, ESP32-C3)

One board C per encoded joint (J2, J3, J4 first; D33). It reads the joint's MT6701 over SSI every 5 ms and puts the
**joint-angle frame** (D34) on the arm's CAN bus. The host does the coarse + fine sum ([docs/16](../../docs/16_packaging_and_routing.md)
§3.3); this board only reports the raw angle. Board: [hardware/README.md](../../hardware/README.md), "Board C".

| ID | Bytes | Meaning |
|---|---|---|
| `0x10` + `JOINT_ID` (J2 = `0x12`) | 0-1 | angle, 14 bit, big-endian (0-16383 = one output turn) |
| | 2 | bits 0-3 MT6701 status (field too strong / too weak, loss of track), bit 4 CRC ok, bit 7 no sensor |
| | 3 | sequence number (wraps at 256) |

It is the frame the base controller sends for the bench MT6701, made by the same code: the SSI read, the decoder and the
frame builder are `firmware/base_controller/src/mt6701` (host-tested there), **shared, not copied**. The base controller forwards
every bus frame to the PC over SLCAN, so `deskarm.bridge.decode_angle` and `CanBackend` read board C's frames
unchanged. Don't run the bench encoder (`BENCH_ENCODER_JOINT`) and a board C with the same joint at once: the PC would
get two `0x12` streams.

## Pins (from the board C schematic)

| ESP32-C3 | Connects to | In `src/config.h` |
|---|---|---|
| IO4 / IO5 | SN65HVD230 D (TX) / R (RX) | `PIN_CAN_TX`, `PIN_CAN_RX` |
| IO10 / IO6 / IO7 | MT6701 CSN / CLK / DO (SSI; 10 k pull-ups on CSN and DO) | `PIN_ENC_*` |
| IO3 | green status LED, 1 k | `PIN_LED`, `LED_ON` (HIGH assumed, not checked against the net) |
| IO18 / IO19 | USB-C (USB Serial/JTAG): flashing and the serial report | - |

**Settings** (compile-time, in `src/config.h` or with `-D`): `JOINT_ID` (default 2; 1-7, the joint's MKS driver id, so
the frame is `0x11`, `0x17`) and `CAN_BITRATE` (default 500000; 125000, 250000 or 1000000, the MKS drivers' other
speeds). Anything else does not build. Compiling the id in is a stop-gap: how a board learns its joint (compiled in,
flash, or straps on IO0/IO1) is Josh's open choice (hardware/README.md, board C item 6).

**What it does at run time**

- Every 5 ms: one 24-bit SSI read (`mt6701::read_word`, bit-banged, ~50 µs, shared with the base controller), then one frame to the TWAI
  driver with no wait. If the TX queue is full (no other node on the bus to ACK) the frame is dropped and counted.
- Bus-off is recovered automatically. Received frames are read and ignored (nothing is addressed to board C yet).
- LED: **on** = sensor and bus fine; **1 Hz blink** = sensor absent, CRC bad, field too strong/weak or loss of track;
  **5 Hz blink** = CAN fault (driver not running, bus-off or error-passive, e.g. no other node to ACK).
- Every 500 ms one line on the USB serial port, e.g. `J2 angle 8192 (180.00 deg) flags 0x10 seq 37 | CAN ok tec 0 rec 0 dropped 0`.
  So a board on the desk, powered from USB alone, shows the sensor working before it goes near the bus.

Bus load (hand estimate): a 4-byte standard frame is 79 bits, ~98 with worst-case stuffing, so 200 frames/s is
~20 kbit/s, **~4% of 500 kbit/s per board**, ~12% for J2-J4. [docs/04](../../docs/04_electronics_control.md)'s bus budget
should include it.

## Build

The Arduino build copies a sketch to a temporary folder before compiling it, so a relative `#include` that leaves the
sketch folder breaks (checked 15 Oct). The shared code therefore comes in as a **library**: the folder
`firmware/base_controller/src/mt6701` (it includes `../slcan/slcan.h` for the frame type, so `slcan` has to sit beside it).

**arduino-cli** (the verified route; from the repo root, ~1 min):

```bash
/d/Arduino/cli/arduino-cli --config-file D:/Arduino/arduino-cli.yaml compile --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc --warnings all --library firmware/base_controller/src/mt6701 firmware/joint_encoder
```

Add `--build-property "compiler.cpp.extra_flags=-DJOINT_ID=3"` for another joint, and `-u -p COMx` to upload.

**Arduino IDE**: make the two shared folders visible as libraries once, with directory junctions in the sketchbook's
`libraries` folder (both are needed: through a junction, `../slcan` is looked up next to it). In `cmd`:

```bat
mklink /J "%USERPROFILE%\OneDrive\Documents\Arduino\libraries\mt6701" <path-to>\7_DOF_Arm\firmware\base_controller\src\mt6701
mklink /J "%USERPROFILE%\OneDrive\Documents\Arduino\libraries\slcan" <path-to>\7_DOF_Arm\firmware\base_controller\src\slcan
```

Then open `joint_encoder.ino`, board **ESP32C3 Dev Module**, **USB CDC On Boot: Enabled**, the board's COM port, Upload.
If the upload does not start, hold BOOT (IO9), tap RESET, release BOOT. The junction route was checked with
`arduino-cli --libraries` on a folder holding the two junctions (it fails with only the `mt6701` one); it was not
tried in the IDE itself.

## Tests

`firmware/base_controller/test/host/run.sh` (see that README) also compiles this sketch's `src/main.cpp` against the
base controller's host stubs, three times: the defaults, `JOINT_ID=7` at 1 Mbit/s and `JOINT_ID=1` at 125 kbit/s. The
SSI read (against a simulated sensor), decoder and frame it uses are unit-tested there.

## What has and hasn't been verified (15 Oct 2026)

| Checked | How | Result |
|---|---|---|
| Host compile check | `CXX="zig c++" sh firmware/base_controller/test/host/run.sh`, three settings; a bad `JOINT_ID` (9) or bitrate (800k) refuses to build | Pass |
| **A real ESP32-C3 build** | `arduino-cli` 1.5.1, esp32 core 3.3.12, `esp32:esp32:esp32c3:CDCOnBoot=cdc`, `--warnings all`, `--library` as above | **Compiles, no warnings**, 303 KB (23% of the app partition), 14.5 KB RAM |
| Same build, `JOINT_ID=3`, shared code through two junctions (`--libraries`) | as above | Compiles |
| Core 2.0.x (ESP-IDF 4.4) build | CI `firmware` job (2.0.17 and 3.3.12, added 15 Oct) | **Not run yet**: it runs on the next push |
| **Flashing, the USB Serial/JTAG port, the LED polarity, a real MT6701, a real CAN bus** | - | **Not done.** Nothing here has run on hardware; board C is not ordered |

Known gaps: the MT6701 CRC's initial value is still an assumption (see the base controller README). (Until later on
15 Oct the SSI bit-bang was repeated from the base controller's `main.cpp`; it is now `mt6701::read_word` in
`src/mt6701/mt6701.h`, with the Arduino pins in `mt6701_arduino.h`, used by both sketches.)
