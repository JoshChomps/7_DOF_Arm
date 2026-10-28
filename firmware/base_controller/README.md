# Base controller firmware (ESP32-S3)

USB ↔ CAN bridge for the arm's CAN bus, plus E-stop and bus-health reporting.

- Speaks **SLCAN** (Lawicel), so it works with `python-can` (`interface="slcan"`) on Windows (`COMx`)
  and on the Raspberry Pi / Linux (`/dev/ttyACM0`, or `slcand` → native `socketcan`).
- Sends a **status frame** (ID `0x7F0`) to the PC every 100 ms: E-stop state, bus-off,
  error counters, dropped frames, uptime, firmware version. `deskarm.bridge.decode_status()` decodes it.
- Recovers automatically from CAN **bus-off**.
- The E-stop **cuts motor power in hardware**. This firmware only *reports* it so the software can stop too.

## Wiring (ESP32-S3-DevKitC-1 + SN65HVD230 CAN module)

| ESP32-S3 | Connects to | Notes |
|---|---|---|
| GPIO4 | SN65HVD230 **CTX** (TX) | change in `src/config.h` |
| GPIO5 | SN65HVD230 **CRX** (RX) | |
| 3V3 / GND | SN65HVD230 3V3 / GND | the SN65HVD230 is a 3.3 V part (don't use a 5 V TJA1050 without level shifting) |
| GPIO6 | E-stop **auxiliary NC contact** → GND | released = LOW, pressed or broken wire = HIGH (fail-safe) |
| USB-C "**USB**" port | PC / Raspberry Pi | the native-USB port, **not** the "UART" port |
| GPIO10 / GPIO12 / GPIO13 | bench MT6701 module: CSN / CLK / DO (SSI), plus 3V3 and GND | **optional** (13 Oct): only with `BENCH_ENCODER_JOINT` set in `src/config.h`; reserve the pins on board B |

CAN bus: CANH/CANL twisted pair to the MKS drivers. **120 Ω at both ends of the bus only.** Most
SN65HVD230 modules have one on board (keep it, since the bridge is an end). Disable it on any module in the middle.

## Build & flash (Windows, Arduino IDE)
The folder is an Arduino sketch: `base_controller.ino` is empty on purpose, and the IDE compiles everything under
`src/` with it (`src/main.cpp` has `setup()`/`loop()`, `src/slcan/` is the protocol core).

1. Install **Arduino IDE 2**. In *Boards Manager*, install **esp32 by Espressif Systems**.
2. *File → Open* → `firmware/base_controller/base_controller.ino`.
3. *Tools* menu:

   | Setting | Value | Why |
   |---|---|---|
   | Board | **ESP32S3 Dev Module** | |
   | USB Mode | **Hardware CDC and JTAG** | same as `ARDUINO_USB_MODE=1` |
   | USB CDC On Boot | **Enabled** | `Serial` is the native USB port (`ARDUINO_USB_CDC_ON_BOOT=1`) |
   | Flash Size | 8 MB (16 MB on N16R8 boards; 8 MB also works) | |
   | PSRAM | Disabled | not used |
   | Port | the COM port of the board's **USB** connector | |
4. Plug the board into its **USB** port and click **Upload**. If the upload does not start, hold **BOOT**, tap
   **RST**, release **BOOT**, and upload again.
5. A new COM port appears (Device Manager → Ports). LED: **blue** = idle, **green** = bridging,
   **red** = E-stop pressed. (The RGB LED is GPIO48 on DevKitC-1 v1.0 and GPIO38 on v1.1; see `src/config.h`.
   On many clone boards the LED only works after bridging the small "RGB" solder jumper next to it.)

**Which core version.** The host checks (below) compare `main.cpp` with the TWAI headers of ESP-IDF 4.4.7, which is
inside esp32 core **2.0.x**. Boards Manager installs the newest **3.x** (ESP-IDF 5.x), which still has the same
`driver/twai.h` API but was not compared line by line. Try 3.x first; if the build fails inside the TWAI code, pick
2.0.17 in the Boards Manager version list and build again.

PlatformIO still works as a second route (`platformio.ini`, `pio run -t upload`): it builds the same `src/` folder
with the `esp32-s3-devkitc-1` board definition and core 2.x. The common N16R8 clones work with either; the firmware
doesn't use the pins that octal PSRAM takes (GPIO35-37).

## Try it
`pip install -e "software[hw]"`, then save this as `check_bridge.py` and run `python check_bridge.py`:
```python
from deskarm.bridge import open_bus, decode_status, STATUS_FRAME_ID

bus = open_bus("COM5")                       # your COM port, 500 kbit/s by default; raises if no bridge answers
print(bus.get_version(timeout=1))            # (1, 1)
for msg in bus:
    if msg.arbitration_id == STATUS_FRAME_ID:
        print(decode_status(msg.data))
        break
bus.shutdown()
```
**If nothing answers:** the ESP32-S3's built-in USB port treats the serial RTS/DTR lines as reset signals, so some
hosts reboot the chip when they open (or close) the port. `open_bus` already waits for the firmware to answer and then
re-sends the bitrate and "open" commands, so this is normally invisible. If you use `can.Bus(interface="slcan", ...)`
directly, give it `sleep_after_open=2` instead. A `RuntimeError: no SLCAN bridge answered` means the wrong COM port, the
board's "UART" connector instead of "USB", or firmware that isn't flashed.

Raspberry Pi with native socketcan: `sudo slcand -o -c -s6 /dev/ttyACM0 can0 && sudo ip link set can0 up`
(`-s6` = 500 kbit/s, `-s8` = 1 Mbit/s), then `candump can0`.

## Protocol summary
| Host → bridge | Meaning | Reply |
|---|---|---|
| `S0`…`S8` | bitrate 10k, 20k, 50k, 100k, 125k, 250k, 500k, 800k, 1M (while closed) | `\r` / `\a` |
| `O` / `L` / `C` | open / open listen-only / close | `\r` / `\a` |
| `tIIILDD…` / `TIIIIIIIILDD…` | send standard / extended frame | `z\r` / `Z\r`, `\a` on error |
| `rIIIL` / `RIIIIIIIIL` | remote frames | `z\r` / `Z\r` |
| `V`, `N`, `F`, `Z0`/`Z1`, `M`/`m` | version, serial, status flags, timestamps, filters (accepted, ignored) | |

Status frame `0x7F0` (8 bytes): `[flags, tx_err, rx_err, dropped, uptime_lo, uptime_hi, fw_major, fw_minor]`.
Flags: bit0 E-stop pressed, bit1 bus-off, bit2 error-passive, bit3 RX overflow, bit4 TX failed.
It is **only** sent to the PC, never onto the CAN bus. Keep motor node IDs away from `0x7F0`.

**Bench output encoder (firmware 0.2, 13 Oct).** With `BENCH_ENCODER_JOINT` set to a joint number (e.g. 2 for the J2
bench gearbox; 0 = off, the default), the bridge reads an MT6701 over SSI every 5 ms (bit-banged, 24 bits: 14-bit
angle, 4 status bits, CRC-6) and sends the PC a **joint-angle frame**, the same one board C will put on the bus:

| ID | Bytes | Meaning |
|---|---|---|
| `0x10` + joint (J2 = `0x12`) | 0-1 | angle, 14 bit, big-endian (0-16383 = one output turn) |
| | 2 | bits 0-3 MT6701 status (field too strong / too weak, loss of track), bit 4 CRC ok, bit 7 no sensor (DO reads all ones) |
| | 3 | sequence number (wraps at 256) |

`deskarm.bridge.decode_angle` decodes it; `CanBackend` keeps the latest one per joint in `.output`, and
`deskarm.bench` logs it. The MT6701 datasheet gives the CRC polynomial (x⁶ + x + 1) but no initial value or test
vector: 0 is assumed, which is why a frame carries the "CRC ok" flag instead of being dropped. Check it with the first
real sensor. Like the status frame it only goes to the PC. Build with it on from the command line:
`--build-property "compiler.cpp.extra_flags=-DBENCH_ENCODER_JOINT=2"` (compiles with no warnings, 13 Oct).

## Tests (run on a PC, no hardware needed)
```sh
firmware/base_controller/test/host/run.sh                  # Linux / CI: needs g++
CXX="zig c++" sh firmware/base_controller/test/host/run.sh # Windows (Git Bash) with zig installed, or any C++17 compiler
pytest software/tests/test_bridge.py                       # status decoding and open_bus everywhere; on Linux also
                                                           # real python-can <-> firmware core over a pseudo-terminal
```
`run.sh` does three things: it runs the unit tests of the protocol core (`src/slcan`) and of the MT6701 SSI read,
decoder and angle frame (`src/mt6701`); it compiles `src/main.cpp` against stub headers in `test/host/stubs`, once for
Arduino core 2.x and once for 3.x; and it compiles board C's sketch (`firmware/joint_encoder`, which shares
`src/mt6701`) against the same stubs with three settings. `test/host/pty_bridge` is a
Linux-only PC emulator of this firmware: the same protocol core, with frames looped back.

## What has and hasn't been verified

| Checked | How | Result (1 Oct 2026) |
|---|---|---|
| SLCAN protocol core (parsing, formatting, every command, status frame) | Host unit tests | Pass |
| python-can ↔ protocol core, end to end | pty emulator + pytest, Linux only (runs in CI) | Written to pass in CI; not runnable on Windows |
| `main.cpp` uses the TWAI driver correctly | Every struct field, macro and function it uses was compared with ESP-IDF v4.4.7's `driver/twai.h` and `hal/twai_types.h` (the IDF inside Arduino-ESP32 2.0.x, which PlatformIO's `espressif32` platform ships). The stubs now copy those declarations, including the fact that the `*_CONFIG_*` macros are brace initialisers | Compiles for both Arduino core branches |
| `Serial` calls (`availableForWrite`, `setTxTimeoutMs`, `write`) | Compared with Arduino-ESP32's `HWCDC.h` | Match |
| RGB LED call | `neopixelWrite` (core 2.x) / `rgbLedWrite` (core 3.x): both branches are now compiled | Match |
| Board definition and USB build flags | Compared with PlatformIO's `esp32-s3-devkitc-1.json` | Consistent |
| Arduino IDE sketch layout (6 Oct) | `base_controller.ino` + `src/`; host tests and compile check re-run after the move | Pass. Not yet opened in the Arduino IDE |
| **A real ESP32-S3 build** (13 Oct) | `arduino-cli` 1.5.1, esp32 core 3.3.12, `--fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc,FlashSize=8M --warnings all`; CI builds core 2.0.17 and 3.3.12 on every push | **Compiles, no warnings**, 307 KB (23% of the app partition) |
| MT6701 decode, CRC-6, angle frame (13 Oct) | Host unit tests: round trip, every single-bit error caught, "no sensor", frame bytes | Pass; the CRC's initial value is an assumption |
| MT6701 SSI read, shared with board C (15 Oct) | `mt6701::read_word` (template in `src/mt6701/mt6701.h`, Arduino pins in `mt6701_arduino.h`) against a simulated sensor: bit order, 24 rising edges, idle levels after; a read that samples after the rising edge fails it (checked). Real build re-run, with and without `BENCH_ENCODER_JOINT=2` | Pass; compiles, no warnings |
| Build with the bench encoder on (13 Oct) | Same `arduino-cli` build with `-DBENCH_ENCODER_JOINT=2` | Compiles, no warnings |
| **Flashing, the USB reset behaviour, a real CAN bus, a real MT6701** | - | **Not done.** Needs the board |

Changes made in that review:
- The loop now sleeps 1 ms when there is nothing to do, instead of spinning a core at 100% (no delay is added while
  frames are flowing).
- USB writes can never block (`Serial.setTxTimeoutMs(0)` on top of the existing "drop if the buffer is full" guard).
- The "RX overflow" status flag no longer raises once, falsely, after the channel is closed and reopened.
- The bitrate table initialises each timing struct from its macro the documented way.
- On the PC side, `open_bus` verifies that the bridge answers (see "If nothing answers" above).

So the first upload from the Arduino IDE is still the real build test. If it fails, the error will most likely be in
the build configuration (the *Tools* settings above, the core version), not in the protocol logic. Then check, in this order: the COM
port appears; `open_bus` returns and prints version `(1, 1)`; status frames arrive every 100 ms; the E-stop bit follows
the button; a frame sent from Python appears on a second CAN node (the CANable).
