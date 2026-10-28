// Board C wiring and behaviour (hardware/README.md, "Board C"). Change pins here to match the board.
#pragma once

// Which joint this board reads. Its frame goes out as ID 0x10 + JOINT_ID (J2 = 0x12, D34), the joint's MKS
// driver id. Compiled in for now: the ID scheme (compiled in, flash, or straps on IO0/IO1) is Josh's open
// choice (hardware/README.md, board C list item 6). Override with -DJOINT_ID=3 etc.
#ifndef JOINT_ID
#define JOINT_ID 2
#endif

// CAN bus speed. The MKS drivers' default is 500 kbit/s; they also do 125k, 250k and 1M.
#ifndef CAN_BITRATE
#define CAN_BITRATE 500000
#endif

// SN65HVD230: IO4 -> D (TX), IO5 <- R (RX), the same numbers as board B.
#define PIN_CAN_TX 4
#define PIN_CAN_RX 5

// MT6701 over SSI. Board B / the base controller uses IO10/12/13; the C3 has no IO12/13.
#define PIN_ENC_CSN 10
#define PIN_ENC_CLK 6
#define PIN_ENC_DO 7
#define ENCODER_PERIOD_MS 5

// Green status LED through 1 k. Assumed to sink to GND (on = HIGH); not checked against the schematic's net.
#define PIN_LED 3
#define LED_ON HIGH

// One human-readable line on the USB serial port (USB Serial/JTAG) per this many ms, for bring-up on the desk.
#define REPORT_PERIOD_MS 500
