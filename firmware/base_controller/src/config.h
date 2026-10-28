// Board wiring and behaviour. Change pins here to match your board/PCB.
#pragma once

// CAN transceiver (SN65HVD230 module: CTX -> TX pin, CRX -> RX pin, 3V3, GND).
#define PIN_CAN_TX 4
#define PIN_CAN_RX 5

// E-stop sense input. Wire the E-stop's auxiliary NC contact between this pin and GND:
// released = contact closed = LOW, pressed (or wire broken) = HIGH -> fail-safe.
#define PIN_ESTOP 6
#define ESTOP_PRESSED_LEVEL HIGH
#define ESTOP_DEBOUNCE_MS 20

// On-board addressable RGB LED: GPIO48 on ESP32-S3-DevKitC-1 v1.0, GPIO38 on v1.1. -1 = none.
#define PIN_RGB_LED 48
#define LED_BRIGHTNESS 24

// Status frame (ID 0x7F0, see slcan.h) injected into the USB stream while the channel is open.
#define STATUS_PERIOD_MS 100

// Bench output encoder: an MT6701 module on spare pins, read over SSI and sent to
// the PC as a joint-angle frame (src/mt6701/mt6701.h), the same frame board C will send. 0 = off; e.g. 2
// for the J2 bench gearbox (frame ID 0x12). Pins are the ESP32-S3's FSPI defaults (CS0, CLK, MISO).
#ifndef BENCH_ENCODER_JOINT
#define BENCH_ENCODER_JOINT 0
#endif
#define PIN_ENC_CSN 10
#define PIN_ENC_CLK 12
#define PIN_ENC_DO 13
#define ENCODER_PERIOD_MS 5
