// Compile-check stub of the Arduino API used by src/main.cpp (host only, not functional).
//
// Signatures follow Arduino-ESP32's HWCDC (the `Serial` object when ARDUINO_USB_MODE=1 and
// ARDUINO_USB_CDC_ON_BOOT=1) and esp32-hal-rgb-led.h. Build with -DSTUB_ARDUINO_MAJOR=2 or 3 to
// check both branches of set_led(): core 2.x has neopixelWrite(), core 3.x renamed it rgbLedWrite().
#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>
#ifndef STUB_ARDUINO_MAJOR
#define STUB_ARDUINO_MAJOR 2
#endif
#define HIGH 1
#define LOW 0
#define INPUT 0x01
#define OUTPUT 0x03
#define INPUT_PULLUP 0x05
#define ESP_ARDUINO_VERSION_MAJOR STUB_ARDUINO_MAJOR
struct StubSerial {
    void begin(unsigned long = 0) {}
    void setTxTimeoutMs(uint32_t) {}
    int available(void) { return 0; }
    int availableForWrite(void) { return 256; }
    int read(void) { return -1; }
    size_t write(uint8_t) { return 1; }
    size_t write(const uint8_t*, size_t n) { return n; }
};
inline StubSerial Serial;
inline unsigned long millis() { return 0; }
inline void delay(uint32_t) {}
inline int digitalRead(uint8_t) { return LOW; }
inline void digitalWrite(uint8_t, uint8_t) {}
inline void delayMicroseconds(uint32_t) {}
inline void pinMode(uint8_t, uint8_t) {}
#if STUB_ARDUINO_MAJOR >= 3
inline void rgbLedWrite(uint8_t, uint8_t, uint8_t, uint8_t) {}
#else
inline void neopixelWrite(uint8_t, uint8_t, uint8_t, uint8_t) {}
#endif
void setup();
void loop();
