// The MT6701's SSI pins on an Arduino board, for mt6701::read_word() (mt6701.h). Shared by the base
// controller (bench encoder) and board C (firmware/joint_encoder):  mt6701::read_word(ssi)
#pragma once
#include <Arduino.h>

#include "mt6701.h"

namespace mt6701 {

struct ArduinoSsi {
    uint8_t csn, clk, dout;

    void begin() const {
        pinMode(csn, OUTPUT);
        pinMode(clk, OUTPUT);
        pinMode(dout, INPUT_PULLUP);          // nothing connected reads all ones: "no sensor"
        digitalWrite(csn, HIGH);
        digitalWrite(clk, HIGH);
    }
    void select(bool on) const { digitalWrite(csn, on ? LOW : HIGH); }
    void clock(bool high) const { digitalWrite(clk, high ? HIGH : LOW); }
    bool data() const { return digitalRead(dout) != LOW; }
    void half_clock() const { delayMicroseconds(1); }
};

}  // namespace mt6701
