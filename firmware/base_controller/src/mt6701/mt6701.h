// MT6701 magnetic angle sensor over SSI, and the joint-angle CAN frame (portable, host-tested).
//
// SSI word (MT6701 datasheet Rev 0.1, 6.7.2), 24 bits, MSB first:
//   D[13:0] angle (0..16383 = 0..360 deg) | Mg[3:0] status | CRC[5:0]
//   Mg[1:0]: 0 normal, 1 field too strong, 2 field too weak; Mg[2] push button; Mg[3] loss of track.
//   CRC-6 over the first 18 bits, polynomial x^6 + x + 1, MSB first. The datasheet gives no initial
//   value or test vector: 0 is assumed, so the frame carries a "CRC ok" flag instead of the firmware
//   dropping readings. Check it against the first real sensor (docs/16 sec. 3.7).
//
// Joint-angle frame: ID 0x10 + joint number (J2 = 0x12, the ids board C will use, docs/16 sec. 3.8), DLC 4:
//   bytes 0-1  angle, 14 bit, big-endian      byte 2  flags: bits 0-3 Mg, bit 4 CRC ok, bit 7 no sensor
//   byte 3     sequence number (wraps), so the host sees lost frames
#pragma once
#include <cstdint>

#include "../slcan/slcan.h"

namespace mt6701 {

constexpr uint32_t kAngleFrameBase = 0x10;

struct Reading {
    uint16_t angle = 0;       // 0..16383
    uint8_t status = 0;       // Mg[3:0]
    bool crc_ok = false;
    bool present = false;     // false: all ones on DO (pulled up, nothing connected)
};

uint8_t crc6(uint32_t bits18);
Reading decode(uint32_t word24);
uint32_t encode(uint16_t angle, uint8_t status);           // a valid word (tests, and a simulated sensor)
slcan::Frame angle_frame(uint8_t joint, const Reading& r, uint8_t seq);

// One 24-bit SSI word, bit-banged. CLK idles high; the sensor puts the first bit on DO when CSN falls and
// the next one on each rising edge, and the host samples after the falling edge (datasheet v1.9 p.26).
// `io` has select(bool low), clock(bool high), data() and half_clock() (a ~1 us wait): mt6701_arduino.h
// on the boards, a simulated sensor in the host test. 48 half clocks of ~1 us: a ~50 us read.
template <class Io>
uint32_t read_word(Io& io) {
    io.select(true);
    io.half_clock();
    uint32_t word = 0;
    for (int i = 0; i < 24; ++i) {
        io.clock(false);
        io.half_clock();
        word = (word << 1) | (io.data() ? 1u : 0u);
        io.clock(true);
        io.half_clock();
    }
    io.select(false);
    return word;
}

}  // namespace mt6701
