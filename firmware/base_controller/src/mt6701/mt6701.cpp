#include "mt6701.h"

namespace mt6701 {

uint8_t crc6(uint32_t bits18) {
    uint8_t crc = 0;
    for (int i = 17; i >= 0; --i) {
        const uint8_t feedback = ((bits18 >> i) & 1u) ^ ((crc >> 5) & 1u);
        crc = static_cast<uint8_t>((crc << 1) & 0x3F);
        if (feedback) crc ^= 0x03;
    }
    return crc;
}

Reading decode(uint32_t word24) {
    word24 &= 0xFFFFFF;
    Reading r;
    r.present = word24 != 0xFFFFFF;                   // DO is pulled up: all ones = nothing there
    r.angle = static_cast<uint16_t>(word24 >> 10);
    r.status = static_cast<uint8_t>((word24 >> 6) & 0x0F);
    r.crc_ok = r.present && crc6(word24 >> 6) == (word24 & 0x3F);
    return r;
}

uint32_t encode(uint16_t angle, uint8_t status) {
    const uint32_t bits18 = (static_cast<uint32_t>(angle & 0x3FFF) << 4) | (status & 0x0Fu);
    return (bits18 << 6) | crc6(bits18);
}

slcan::Frame angle_frame(uint8_t joint, const Reading& r, uint8_t seq) {
    slcan::Frame f;
    f.id = kAngleFrameBase + joint;
    f.dlc = 4;
    f.data[0] = static_cast<uint8_t>(r.angle >> 8);
    f.data[1] = static_cast<uint8_t>(r.angle & 0xFF);
    f.data[2] = static_cast<uint8_t>((r.status & 0x0F) | (r.crc_ok ? 0x10 : 0) | (r.present ? 0 : 0x80));
    f.data[3] = seq;
    return f;
}

}  // namespace mt6701
