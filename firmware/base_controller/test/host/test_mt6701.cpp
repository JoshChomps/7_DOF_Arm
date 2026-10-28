// Host unit tests for src/mt6701. Build + run: ./run.sh
#include <cstdio>

#include "mt6701.h"

static int failures = 0;
#define CHECK(cond)                                                              \
    do {                                                                         \
        if (!(cond)) {                                                           \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);          \
            ++failures;                                                          \
        }                                                                        \
    } while (0)

static void test_round_trip() {
    const uint16_t angles[] = {0, 1, 8191, 12345, 16383};
    const uint8_t statuses[] = {0, 1, 2, 8};
    for (uint16_t angle : angles) {
        for (uint8_t status : statuses) {
            const mt6701::Reading r = mt6701::decode(mt6701::encode(angle, status));
            CHECK(r.present && r.crc_ok && r.angle == angle && r.status == status);
        }
    }
}

static void test_every_single_bit_error_is_caught() {
    const uint32_t word = mt6701::encode(12345, 0);
    for (int bit = 0; bit < 24; ++bit) CHECK(!mt6701::decode(word ^ (1u << bit)).crc_ok);
}

static void test_nothing_connected() {
    CHECK(!mt6701::decode(0xFFFFFF).present);      // DO pulled up, no sensor
    CHECK(mt6701::decode(0).present && mt6701::decode(0).crc_ok);   // angle 0, status 0 is a real reading
    CHECK(!mt6701::decode(0xFFFFFF).crc_ok);
}

static void test_angle_frame_layout() {
    const mt6701::Reading r = mt6701::decode(mt6701::encode(0x2A5C, 2));
    const slcan::Frame f = mt6701::angle_frame(2, r, 7);
    CHECK(f.id == 0x12 && f.dlc == 4 && !f.ext);
    CHECK(f.data[0] == 0x2A && f.data[1] == 0x5C);
    CHECK(f.data[2] == (0x02 | 0x10) && f.data[3] == 7);
    const slcan::Frame none = mt6701::angle_frame(2, mt6701::decode(0xFFFFFF), 0);
    CHECK(none.data[2] & 0x80);
}

// An MT6701 on the wire: drives the word's first bit when CSN falls and the next one on each rising CLK edge.
struct SimSensor {
    uint32_t word;
    int bit = 23;
    bool low = false, clk = true;
    int rising_edges = 0;
    void select(bool on) {
        if (on && !low) bit = 23;
        low = on;
    }
    void clock(bool high) {
        if (high && !clk && low) { ++rising_edges; --bit; }
        clk = high;
    }
    bool data() const { return !low || bit < 0 || ((word >> bit) & 1u); }   // pulled up when idle
    void half_clock() {}
};

static void test_ssi_read_bit_order_and_edges() {
    const uint32_t word = mt6701::encode(0x2A5C, 2);
    SimSensor s{word};
    CHECK(mt6701::read_word(s) == word);
    CHECK(s.rising_edges == 24 && !s.low && s.clk);        // released, CLK back at its idle level
    SimSensor again{mt6701::encode(1, 0)};
    CHECK(mt6701::read_word(again) == mt6701::encode(1, 0));
    CHECK(mt6701::read_word(again) == mt6701::encode(1, 0));   // a second read starts from the MSB again
}

int main() {
    test_round_trip();
    test_every_single_bit_error_is_caught();
    test_nothing_connected();
    test_angle_frame_layout();
    test_ssi_read_bit_order_and_edges();
    if (failures) {
        std::printf("%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("mt6701 host tests: all passed\n");
    return 0;
}
