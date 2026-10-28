// Host unit tests for src/slcan. Build + run: ./run.sh
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "slcan.h"

static int failures = 0;
#define CHECK(cond)                                                              \
    do {                                                                         \
        if (!(cond)) {                                                           \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);          \
            ++failures;                                                          \
        }                                                                        \
    } while (0)

struct MockPort : slcan::Port {
    bool open_ok = true, tx_ok = true, is_open = false, listen = false;
    uint32_t rate = 0;
    std::vector<slcan::Frame> sent;
    bool open(uint32_t bitrate, bool listen_only) override {
        if (!open_ok) return false;
        is_open = true; rate = bitrate; listen = listen_only;
        return true;
    }
    void close() override { is_open = false; }
    bool transmit(const slcan::Frame& f) override { if (tx_ok) sent.push_back(f); return tx_ok; }
    uint8_t status_flags() override { return 0xA5; }
};

struct MockOut : slcan::Output {
    std::string buf;
    void write(const char* d, size_t n) override { buf.append(d, n); }
};

struct Rig {
    MockPort port;
    MockOut out;
    slcan::Parser p{port, out};
    std::string cmd(const std::string& s) {   // send one command, return the reply
        out.buf.clear();
        for (char c : s) p.feed(c);
        p.feed('\r');
        return out.buf;
    }
};

static void test_open_close_and_bitrate() {
    Rig r;
    CHECK(r.cmd("S8") == "\r");
    CHECK(r.p.bitrate() == 1000000);
    CHECK(r.cmd("S9") == "\a");               // unsupported code
    CHECK(r.cmd("O") == "\r");
    CHECK(r.port.is_open && r.port.rate == 1000000 && !r.port.listen);
    CHECK(r.cmd("S6") == "\a");               // can't change bitrate while open
    CHECK(r.cmd("O") == "\a");                // already open
    CHECK(r.cmd("C") == "\r");
    CHECK(!r.port.is_open);
    CHECK(r.cmd("C") == "\r");                // closing twice is fine (python-can does it)
    CHECK(r.cmd("L") == "\r");
    CHECK(r.port.listen && r.p.listen_only());
}

static void test_python_can_open_sequence() {
    Rig r;                                    // python-can: C, S6, <empty line>, O
    CHECK(r.cmd("C") == "\r");
    CHECK(r.cmd("S6") == "\r");
    CHECK(r.cmd("") == "\r");
    CHECK(r.cmd("O") == "\r");
    CHECK(r.port.rate == 500000);
}

static void test_transmit() {
    Rig r;
    CHECK(r.cmd("t1232ABCD") == "\a");        // closed -> error
    r.cmd("O");
    CHECK(r.cmd("t1232ABCD") == "z\r");
    CHECK(r.port.sent.size() == 1);
    const slcan::Frame& f = r.port.sent[0];
    CHECK(f.id == 0x123 && !f.ext && !f.rtr && f.dlc == 2 && f.data[0] == 0xAB && f.data[1] == 0xCD);
    CHECK(r.cmd("T1ABCDEF08" "0011223344556677") == "Z\r");
    CHECK(r.port.sent[1].ext && r.port.sent[1].id == 0x1ABCDEF0 && r.port.sent[1].data[7] == 0x77);
    CHECK(r.cmd("r7FF4") == "z\r");
    CHECK(r.port.sent[2].rtr && r.port.sent[2].dlc == 4);
    CHECK(r.cmd("R000000013") == "Z\r");
    CHECK(r.cmd("t8001") == "\a");            // id > 0x7FF
    CHECK(r.cmd("T2000000000") == "\a");      // id > 0x1FFFFFFF
    CHECK(r.cmd("t1239") == "\a");            // dlc > 8
    CHECK(r.cmd("t1232AB") == "\a");          // too little data
    CHECK(r.cmd("t1231ABCD") == "\a");        // too much data
    CHECK(r.cmd("t12G1AB") == "\a");          // bad hex
    r.port.tx_ok = false;
    CHECK(r.cmd("t1230") == "\a");            // hardware queue full
    r.cmd("C");
    r.cmd("L");
    r.port.tx_ok = true;
    CHECK(r.cmd("t1230") == "\a");            // listen-only can't transmit
}

static void test_info_commands() {
    Rig r;
    CHECK(r.cmd("V") == "V0101\r");
    CHECK(r.cmd("N") == "NDA01\r");
    CHECK(r.cmd("F") == "\a");                // only while open
    r.cmd("O");
    CHECK(r.cmd("F") == "FA5\r");
    CHECK(r.cmd("Z1") == "\r" && r.p.timestamps());
    CHECK(r.cmd("Z2") == "\a");
    CHECK(r.cmd("M00000000") == "\r");
    CHECK(r.cmd("mFFFFFFFF") == "\r");
    CHECK(r.cmd("X") == "\a");
}

static void test_format_and_report() {
    slcan::Frame f;
    f.id = 0x7F0; f.dlc = 3; f.data[0] = 1; f.data[1] = 0xAB; f.data[2] = 0xFF;
    char buf[slcan::kMaxFrameText];
    size_t n = slcan::Parser::format_frame(f, false, 0, buf);
    CHECK(std::string(buf, n) == "t7F0301ABFF\r");
    n = slcan::Parser::format_frame(f, true, 0x1234, buf);
    CHECK(std::string(buf, n) == "t7F0301ABFF1234\r");
    f.ext = true; f.id = 0x18FF50E5; f.rtr = true; f.dlc = 8;
    n = slcan::Parser::format_frame(f, false, 0, buf);
    CHECK(std::string(buf, n) == "R18FF50E58\r");
    f.dlc = 9;
    CHECK(slcan::Parser::format_frame(f, false, 0, buf) == 0);

    Rig r;
    slcan::Frame g; g.id = 1; g.dlc = 1; g.data[0] = 0x42;
    r.p.report_frame(g, 0);
    CHECK(r.out.buf.empty());                 // nothing reported while closed
    r.cmd("O");
    r.out.buf.clear();
    r.p.report_frame(g, 0);
    CHECK(r.out.buf == "t0011" "42\r");
}

static void test_round_trip_all_shapes() {
    for (int ext = 0; ext < 2; ++ext)
        for (int rtr = 0; rtr < 2; ++rtr)
            for (int dlc = 0; dlc <= 8; ++dlc) {
                slcan::Frame f;
                f.ext = ext; f.rtr = rtr; f.dlc = dlc; f.id = ext ? 0x1F2E3D4C : 0x5A5;
                for (int i = 0; i < 8; ++i) f.data[i] = rtr ? 0 : static_cast<uint8_t>(i * 37 + dlc);
                for (int i = dlc; i < 8; ++i) f.data[i] = 0;
                char buf[slcan::kMaxFrameText];
                size_t n = slcan::Parser::format_frame(f, false, 0, buf);
                slcan::Frame g;
                CHECK(slcan::Parser::parse_frame(buf, n - 1, g));
                CHECK(g.id == f.id && g.ext == f.ext && g.rtr == f.rtr && g.dlc == f.dlc);
                CHECK(std::memcmp(g.data, f.data, 8) == 0);
            }
}

static void test_line_handling() {
    Rig r;
    std::string s = "V\r\nN\n\r";             // CRLF tolerated, stray LF ignored
    for (char c : s) r.p.feed(c);
    CHECK(r.out.buf == "V0101\rNDA01\r");
    r.out.buf.clear();
    for (int i = 0; i < 100; ++i) r.p.feed('t');
    r.p.feed('\r');
    CHECK(r.out.buf == "\a");                 // overlong line rejected, parser recovers
    CHECK(r.cmd("V") == "V0101\r");
}

static void test_status_frame() {
    slcan::BridgeStatus s;
    s.estop_pressed = true; s.rx_overflow = true; s.tx_error_counter = 7; s.rx_error_counter = 9;
    s.dropped = 3; s.uptime_s = 0x1234;
    slcan::Frame f = slcan::encode_status(s);
    CHECK(f.id == slcan::kStatusFrameId && f.dlc == 8 && !f.ext);
    CHECK(f.data[0] == 0x09 && f.data[1] == 7 && f.data[2] == 9 && f.data[3] == 3);
    CHECK(f.data[4] == 0x34 && f.data[5] == 0x12);
    CHECK(f.data[6] == slcan::kFwMajor && f.data[7] == slcan::kFwMinor);
}

int main() {
    test_open_close_and_bitrate();
    test_python_can_open_sequence();
    test_transmit();
    test_info_commands();
    test_format_and_report();
    test_round_trip_all_shapes();
    test_line_handling();
    test_status_frame();
    if (failures) {
        std::printf("%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("slcan host tests: all passed\n");
    return 0;
}
