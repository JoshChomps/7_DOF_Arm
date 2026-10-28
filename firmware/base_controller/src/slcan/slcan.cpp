#include "slcan.h"

#include <cstring>

namespace slcan {
namespace {

const char kHex[] = "0123456789ABCDEF";

int hex_value(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

bool parse_hex(const char* s, size_t n, uint32_t& value) {
    value = 0;
    for (size_t i = 0; i < n; ++i) {
        int v = hex_value(s[i]);
        if (v < 0) return false;
        value = (value << 4) | static_cast<uint32_t>(v);
    }
    return true;
}

void put_hex(char*& p, uint32_t value, int digits) {
    for (int i = digits - 1; i >= 0; --i) *p++ = kHex[(value >> (4 * i)) & 0xF];
}

}  // namespace

uint32_t Parser::bitrate_for_code(char code) {
    switch (code) {
        case '0': return 10000;
        case '1': return 20000;
        case '2': return 50000;
        case '3': return 100000;
        case '4': return 125000;
        case '5': return 250000;
        case '6': return 500000;
        case '7': return 800000;
        case '8': return 1000000;
        default: return 0;
    }
}

bool Parser::parse_frame(const char* line, size_t len, Frame& f) {
    if (len < 1) return false;
    const char kind = line[0];
    const bool ext = (kind == 'T' || kind == 'R');
    const bool rtr = (kind == 'r' || kind == 'R');
    if (kind != 't' && kind != 'T' && kind != 'r' && kind != 'R') return false;
    const size_t id_len = ext ? 8 : 3;
    if (len < 1 + id_len + 1) return false;

    uint32_t id = 0;
    if (!parse_hex(line + 1, id_len, id)) return false;
    if (id > (ext ? 0x1FFFFFFFu : 0x7FFu)) return false;
    const int dlc = hex_value(line[1 + id_len]);
    if (dlc < 0 || dlc > 8) return false;

    const size_t data_start = 2 + id_len;
    const size_t expected = rtr ? data_start : data_start + 2 * static_cast<size_t>(dlc);
    if (len != expected) return false;

    f = Frame{};
    f.id = id;
    f.ext = ext;
    f.rtr = rtr;
    f.dlc = static_cast<uint8_t>(dlc);
    if (!rtr) {
        for (int i = 0; i < dlc; ++i) {
            uint32_t b = 0;
            if (!parse_hex(line + data_start + 2 * i, 2, b)) return false;
            f.data[i] = static_cast<uint8_t>(b);
        }
    }
    return true;
}

size_t Parser::format_frame(const Frame& f, bool with_timestamp, uint16_t timestamp_ms, char* out) {
    if (f.dlc > 8 || f.id > (f.ext ? 0x1FFFFFFFu : 0x7FFu)) return 0;
    char* p = out;
    *p++ = f.rtr ? (f.ext ? 'R' : 'r') : (f.ext ? 'T' : 't');
    put_hex(p, f.id, f.ext ? 8 : 3);
    *p++ = kHex[f.dlc];
    if (!f.rtr) {
        for (int i = 0; i < f.dlc; ++i) put_hex(p, f.data[i], 2);
    }
    if (with_timestamp) put_hex(p, timestamp_ms, 4);
    *p++ = '\r';
    return static_cast<size_t>(p - out);
}

void Parser::report_frame(const Frame& frame, uint16_t timestamp_ms) {
    if (!open_) return;
    char buf[kMaxFrameText];
    const size_t n = format_frame(frame, timestamps_, timestamp_ms, buf);
    if (n) out_.write(buf, n);
}

void Parser::feed(char c) {
    if (c == '\n') return;                      // tolerate CRLF terminals
    if (c == '\r') {
        if (overflow_) {
            error();
        } else {
            line_[len_] = '\0';
            handle_line(line_, len_);
        }
        len_ = 0;
        overflow_ = false;
        return;
    }
    if (len_ >= kMaxLine) {
        overflow_ = true;                       // swallow the rest of the line, then error
        return;
    }
    line_[len_++] = c;
}

void Parser::handle_line(const char* line, size_t len) {
    if (len == 0) {                             // python-can writes an empty line in set_bitrate
        ok();
        return;
    }
    switch (line[0]) {
        case 'S': {
            const uint32_t rate = (len == 2) ? bitrate_for_code(line[1]) : 0;
            if (open_ || rate == 0) return error();
            bitrate_ = rate;
            return ok();
        }
        case 'O':
        case 'L': {
            if (len != 1 || open_) return error();
            const bool listen = (line[0] == 'L');
            if (!port_.open(bitrate_, listen)) return error();
            open_ = true;
            listen_only_ = listen;
            return ok();
        }
        case 'C':
            if (len != 1) return error();
            if (open_) port_.close();           // closing an already-closed channel is fine
            open_ = false;
            listen_only_ = false;
            return ok();
        case 't':
        case 'T':
        case 'r':
        case 'R': {
            Frame f;
            if (!open_ || listen_only_ || !parse_frame(line, len, f)) return error();
            if (!port_.transmit(f)) return error();
            const bool ext = (line[0] == 'T' || line[0] == 'R');
            out_.write(ext ? "Z\r" : "z\r", 2);
            return;
        }
        case 'V':
            if (len != 1) return error();
            out_.write("V0101\r", 6);
            return;
        case 'N':
            if (len != 1) return error();
            out_.write("NDA01\r", 6);
            return;
        case 'F': {
            if (len != 1 || !open_) return error();
            char buf[4] = {'F', 0, 0, '\r'};
            const uint8_t flags = port_.status_flags();
            buf[1] = kHex[flags >> 4];
            buf[2] = kHex[flags & 0xF];
            out_.write(buf, 4);
            return;
        }
        case 'Z':
            if (len != 2 || (line[1] != '0' && line[1] != '1')) return error();
            timestamps_ = (line[1] == '1');
            return ok();
        case 'M':
        case 'm': {
            uint32_t ignored = 0;
            if (len != 9 || !parse_hex(line + 1, 8, ignored)) return error();
            return ok();                        // acceptance filters: accept everything
        }
        default:
            return error();
    }
}

Frame encode_status(const BridgeStatus& s) {
    Frame f;
    f.id = kStatusFrameId;
    f.dlc = 8;
    f.data[0] = static_cast<uint8_t>((s.estop_pressed ? 0x01 : 0) | (s.bus_off ? 0x02 : 0) |
                                     (s.error_passive ? 0x04 : 0) | (s.rx_overflow ? 0x08 : 0) |
                                     (s.tx_failed ? 0x10 : 0));
    f.data[1] = s.tx_error_counter;
    f.data[2] = s.rx_error_counter;
    f.data[3] = s.dropped;
    f.data[4] = static_cast<uint8_t>(s.uptime_s & 0xFF);
    f.data[5] = static_cast<uint8_t>(s.uptime_s >> 8);
    f.data[6] = kFwMajor;
    f.data[7] = kFwMinor;
    return f;
}

}  // namespace slcan
