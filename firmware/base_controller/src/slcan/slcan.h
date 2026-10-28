// SLCAN (Lawicel) protocol core for the deskarm base controller.
//
// Portable C++ with no hardware or Arduino dependencies: the ESP32 glue (main.cpp) and the host
// test harness both drive this same code. Compatible with python-can's `slcan` interface.
//
// Host -> device commands (each terminated by '\r'):
//   S0..S8   set bitrate (10k,20k,50k,100k,125k,250k,500k,800k,1M), only while closed
//   O / L    open channel (normal / listen-only)        C     close channel
//   tIIILDD..   send standard frame   TIIIIIIIILDD..   send extended frame
//   rIIIL       standard remote frame RIIIIIIIIL       extended remote frame
//   V version   N serial number   F status flags   Z0/Z1 timestamps off/on   M/m filters (ignored)
//   (empty line) -> OK, python-can sends one while setting the bitrate
// Device -> host: '\r' = OK, '\a' = error, 'z\r'/'Z\r' = transmit queued, and received frames
//   in the same t/T/r/R format, followed by a 4-hex-digit millisecond timestamp when Z1 is set.
#pragma once

#include <cstddef>
#include <cstdint>

namespace slcan {

constexpr size_t kMaxLine = 32;        // longest command: T + 8 id + 1 dlc + 16 data = 26 chars
constexpr size_t kMaxFrameText = 32;   // longest output line incl. timestamp and '\r'

struct Frame {
    uint32_t id = 0;
    bool ext = false;
    bool rtr = false;
    uint8_t dlc = 0;
    uint8_t data[8] = {0};
};

// What the protocol needs from the CAN hardware.
class Port {
public:
    virtual ~Port() = default;
    virtual bool open(uint32_t bitrate, bool listen_only) = 0;
    virtual void close() = 0;
    virtual bool transmit(const Frame& frame) = 0;
    virtual uint8_t status_flags() = 0;   // Lawicel 'F' bits: 0 rx full, 1 tx full, 2 err warn,
                                          // 3 overrun, 5 err passive, 6 arb lost, 7 bus error
};

// Where protocol output (acks, frames) goes: USB CDC on the ESP32, a pty on the host.
class Output {
public:
    virtual ~Output() = default;
    virtual void write(const char* data, size_t len) = 0;
};

class Parser {
public:
    Parser(Port& port, Output& out) : port_(port), out_(out) {}

    void feed(char c);                    // feed one byte received from the host
    bool is_open() const { return open_; }
    bool listen_only() const { return listen_only_; }
    bool timestamps() const { return timestamps_; }
    uint32_t bitrate() const { return bitrate_; }

    // Report a frame received on the bus (or a synthetic one, like the status frame).
    void report_frame(const Frame& frame, uint16_t timestamp_ms);

    // Format a frame as an SLCAN line (incl. '\r'). Returns length, 0 if the frame is invalid.
    static size_t format_frame(const Frame& frame, bool with_timestamp, uint16_t timestamp_ms,
                               char* out);
    // Parse a t/T/r/R command (without '\r'). Returns false on any format error.
    static bool parse_frame(const char* line, size_t len, Frame& frame);
    // Bitrate for an 'S' code digit, 0 if unsupported.
    static uint32_t bitrate_for_code(char code);

private:
    void handle_line(const char* line, size_t len);
    void ok() { out_.write("\r", 1); }
    void error() { out_.write("\a", 1); }

    Port& port_;
    Output& out_;
    char line_[kMaxLine + 1] = {0};
    size_t len_ = 0;
    bool overflow_ = false;
    bool open_ = false;
    bool listen_only_ = false;
    bool timestamps_ = false;
    uint32_t bitrate_ = 500000;           // default if the host opens without an S command
};

// ---- deskarm bridge status frame -----------------------------------------------------------
// Synthetic frame the bridge injects into the USB stream (never sent on the CAN bus) so the host
// sees E-stop and bus health as an ordinary CAN message. Decoded by deskarm.bridge on the PC.
constexpr uint32_t kStatusFrameId = 0x7F0;
constexpr uint8_t kFwMajor = 0;
constexpr uint8_t kFwMinor = 2;

struct BridgeStatus {
    bool estop_pressed = false;   // motor power is cut
    bool bus_off = false;
    bool error_passive = false;
    bool rx_overflow = false;     // frames lost since the previous status frame
    bool tx_failed = false;       // a host frame could not be queued since the previous report
    uint8_t tx_error_counter = 0;
    uint8_t rx_error_counter = 0;
    uint8_t dropped = 0;          // frames lost since the previous report, either way (saturating)
    uint16_t uptime_s = 0;
};

Frame encode_status(const BridgeStatus& s);

}  // namespace slcan
