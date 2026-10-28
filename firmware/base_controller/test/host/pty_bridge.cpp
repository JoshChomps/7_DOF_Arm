// PC emulator of the base controller: runs the real slcan core on a pseudo-terminal so the host
// software (python-can "slcan", deskarm) can be tested without hardware.
//
// Prints the pty path on the first line of stdout. Transmitted frames are looped back as received
// frames (like a bus with an echoing node), and status frames (ID 0x7F0) are sent every 100 ms.
// Usage: ./pty_bridge [--estop]
#include <fcntl.h>
#include <poll.h>
#include <pty.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <deque>

#include "slcan.h"

namespace {

class FdOutput : public slcan::Output {
public:
    explicit FdOutput(int fd) : fd_(fd) {}
    void write(const char* d, size_t n) override {
        while (n > 0) {
            ssize_t w = ::write(fd_, d, n);
            if (w <= 0) return;
            d += w;
            n -= static_cast<size_t>(w);
        }
    }
private:
    int fd_;
};

class LoopbackPort : public slcan::Port {
public:
    bool open(uint32_t, bool) override { return true; }
    void close() override { pending.clear(); }
    bool transmit(const slcan::Frame& f) override { pending.push_back(f); return true; }
    uint8_t status_flags() override { return 0; }
    std::deque<slcan::Frame> pending;
};

uint32_t now_ms() {
    using namespace std::chrono;
    return static_cast<uint32_t>(
        duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count());
}

}  // namespace

int main(int argc, char** argv) {
    const bool estop = argc > 1 && std::strcmp(argv[1], "--estop") == 0;
    int master = -1, slave = -1;
    char name[256] = {0};
    if (openpty(&master, &slave, name, nullptr, nullptr) != 0) {
        std::perror("openpty");
        return 1;
    }
    termios tio;
    tcgetattr(slave, &tio);
    cfmakeraw(&tio);                          // no CR/LF translation
    tcsetattr(slave, TCSANOW, &tio);
    std::printf("%s\n", name);
    std::fflush(stdout);

    FdOutput out(master);
    LoopbackPort port;
    slcan::Parser parser(port, out);
    const uint32_t t0 = now_ms();
    uint32_t last_status = 0;

    for (;;) {
        pollfd pfd{master, POLLIN, 0};
        if (poll(&pfd, 1, 5) > 0 && (pfd.revents & POLLIN)) {
            char buf[256];
            ssize_t n = read(master, buf, sizeof(buf));
            if (n <= 0) continue;
            for (ssize_t i = 0; i < n; ++i) parser.feed(buf[i]);
        }
        const uint32_t t = now_ms() - t0;
        while (!port.pending.empty()) {
            parser.report_frame(port.pending.front(), static_cast<uint16_t>(t % 60000));
            port.pending.pop_front();
        }
        if (parser.is_open() && t - last_status >= 100) {
            slcan::BridgeStatus st;
            st.estop_pressed = estop;
            st.uptime_s = static_cast<uint16_t>(t / 1000);
            parser.report_frame(slcan::encode_status(st), static_cast<uint16_t>(t % 60000));
            last_status = t;
        }
    }
}
