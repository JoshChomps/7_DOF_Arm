// deskarm base controller firmware (ESP32-S3).
//
// * USB CDC <-> CAN bridge speaking SLCAN, so `python-can` (interface="slcan") works on Windows
//   (COMx) and Linux/Raspberry Pi (/dev/ttyACM0, or `slcand` for native socketcan).
// * Monitors the E-stop and CAN bus health and reports them to the PC as a status frame
//   (ID 0x7F0) every 100 ms. The E-stop itself cuts motor power in hardware; this is reporting.
// * Recovers automatically from CAN bus-off.
// * Optionally reads a bench MT6701 output encoder and reports it as a joint-angle frame
//   (BENCH_ENCODER_JOINT in config.h; the frame is the one board C will send, src/mt6701).
//
// The protocol logic lives in src/slcan and src/mt6701 (portable, unit-tested on the PC: see test/host).
#include <Arduino.h>

#include "config.h"
#include "driver/twai.h"
#include "mt6701/mt6701_arduino.h"
#include "slcan/slcan.h"

namespace {

class UsbOutput : public slcan::Output {
public:
    void write(const char* data, size_t len) override {
        // Never block the CAN loop if the PC isn't reading: drop and count instead.
        if (Serial.availableForWrite() < static_cast<int>(len)) {
            ++usb_dropped;
            return;
        }
        Serial.write(reinterpret_cast<const uint8_t*>(data), len);
    }
    uint32_t usb_dropped = 0;
};

constexpr uint32_t kRxQueueLen = 64;
constexpr uint32_t kTxQueueLen = 32;

// The ESP-IDF TWAI_TIMING_CONFIG_* macros are brace initialisers ({.brp = 8, ...}), so they can
// only *initialise* a variable. Each case therefore builds a local and copies it out.
#define TIMING_CASE(rate, macro)                \
    case rate: {                                \
        const twai_timing_config_t c = macro(); \
        *t = c;                                 \
        return true;                            \
    }

bool timing_for(uint32_t bitrate, twai_timing_config_t* t) {
    switch (bitrate) {
#ifdef TWAI_TIMING_CONFIG_10KBITS
        TIMING_CASE(10000, TWAI_TIMING_CONFIG_10KBITS)
#endif
#ifdef TWAI_TIMING_CONFIG_20KBITS
        TIMING_CASE(20000, TWAI_TIMING_CONFIG_20KBITS)
#endif
        TIMING_CASE(50000, TWAI_TIMING_CONFIG_50KBITS)
        TIMING_CASE(100000, TWAI_TIMING_CONFIG_100KBITS)
        TIMING_CASE(125000, TWAI_TIMING_CONFIG_125KBITS)
        TIMING_CASE(250000, TWAI_TIMING_CONFIG_250KBITS)
        TIMING_CASE(500000, TWAI_TIMING_CONFIG_500KBITS)
        TIMING_CASE(800000, TWAI_TIMING_CONFIG_800KBITS)
        TIMING_CASE(1000000, TWAI_TIMING_CONFIG_1MBITS)
        default: return false;
    }
}
#undef TIMING_CASE

class TwaiPort : public slcan::Port {
public:
    bool open(uint32_t bitrate, bool listen_only) override {
        twai_timing_config_t timing;
        if (!timing_for(bitrate, &timing)) return false;
        twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(
            static_cast<gpio_num_t>(PIN_CAN_TX), static_cast<gpio_num_t>(PIN_CAN_RX),
            listen_only ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL);
        general.rx_queue_len = kRxQueueLen;
        general.tx_queue_len = kTxQueueLen;
        twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
        if (twai_driver_install(&general, &timing, &filter) != ESP_OK) return false;
        if (twai_start() != ESP_OK) {
            twai_driver_uninstall();
            return false;
        }
        running = true;
        return true;
    }

    void close() override {
        if (!running) return;
        twai_stop();
        twai_driver_uninstall();
        running = false;
    }

    bool transmit(const slcan::Frame& f) override {
        twai_message_t m = {};
        m.identifier = f.id;
        m.extd = f.ext;
        m.rtr = f.rtr;
        m.data_length_code = f.dlc;
        memcpy(m.data, f.data, sizeof(m.data));
        if (twai_transmit(&m, pdMS_TO_TICKS(2)) == ESP_OK) return true;
        tx_failed = true;
        if (dropped < 255) ++dropped;
        return false;
    }

    uint8_t status_flags() override {
        twai_status_info_t s;
        if (!running || twai_get_status_info(&s) != ESP_OK) return 0;
        uint8_t flags = 0;
        if (s.msgs_to_rx >= kRxQueueLen) flags |= 0x01;
        if (s.msgs_to_tx >= kTxQueueLen) flags |= 0x02;
        if (s.tx_error_counter >= 96 || s.rx_error_counter >= 96) flags |= 0x04;
        if (s.rx_missed_count > 0) flags |= 0x08;
        if (s.tx_error_counter >= 128 || s.rx_error_counter >= 128) flags |= 0x20;
        if (s.arb_lost_count > 0) flags |= 0x40;
        if (s.state == TWAI_STATE_BUS_OFF) flags |= 0x80;
        return flags;
    }

    bool running = false;
    bool tx_failed = false;
    uint8_t dropped = 0;
};

UsbOutput usb;
TwaiPort can_port;
slcan::Parser parser(can_port, usb);

bool estop_pressed = true;           // assume pressed until the input has been read
int estop_raw_last = -1;
uint32_t estop_changed_ms = 0;
uint32_t last_status_ms = 0;
uint32_t last_rx_missed = 0;
uint32_t last_encoder_ms = 0;
uint8_t encoder_seq = 0;
const mt6701::ArduinoSsi encoder{PIN_ENC_CSN, PIN_ENC_CLK, PIN_ENC_DO};   // read_word(): a ~50 us read

void set_led(uint8_t r, uint8_t g, uint8_t b) {
#if PIN_RGB_LED >= 0
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    rgbLedWrite(PIN_RGB_LED, r, g, b);
#else
    neopixelWrite(PIN_RGB_LED, r, g, b);
#endif
#endif
}

void update_estop(uint32_t now) {
    const int raw = digitalRead(PIN_ESTOP);
    if (raw != estop_raw_last) {
        estop_raw_last = raw;
        estop_changed_ms = now;
    }
    const bool pressed = (raw == ESTOP_PRESSED_LEVEL);
    if (pressed != estop_pressed && now - estop_changed_ms >= ESTOP_DEBOUNCE_MS) {
        estop_pressed = pressed;
        last_status_ms = 0;          // report an E-stop change immediately
    }
}

void send_status(uint32_t now) {
    slcan::BridgeStatus st;
    st.estop_pressed = estop_pressed;
    twai_status_info_t s;
    if (can_port.running && twai_get_status_info(&s) == ESP_OK) {
        st.bus_off = (s.state == TWAI_STATE_BUS_OFF || s.state == TWAI_STATE_RECOVERING);
        st.error_passive = (s.tx_error_counter >= 128 || s.rx_error_counter >= 128);
        st.tx_error_counter = s.tx_error_counter > 255 ? 255 : s.tx_error_counter;
        st.rx_error_counter = s.rx_error_counter > 255 ? 255 : s.rx_error_counter;
        st.rx_overflow = (s.rx_missed_count > last_rx_missed);   // counter restarts at 0 on reopen
        last_rx_missed = s.rx_missed_count;
    }
    st.tx_failed = can_port.tx_failed;
    const uint32_t dropped = can_port.dropped + usb.usb_dropped;   // CAN sends + USB lines lost
    st.dropped = dropped > 255 ? 255 : static_cast<uint8_t>(dropped);
    st.uptime_s = static_cast<uint16_t>(now / 1000);
    can_port.tx_failed = false;
    can_port.dropped = 0;
    usb.usb_dropped = 0;
    parser.report_frame(slcan::encode_status(st), static_cast<uint16_t>(now % 60000));
}

void service_bus_off() {
    twai_status_info_t s;
    if (!can_port.running || twai_get_status_info(&s) != ESP_OK) return;
    if (s.state == TWAI_STATE_BUS_OFF) twai_initiate_recovery();
    else if (s.state == TWAI_STATE_STOPPED) twai_start();   // recovery finished
}

}  // namespace

void setup() {
    Serial.begin(115200);            // baud is ignored for native USB CDC
    Serial.setTxTimeoutMs(0);        // never wait for a PC that isn't reading (see UsbOutput)
    pinMode(PIN_ESTOP, INPUT_PULLUP);
    if (BENCH_ENCODER_JOINT > 0) encoder.begin();
}

void loop() {
    const uint32_t now = millis();
    bool busy = false;               // did this pass move any data?

    while (Serial.available() > 0) {
        parser.feed(static_cast<char>(Serial.read()));
        busy = true;
    }

    if (parser.is_open() && can_port.running) {
        twai_message_t m;
        while (twai_receive(&m, 0) == ESP_OK) {
            busy = true;
            slcan::Frame f;
            f.id = m.identifier;
            f.ext = m.extd;
            f.rtr = m.rtr;
            f.dlc = m.data_length_code > 8 ? 8 : m.data_length_code;
            memcpy(f.data, m.data, sizeof(f.data));
            parser.report_frame(f, static_cast<uint16_t>(now % 60000));
        }
        service_bus_off();
    }

    update_estop(now);
    if (parser.is_open() && now - last_status_ms >= STATUS_PERIOD_MS) {
        send_status(now);
        last_status_ms = now;
    }
    if (BENCH_ENCODER_JOINT > 0 && parser.is_open() && now - last_encoder_ms >= ENCODER_PERIOD_MS) {
        const mt6701::Reading r = mt6701::decode(mt6701::read_word(encoder));
        parser.report_frame(mt6701::angle_frame(BENCH_ENCODER_JOINT, r, encoder_seq++),
                            static_cast<uint16_t>(now % 60000));
        last_encoder_ms = now;
    }

    // red: E-stop pressed, green: bridging, blue: idle. Only touch the LED when the state changes.
    static int led_state = -1;
    const int state = estop_pressed ? 0 : (parser.is_open() ? 1 : 2);
    if (state != led_state) {
        led_state = state;
        if (state == 0) set_led(LED_BRIGHTNESS, 0, 0);
        else if (state == 1) set_led(0, LED_BRIGHTNESS, 0);
        else set_led(0, 0, LED_BRIGHTNESS);
    }

    // Nothing to do: sleep one tick (1 ms) so the idle task runs and the core isn't pinned at
    // 100 %. Under load the loop spins without sleeping, so no latency is added to traffic.
    if (!busy) delay(1);
}
