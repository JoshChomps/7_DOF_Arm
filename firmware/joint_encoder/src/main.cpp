// deskarm joint encoder node (board C, ESP32-C3).
//
// * Reads the joint's MT6701 over SSI every 5 ms and puts the joint-angle frame on the arm's CAN bus
//   (ID 0x10 + JOINT_ID, 4 bytes: 14-bit angle big-endian, flags, sequence; D34). The host does the
//   coarse + fine sum (docs/16 sec. 3.3); this board only reports the raw angle.
// * The SSI read, the decoder and the frame are the base controller's, shared, not copied: they live in
//   firmware/base_controller/src/mt6701 and are built in as a library (README.md in this folder).
// * Recovers from CAN bus-off. Never waits on the bus: a frame that finds the TX queue full is dropped.
// * Status LED: on = sensor and bus fine, slow blink = sensor problem, fast blink = CAN problem.
//   A line on the USB serial port every 500 ms says the same in words.
#include <Arduino.h>

#include <cstdio>

#include "config.h"
#include "driver/twai.h"
#include "mt6701_arduino.h"

static_assert(JOINT_ID >= 1 && JOINT_ID <= 7, "JOINT_ID is the joint's MKS driver id, 1..7");

#if CAN_BITRATE == 125000
#define CAN_TIMING TWAI_TIMING_CONFIG_125KBITS
#elif CAN_BITRATE == 250000
#define CAN_TIMING TWAI_TIMING_CONFIG_250KBITS
#elif CAN_BITRATE == 500000
#define CAN_TIMING TWAI_TIMING_CONFIG_500KBITS
#elif CAN_BITRATE == 1000000
#define CAN_TIMING TWAI_TIMING_CONFIG_1MBITS
#else
#error "CAN_BITRATE must be one the MKS drivers support: 125000, 250000, 500000 or 1000000"
#endif

namespace {

bool can_running = false;
uint32_t dropped = 0;                // frames that found the TX queue full (no ACK: nothing else on the bus)
uint32_t last_encoder_ms = 0;
uint32_t last_report_ms = 0;
uint8_t seq = 0;
uint8_t flags = 0;                   // byte 2 of the last frame sent
mt6701::Reading reading;

const mt6701::ArduinoSsi encoder{PIN_ENC_CSN, PIN_ENC_CLK, PIN_ENC_DO};   // the board has 10 k pull-ups on CSN, DO

void start_can() {
    const twai_timing_config_t timing = CAN_TIMING();
    const twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(
        static_cast<gpio_num_t>(PIN_CAN_TX), static_cast<gpio_num_t>(PIN_CAN_RX), TWAI_MODE_NORMAL);
    const twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&general, &timing, &filter) != ESP_OK) return;
    can_running = twai_start() == ESP_OK;
}

void send_angle(uint32_t now) {
    reading = mt6701::decode(mt6701::read_word(encoder));
    const slcan::Frame f = mt6701::angle_frame(JOINT_ID, reading, seq++);
    flags = f.data[2];
    twai_message_t m = {};
    m.identifier = f.id;
    m.data_length_code = f.dlc;
    memcpy(m.data, f.data, sizeof(m.data));
    if (!can_running || twai_transmit(&m, 0) != ESP_OK) ++dropped;
    last_encoder_ms = now;
}

// Bus-off: start recovery; once it has finished the driver is stopped, so start it again. Returns
// whether the bus is healthy (running and not error-passive).
bool service_can(twai_status_info_t* s) {
    if (!can_running || twai_get_status_info(s) != ESP_OK) return false;
    if (s->state == TWAI_STATE_BUS_OFF) twai_initiate_recovery();
    else if (s->state == TWAI_STATE_STOPPED) twai_start();
    twai_message_t m;
    while (twai_receive(&m, 0) == ESP_OK) {}       // nothing to listen to yet; keep the RX queue empty
    return s->state == TWAI_STATE_RUNNING && s->tx_error_counter < 128 && s->rx_error_counter < 128;
}

bool sensor_ok() {
    return reading.present && reading.crc_ok && (reading.status & 0x03) == 0 && (reading.status & 0x08) == 0;
}

void report(uint32_t now, const twai_status_info_t& s, bool can_ok) {
    char line[112];
    const int n = snprintf(line, sizeof(line),
                           "J%d angle %u (%lu.%02lu deg) flags 0x%02x seq %u | CAN %s tec %lu rec %lu dropped %lu\r\n",
                           JOINT_ID, static_cast<unsigned>(reading.angle),
                           static_cast<unsigned long>(reading.angle * 36000ul / 16384 / 100),
                           static_cast<unsigned long>(reading.angle * 36000ul / 16384 % 100),
                           static_cast<unsigned>(flags), static_cast<unsigned>(seq),
                           can_ok ? "ok" : "fault", static_cast<unsigned long>(can_running ? s.tx_error_counter : 0),
                           static_cast<unsigned long>(can_running ? s.rx_error_counter : 0),
                           static_cast<unsigned long>(dropped));
    if (n > 0 && Serial.availableForWrite() >= n) Serial.write(reinterpret_cast<const uint8_t*>(line), n);
    last_report_ms = now;
}

}  // namespace

void setup() {
    Serial.begin(115200);            // baud is ignored for USB Serial/JTAG
    Serial.setTxTimeoutMs(0);        // never wait for a PC that isn't reading
    pinMode(PIN_LED, OUTPUT);
    encoder.begin();
    start_can();
}

void loop() {
    const uint32_t now = millis();
    if (now - last_encoder_ms >= ENCODER_PERIOD_MS) send_angle(now);

    twai_status_info_t s = {};
    const bool can_ok = service_can(&s);
    if (now - last_report_ms >= REPORT_PERIOD_MS) report(now, s, can_ok);

    // on: all fine; 1 Hz blink: sensor absent, CRC bad, field too strong/weak or track lost; 5 Hz: CAN fault.
    const bool lit = !sensor_ok() ? (now / 500) % 2 == 0 : !can_ok ? (now / 100) % 2 == 0 : true;
    digitalWrite(PIN_LED, lit ? LED_ON : !LED_ON);

    delay(1);                        // let the idle task run; 1 ms against a 5 ms period
}
