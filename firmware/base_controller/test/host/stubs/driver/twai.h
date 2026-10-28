// Compile-check stub of the ESP-IDF TWAI (legacy) driver API used by src/main.cpp.
//
// Shapes copied from ESP-IDF v4.4.7 (the IDF inside Arduino-ESP32 2.0.x, which PlatformIO's
// `espressif32` platform ships): components/driver/include/driver/twai.h and
// components/hal/include/hal/twai_types.h. In particular the *_CONFIG_* macros are **brace
// initialisers with designated fields**, exactly like the real ones, so code that only compiles
// with a typed temporary (e.g. `*t = MACRO();` in C) is caught here. Not functional.
#pragma once
#include <cstdint>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_ERR_TIMEOUT 0x107
#define ESP_INTR_FLAG_LEVEL1 (1 << 1)
typedef int gpio_num_t;
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(x) (x)
#define TWAI_IO_UNUSED ((gpio_num_t) -1)
#define TWAI_ALERT_NONE 0
#define TWAI_FRAME_MAX_DLC 8
typedef enum { TWAI_MODE_NORMAL, TWAI_MODE_NO_ACK, TWAI_MODE_LISTEN_ONLY } twai_mode_t;
typedef enum { TWAI_STATE_STOPPED, TWAI_STATE_RUNNING, TWAI_STATE_BUS_OFF, TWAI_STATE_RECOVERING } twai_state_t;
typedef struct {
    twai_mode_t mode;
    gpio_num_t tx_io;
    gpio_num_t rx_io;
    gpio_num_t clkout_io;
    gpio_num_t bus_off_io;
    uint32_t tx_queue_len;
    uint32_t rx_queue_len;
    uint32_t alerts_enabled;
    uint32_t clkout_divider;
    int intr_flags;
} twai_general_config_t;
typedef struct {
    uint32_t brp;
    uint8_t tseg_1;
    uint8_t tseg_2;
    uint8_t sjw;
    bool triple_sampling;
} twai_timing_config_t;
typedef struct {
    uint32_t acceptance_code;
    uint32_t acceptance_mask;
    bool single_filter;
} twai_filter_config_t;
typedef struct {
    union {
        struct {
            uint32_t extd : 1;
            uint32_t rtr : 1;
            uint32_t ss : 1;
            uint32_t self : 1;
            uint32_t dlc_non_comp : 1;
            uint32_t reserved : 27;
        };
        uint32_t flags;
    };
    uint32_t identifier;
    uint8_t data_length_code;
    uint8_t data[TWAI_FRAME_MAX_DLC];
} twai_message_t;
typedef struct {
    twai_state_t state;
    uint32_t msgs_to_tx;
    uint32_t msgs_to_rx;
    uint32_t tx_error_counter;
    uint32_t rx_error_counter;
    uint32_t tx_failed_count;
    uint32_t rx_missed_count;
    uint32_t rx_overrun_count;
    uint32_t arb_lost_count;
    uint32_t bus_error_count;
} twai_status_info_t;
#define TWAI_GENERAL_CONFIG_DEFAULT(tx_io_num, rx_io_num, op_mode) {.mode = op_mode, .tx_io = tx_io_num, .rx_io = rx_io_num, \
                                                                    .clkout_io = TWAI_IO_UNUSED, .bus_off_io = TWAI_IO_UNUSED, \
                                                                    .tx_queue_len = 5, .rx_queue_len = 5, \
                                                                    .alerts_enabled = TWAI_ALERT_NONE, .clkout_divider = 0, \
                                                                    .intr_flags = ESP_INTR_FLAG_LEVEL1}
#define TWAI_FILTER_CONFIG_ACCEPT_ALL() {.acceptance_code = 0, .acceptance_mask = 0xFFFFFFFF, .single_filter = true}
// 10 and 20 kbit/s exist only on chips with a wide enough prescaler (the ESP32-S3 has them).
#define TWAI_TIMING_CONFIG_10KBITS() {.brp = 400, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
#define TWAI_TIMING_CONFIG_20KBITS() {.brp = 200, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
#define TWAI_TIMING_CONFIG_50KBITS() {.brp = 80, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
#define TWAI_TIMING_CONFIG_100KBITS() {.brp = 40, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
#define TWAI_TIMING_CONFIG_125KBITS() {.brp = 32, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
#define TWAI_TIMING_CONFIG_250KBITS() {.brp = 16, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
#define TWAI_TIMING_CONFIG_500KBITS() {.brp = 8, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
#define TWAI_TIMING_CONFIG_800KBITS() {.brp = 4, .tseg_1 = 16, .tseg_2 = 8, .sjw = 3, .triple_sampling = false}
#define TWAI_TIMING_CONFIG_1MBITS() {.brp = 4, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
inline esp_err_t twai_driver_install(const twai_general_config_t*, const twai_timing_config_t*, const twai_filter_config_t*) { return ESP_OK; }
inline esp_err_t twai_driver_uninstall(void) { return ESP_OK; }
inline esp_err_t twai_start(void) { return ESP_OK; }
inline esp_err_t twai_stop(void) { return ESP_OK; }
inline esp_err_t twai_transmit(const twai_message_t*, TickType_t) { return ESP_OK; }
inline esp_err_t twai_receive(twai_message_t*, TickType_t) { return ESP_ERR_TIMEOUT; }
inline esp_err_t twai_get_status_info(twai_status_info_t*) { return ESP_OK; }
inline esp_err_t twai_initiate_recovery(void) { return ESP_OK; }
