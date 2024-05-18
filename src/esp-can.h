#pragma once
#include <Arduino.h>
#include <driver/twai.h>

#define can_general_config_t twai_general_config_t
#define can_timing_config_t twai_timing_config_t
#define can_filter_config_t twai_filter_config_t
#define can_message_t twai_message_t

#define CAN_TIMING_CONFIG_1KBITS() TWAI_TIMING_CONFIG_1KBITS()
#define CAN_TIMING_CONFIG_5KBITS() TWAI_TIMING_CONFIG_5KBITS()
#define CAN_TIMING_CONFIG_10KBITS() TWAI_TIMING_CONFIG_10KBITS()
#define CAN_TIMING_CONFIG_12_5KBITS() TWAI_TIMING_CONFIG_12_5KBITS()
#define CAN_TIMING_CONFIG_16KBITS() TWAI_TIMING_CONFIG_16KBITS()
#define CAN_TIMING_CONFIG_20KBITS() TWAI_TIMING_CONFIG_20KBITS()
#define CAN_TIMING_CONFIG_25KBITS() TWAI_TIMING_CONFIG_25KBITS()
#define CAN_TIMING_CONFIG_50KBITS() TWAI_TIMING_CONFIG_50KBITS()
#define CAN_TIMING_CONFIG_100KBITS() TWAI_TIMING_CONFIG_100KBITS()
#define CAN_TIMING_CONFIG_125KBITS() TWAI_TIMING_CONFIG_125KBITS()
#define CAN_TIMING_CONFIG_250KBITS() TWAI_TIMING_CONFIG_250KBITS()
#define CAN_TIMING_CONFIG_500KBITS() TWAI_TIMING_CONFIG_500KBITS()
#define CAN_TIMING_CONFIG_800KBITS() TWAI_TIMING_CONFIG_800KBITS()
#define CAN_TIMING_CONFIG_1MBITS() TWAI_TIMING_CONFIG_1MBITS()

class Can {
    public:
        Can();
        esp_err_t begin(gpio_num_t tx, gpio_num_t rx, twai_timing_config_t t_config);
        esp_err_t send(can_message_t* message);
        void setReceiveCallback(std::function<void(can_message_t)> onReceive);
        void receive();
    private:
        std::function<void(can_message_t)> onReceive;
};