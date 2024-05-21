#include <Arduino.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <FastLED.h>

#include "esp-can.h"
#include "cui-amt23.h"
#include "PID.h"

#define SPI_HOST SPI2_HOST

struct amt23_data {
    uint16_t data;
    struct {
        bool k0;
        bool k1;
    } data_cf;
    struct {
        bool k0;
        bool k1;
    } cf;
    bool valid;
};

amt23_data read_amt23(uint8_t* data){
    amt23_data result;
    result.data = ((data[0] & 0b00111111) << 8) | data[1];
    result.cf.k0 = (data[0] & 0b11000000) >> 6 & 1;
    result.cf.k1 = (data[0] & 0b11000000) >> 7 & 1;
    //check bit
    bool k0, k1 = false;
    for(int i = 0; i < 7; i++){
        k0 ^= (result.data >> i*2) & 1;
        k1 ^= (result.data >> (i*2 + 1)) & 1;
    }
    result.valid = !k0 == result.cf.k0 && !k1 == result.cf.k1;
    result.data_cf.k0 = !k0;
    result.data_cf.k1 = !k1;
    return result;
}

void setup() {
    // led setup
    CRGB leds[1];
    FastLED.addLeds<SK6812, 2, GRB>(leds, 1);
    leds[0] = CRGB::Red;
    FastLED.show();

    pinMode(GPIO_NUM_3, INPUT_PULLUP);

    // spi setup
    auto buscfg = get_amt23_bus_config(GPIO_NUM_6, GPIO_NUM_7);
    esp_err_t ret = spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_DISABLED);
    gpio_set_pull_mode(GPIO_NUM_6, GPIO_PULLDOWN_ONLY);
    AMT23 encoder[] = {
        AMT23(SPI_HOST, GPIO_NUM_8),
        AMT23(SPI_HOST, GPIO_NUM_10),
        AMT23(SPI_HOST, GPIO_NUM_1),
        AMT23(SPI_HOST, GPIO_NUM_0)
    };

    Can can = Can();
    can.begin(GPIO_NUM_4, GPIO_NUM_5, CAN_TIMING_CONFIG_1MBITS());
    can.setReceiveCallback([&](can_message_t msg){});

    const uint64_t zero64 = 0;
    can_message_t message_dcmd0;
    memcpy(&message_dcmd0.data, &zero64, 8);
    message_dcmd0.identifier = 0x10;
    message_dcmd0.flags = TWAI_MSG_FLAG_NONE;
    message_dcmd0.data_length_code = 4;
    can_message_t message_dcmd1;
    memcpy(&message_dcmd1.data, &zero64, 8);
    message_dcmd1.identifier = 0x11;
    message_dcmd1.flags = TWAI_MSG_FLAG_NONE;
    message_dcmd1.data_length_code = 4;

    // float target_rot = 0;
    // float rotation = 0;
    // float motor = 0;
    float target[] = {0, 0, 0, 0};
    float rotation[] = {0, 0, 0, 0};
    float motor[] = {0, 0, 0, 0};

    // PID motor_pid(0.0, 0.3, 0.0, 1.0f, 0.0f, 0.0f, true, 0.0f, 2*PI);
    PID motor_pid[] = {
        PID(0.0, 0.3, 0.0, 1.0f, 0.0f, 0.0f, true, 0.0f, 2*PI),
        PID(0.0, 0.3, 0.0, 1.0f, 0.0f, 0.0f, true, 0.0f, 2*PI),
        PID(0.0, 0.3, 0.0, 1.0f, 0.0f, 0.0f, true, 0.0f, 2*PI),
        PID(0.0, 0.3, 0.0, 1.0f, 0.0f, 0.0f, true, 0.0f, 2*PI)
    };

    while (true)
    {
        delay(1);
        if(digitalRead(GPIO_NUM_3) == LOW){
            target[0] = - PI / 2;
            target[1] = PI / 2;
            target[2] = PI / 2;
            target[3] = - PI / 2;
        }else{
            target[0] = 0;
            target[1] = 0;
            target[2] = 0;
            target[3] = 0;
        }
        int16_t motor_int[4];
        for(size_t i = 0; i < 4; i++){
            encoder[i].get_f(&rotation[i]);
            motor[i] = motor_pid[i].process(rotation[i], target[i]);
            motor[i] = (motor[i] > 0.6) ? 0.6 : (motor[i] < -0.6) ? -0.6 : motor[i];

            motor_int[i] = motor[i] * INT16_MAX;
        }
        printf("r0: %f, r1: %f, r2: %f, r3: %f\n", rotation[0], rotation[1], rotation[2], rotation[3]);
        // printf("t0: %f, t1: %f, t2: %f, t3: %f\n", target[0], target[1], target[2], target[3]);
        // printf("m0: %f, m1: %f, m2: %f, m3: %f\n", motor[0], motor[1], motor[2], motor[3]);
        memcpy(message_dcmd0.data, &motor_int[0], 4);
        memcpy(message_dcmd1.data, &motor_int[2], 4);
        can.send(&message_dcmd0);
        can.send(&message_dcmd1);
    }
  
}

void loop() {}
