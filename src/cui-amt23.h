#pragma once
#include <math.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"

constexpr spi_bus_config_t get_amt23_bus_config(const gpio_num_t miso, const gpio_num_t sclk){
    return spi_bus_config_t{
        .mosi_io_num = GPIO_NUM_NC,
        .miso_io_num = miso,
        .sclk_io_num = sclk,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = 0,
    };
}
constexpr spi_device_interface_config_t get_amt23_device_config(const gpio_num_t cs){
    return spi_device_interface_config_t{
        .dummy_bits = 1,
        .mode = 2,
        .cs_ena_pretrans = 1,
        .cs_ena_posttrans = 1,
        .clock_speed_hz = 1000000,
        .spics_io_num = cs,
        .queue_size = 1
    };
}

class AMT23
{
private:
    struct cf{
        bool k0; bool k1;
        bool operator==(cf other){return k0 == other.k0 && k1 == other.k1;};
    };
    struct data_t{uint16_t rot; cf _cf;};
    spi_device_handle_t device;
    esp_err_t err;
    cf calc_data_checkbit(uint16_t data);
    data_t read_spi();
public:
    AMT23(spi_host_device_t spi_host, gpio_num_t cs);

    // @brief get encoder value by spi
    // @param data pointer to return value. the value is 14 bit 0~16383
    // @return is data valid
    bool get(uint16_t* data);

    // @brief get encoder value by spi
    // @param data pointer to return value. the value is in radian 0~2*PI
    // @return is data valid
    bool get_f(float* data);
};
