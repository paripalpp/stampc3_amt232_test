#include "cui-amt23.h"


AMT23::AMT23(spi_host_device_t spi_host, gpio_num_t cs){
    auto dev_cfg = get_amt23_device_config(cs);
    err = spi_bus_add_device(spi_host, &dev_cfg, &device);
}

AMT23::cf AMT23::calc_data_checkbit(uint16_t data){
    bool k0, k1 = false;
    for(int i = 0; i < 7; i++){
        k0 ^= (data >> i*2) & 1;
        k1 ^= (data >> (i*2 + 1)) & 1;
    }
    return {k0,k1};
}

AMT23::data_t AMT23::read_spi(){
    spi_transaction_t transaction = {
        .flags = SPI_TRANS_USE_RXDATA,
        .length = 15,
    };
    esp_err_t err = spi_device_transmit(device, &transaction);
    ets_delay_us(1);
    AMT23::data_t data;
    data.rot = ((transaction.rx_data[0] & 0b00111111) << 8) | transaction.rx_data[1];
    data._cf.k0 = (transaction.rx_data[0] & 0b11000000) >> 6 & 1;
    data._cf.k1 = (transaction.rx_data[0] & 0b11000000) >> 7 & 1;
    return data;
}

bool AMT23::get(uint16_t* data){
    auto spi_data = read_spi();
    auto data_cf = calc_data_checkbit(spi_data.rot);
    // printf("rot : %d, data_cf : %d,%d, cf : %d,%d\n", spi_data.rot, data_cf.k0, data_cf.k1, spi_data._cf.k0, spi_data._cf.k1);
    if(data_cf == spi_data._cf){
        *data = spi_data.rot;
        return true;
    }else{
        *data = spi_data.rot;
        return false;
    }
}

bool AMT23::get_f(float* data){
    uint16_t data_int;
    auto ret = get(&data_int);
    if(ret){
        *data = (float)data_int / 16384.0f * 2.0f *M_PI;
        return ret;
    }else{
        *data = (float)data_int / 16384.0f * 2.0f *M_PI;
        return ret;
    }
}