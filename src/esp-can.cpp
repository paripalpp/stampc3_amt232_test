#include "esp-can.h"

Can::Can(){
    this->onReceive = [](can_message_t msg){};
}

esp_err_t Can::begin(gpio_num_t tx, gpio_num_t rx, twai_timing_config_t t_config) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, TWAI_MODE_NORMAL);
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if(err){
        return err;
    }
    return twai_start();
}

esp_err_t Can::send(can_message_t* message) {
    return twai_transmit(message, pdMS_TO_TICKS(0));
}

void Can::setReceiveCallback(std::function<void(can_message_t)> onReceive){
    this->onReceive = onReceive;
}

void Can::receive() {
    twai_status_info_t status_info;
    twai_get_status_info(&status_info);
    for(size_t i = 0; i < status_info.msgs_to_rx; i++){
        twai_message_t message;
        twai_receive(&message, pdMS_TO_TICKS(0));
        this->onReceive(message);
    }
}