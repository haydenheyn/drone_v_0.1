#include "i2c_master.hpp"

static const char* TAG = "I2CMaster";

bool I2CMaster::init(gpio_num_t sda_pin, gpio_num_t scl_pin, 
                     i2c_port_t port, uint32_t freq) {
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda_pin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = scl_pin;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = freq;
    conf.clk_flags = 0; // Use default I2C clock source

    esp_err_t ret = i2c_param_config(port, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(ret));
        return false;
    }

    ret = i2c_driver_install(port, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return false;
    }

    ESP_LOGI(TAG, "I2C initialized successfully");
    return true;
}
