
#ifndef I2C_MASTER_HPP
#define I2C_MASTER_HPP

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"

class I2CMaster {
public:
    static bool init(gpio_num_t sda_pin, gpio_num_t scl_pin, 
                     i2c_port_t port = I2C_NUM_0, uint32_t freq = 100000);
};

#endif // I2C_MASTER_HPP
