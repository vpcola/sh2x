#include "sht2x.h"


esp_err_t i2c_init(i2c_port_t port, gpio_num_t sda, gpio_num_t scl, uint32_t clk_speed) 
{
    esp_err_t ret;

    i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = scl,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = clk_speed,
    };

    i2c_param_config(port, &config);
    return i2c_driver_install(port, config.mode, 0, 0, 0);
}

void app_main() {
    float rh, temp;
    uint8_t reg;
    i2c_port_t port = I2C_NUM_0;
    printf("- Initialize I2C master\n");
    i2c_init(port, GPIO_NUM_14, GPIO_NUM_12, 100000);
    sht21_register(port, &reg);
    printf("Register = %X\n", reg);
    sht21_rh(port, &rh);
    printf("RH = %f\n", rh);
    sht21_temp(port, &temp);
    printf("Temp = %f\n", temp);

    while(true)
    {}
}



