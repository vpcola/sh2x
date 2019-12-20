#ifndef __SHT21X_H_
#define __SHT21X_H_

#include <freertos/FreeRTOS.h>
#include <driver/gpio.h>
#include <driver/i2c.h>

// I2C slave address (7bit slave address)
#define SHT21_ADDR 0x40
// Register map
#define SHT21_CMD_TEMP_NO_HOLD        0xF3
#define SHT21_CMD_RH_NO_HOLD          0xF5
#define SHT21_CMD_WRITE_USER_REGISTER 0xE6
#define SHT21_CMD_READ_USER_REGISTER  0xE7
// I2C acknowledgement 
#define ACK_CHECK_EN    0x01 // I2C Master enable ack check from slave
#define ACK_CHECK_DIS   0x00 // I2C Master disable ack check from slave
#define ACK_VAL         0x00 // I2C Ack value 
#define NACK_VAL        0x01 // I2C nack value

esp_err_t sht21_readreg(i2c_port_t port, uint8_t *ans); 
esp_err_t sht21_humidity(i2c_port_t port, float * humid); 
esp_err_t sht21_temperature(i2c_port_t port, float * temp); 

#endif
