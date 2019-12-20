#include "sht21x.h"

// Reads a byte from the user register
esp_err_t sht21_readreg(i2c_port_t port, uint8_t *ans) 
{
  esp_err_t rc; 

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  // Write the 7 bit address followed by the read write flag, enable ack check
  i2c_master_write_byte(cmd, (SHT21_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
  // Write the register value  
  i2c_master_write_byte(cmd, SHT21_CMD_READ_USER_REGISTER, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  // Finish the I2C transaction (by sending the command out)
  rc = i2c_master_cmd_begin(port, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  if (rc  != ESP_OK) return rc;

  // Now switch to read mode, and read the data from the device based
  // on the register we sent earlier
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  // Write the address and the read flag, with ack
  i2c_master_write_byte(cmd, (SHT21_ADDR << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
  // Read 1 byte and put it in ans
  i2c_master_read_byte(cmd, ans, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);

  rc = i2c_master_cmd_begin(port, cmd, 1000 / portTICK_RATE_MS) ;
  i2c_cmd_link_delete(cmd);

  return rc;
}

// Sends a command to the device and receive 3 bytes.
static esp_err_t sht21_cmd_bytes(i2c_port_t port, uint8_t reg, uint8_t *buff) 
{
  esp_err_t rc;  

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);

  i2c_master_write_byte(cmd, (SHT21_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);

  i2c_master_stop(cmd);
  rc = i2c_master_cmd_begin(port, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  if (rc != ESP_OK) return rc;
  // After sending the target register, now read 3 bytes from the device
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (SHT21_ADDR << 1) | I2C_MASTER_READ, ACK_CHECK_EN );
  // First read the first 2 bytes
  i2c_master_read(cmd, buff, 2, I2C_MASTER_ACK);
  // Read the last byte (with NACK value)
  i2c_master_read_byte(cmd, buff+2, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);
  // Finalize the transaction
  rc = i2c_master_cmd_begin(port, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  return rc;
}


esp_err_t sht21_humidity(i2c_port_t port, float *ans) 
{
  uint8_t buff[3];

  esp_err_t ret = sht21_cmd_bytes(port, SHT21_CMD_RH_NO_HOLD, buff);
  if (ret != ESP_OK) return ret;

  uint16_t val = (buff[0] << 8) | (buff[1] && 0xFC);
  *ans = -6 + 125 * (val / 65536.0);

  return ESP_OK;
}


esp_err_t sht21_temperature(i2c_port_t port, float *ans) 
{
  uint8_t buff[3];
  esp_err_t ret = sht21_cmd_bytes(port, SHT21_CMD_TEMP_NO_HOLD, buff);
  if (ret != ESP_OK) return ret;

  uint16_t val = (buff[0] << 8) | (buff[1] && 0xFC);
  *ans = -46.25 + 175.72 * (val / 65536.0);

  return ESP_OK;
}



