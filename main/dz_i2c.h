#ifndef INC_DZ_I2C_H_
#define INC_DZ_I2C_H_

#include "driver/i2c.h"

#define SLAVE_ADDR 0x69
#define WRITE_BIT I2C_MASTER_WRITE
#define READ_BIT I2C_MASTER_READ
#define ACK_CHECK_EN 0x1
#define ACK_VAL 0x0 /*!< I2C ack value */
#define NACK_VAL 0x1
#define I2C_DELAY (1000 / portTICK_PERIOD_MS)
#define I2C_NUM I2C_NUM_0

esp_err_t dz_i2c_init();

esp_err_t dz_i2c_read_reg(uint8_t reg_addr, uint8_t *reg_val, uint32_t len);

esp_err_t dz_i2c_write_reg(uint8_t reg_addr, uint8_t *reg_val, uint32_t len);

#endif