#ifndef INC_DZ_I2C_H_
#define INC_DZ_I2C_H_

#include "driver/i2c.h"

#define SLAVE_ADDR 0x69
#define WRITE_BIT I2C_MASTER_WRITE
#define READ_BIT I2C_MASTER_READ
#define ACK_CHECK_EN 0x1
#define ACK_VAL 0x0 /*!< I2C ack value */
#define NACK_VAL 0x1
#define I2C_DELAY (10 / portTICK_PERIOD_MS)
#define I2C_NUM I2C_NUM_0

#define BME280_INIT_VALUE 0
#define SUCCESS 0
#define FAIL -1
typedef	int8_t s8;/**< used for signed 8bit */
typedef	int16_t s16;/**< used for signed 16bit */
typedef	int32_t s32;/**< used for signed 32bit */
typedef	int64_t s64;/**< used for signed 64bit */

typedef	u_int8_t u8;/**< used for unsigned 8bit */
typedef	u_int16_t u16;/**< used for unsigned 16bit */
typedef	u_int32_t u32;/**< used for unsigned 32bit */
typedef	u_int64_t u64;/**< used for unsigned 64bit */

esp_err_t dz_i2c_init();

esp_err_t dz_i2c_read_reg(uint8_t reg_addr, uint8_t *reg_val, uint32_t len);

esp_err_t dz_i2c_write_reg(uint8_t reg_addr, uint8_t *reg_val, uint32_t len);

#endif