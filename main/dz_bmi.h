#ifndef INC_DZ_BMI_H_
#define INC_DZ_BMI_H_

#include "dz_i2c.h"
#include "esp_log.h"
#include "math.h"



#define rad_to_deg(a) ((double)a/(double)M_PI*(double)180)


#define BMI160_ADDR_CHIPID 0x00
#define BMI160_VALUE_CHIPID 0xD1
#define BMI160_ADDR_ERR_REG 0x02
#define BMI160_ADDR_PMU_STATUS 0x03

#define BMI160_ADDR_ACC_CONF 0x40
#define BMI160_ADDR_ACC_RANGE 0x41
#define BMI160_ADDR_GYR_CONF 0x42
#define BMI160_ADDR_GYR_RANGE 0x43

#define BMI160_ADDR_MAG_X_LSB 0x04
#define BMI160_ADDR_MAG_X_MSB 0x05 

#define BMI160_ADDR_GYR_X_LSB 0x0C
#define BMI160_ADDR_GYR_X_MSB 0x0D
#define BMI160_ADDR_GYR_Y_LSB 0x0E
#define BMI160_ADDR_GYR_Y_MSB 0x0F
#define BMI160_ADDR_GYR_Z_LSB 0x10
#define BMI160_ADDR_GYR_Z_MSB 0x11

#define BMI160_ADDR_ACC_X_LSB 0x12
#define BMI160_ADDR_ACC_X_MSB 0x13
#define BMI160_ADDR_ACC_Y_LSB 0x14
#define BMI160_ADDR_ACC_Y_MSB 0x15
#define BMI160_ADDR_ACC_Z_LSB 0x16
#define BMI160_ADDR_ACC_Z_MSB 0x17

#define BMI160_ADDR_STATUS 0x1B
#define BMI160_ADDR_STEP_CONF0 0x7A
#define BMI160_ADDR_STEP_CONF1 0x7B
#define BMI160_ADDR_CMD 0x7E

#define BMI160_ADDR_FOC_CONF 0x69
#define BMI160_ADDR_OFFSET_0 0x71
#define BMI160_ADDR_OFFSET_1 0x72
#define BMI160_ADDR_OFFSET_2 0x73
#define BMI160_ADDR_OFFSET_3 0x74
#define BMI160_ADDR_OFFSET_4 0x75
#define BMI160_ADDR_OFFSET_5 0x76
#define BMI160_ADDR_OFFSET_6 0x77




struct bmi_dev{
    int16_t gyrox,gyroy,gyroz;
    int16_t accx, accy,accz;
};

struct bmi_euler{
    double pitch,yaw,roll;
};

esp_err_t dz_bmi_init();

esp_err_t dz_bmi_read(struct bmi_dev *devx);

esp_err_t dz_bmi_read_euler(struct bmi_euler *euler, uint32_t deltaTime);

#endif