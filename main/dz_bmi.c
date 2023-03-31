#include "dz_bmi.h"

struct bmi_dev dev;

static void bmi_delay()
{
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

static esp_err_t bmi_getPMu()
{
    uint8_t reg;
    if (dz_i2c_read_reg(BMI160_ADDR_PMU_STATUS, &reg, 1) != ESP_OK)
    {
        ESP_LOGW("I2C", "read ERROR");
        return ESP_FAIL;
    }
    if (reg == 0x14)
    {
        ESP_LOGI("bmiPMU", "acc, gyr are normal mode %hhx", reg);
    }
    else
    {
        ESP_LOGW("bmiPMU", "not Normal Mode %hhx", reg);
    }
    bmi_delay();
    return ESP_OK;
}

static esp_err_t bmi_setPmu()
{
    uint8_t reg;
    reg = 0x11;
    if (dz_i2c_write_reg(BMI160_ADDR_CMD, &reg, 1) != ESP_OK)
    {
        ESP_LOGW("I2C", "write ERROR");
        return ESP_FAIL;
    }
    bmi_delay();

    reg = 0x15;
    if (dz_i2c_write_reg(BMI160_ADDR_CMD, &reg, 1) != ESP_OK)
    {
        ESP_LOGW("I2C", "write ERROR");
        return ESP_FAIL;
    }
    bmi_delay();

    reg = 0x18;
    if (dz_i2c_write_reg(BMI160_ADDR_CMD, &reg, 1) != ESP_OK)
    {
        ESP_LOGW("I2C", "write ERROR");
        return ESP_FAIL;
    }
    bmi_delay();
    return ESP_OK;
}

static esp_err_t bmi_setAccGyro()
{
    uint8_t reg;
    reg = 0b00101100; // 1600odr
    if (dz_i2c_write_reg(BMI160_ADDR_ACC_CONF, &reg, 1) != ESP_OK)
    {
        ESP_LOGW("I2C", "write ERROR");
        return ESP_FAIL;
    }
    bmi_delay();

    reg = 0b00000011; // +-2g range
    if (dz_i2c_write_reg(BMI160_ADDR_ACC_RANGE, &reg, 1) != ESP_OK)
    {
        ESP_LOGW("I2C", "write ERROR");
        return ESP_FAIL;
    }
    bmi_delay();

    reg = 0b00101100; // 1600odr
    if (dz_i2c_write_reg(BMI160_ADDR_GYR_CONF, &reg, 1) != ESP_OK)
    {
        ESP_LOGW("I2C", "write ERROR");
        return ESP_FAIL;
    }
    bmi_delay();

    reg = 0b00000001; // +-1000 / s
    if (dz_i2c_write_reg(BMI160_ADDR_GYR_RANGE, &reg, 1) != ESP_OK)
    {
        ESP_LOGW("I2C", "write ERROR");
        return ESP_FAIL;
    }
    bmi_delay();

    return ESP_OK;
}

static esp_err_t bmi_setAccOffsetZero()
{
    uint8_t reg;
    reg = 0x00;
    if (dz_i2c_write_reg(BMI160_ADDR_OFFSET_3, &reg, 1) != ESP_OK)
    {
        ESP_LOGW("I2C", "write ERROR");
        return ESP_FAIL;
    }
    bmi_delay();

    if (dz_i2c_write_reg(BMI160_ADDR_OFFSET_4, &reg, 1) != ESP_OK)
    {
        ESP_LOGW("I2C", "write ERROR");
        return ESP_FAIL;
    }
    bmi_delay();

    if (dz_i2c_write_reg(BMI160_ADDR_OFFSET_5, &reg, 1) != ESP_OK)
    {
        ESP_LOGW("I2C", "write ERROR");
        return ESP_FAIL;
    }
    bmi_delay();


    return ESP_OK;
}

static esp_err_t bmi_getOffset()
{
    uint8_t reg;
    uint8_t data[6];
    if (dz_i2c_read_reg(BMI160_ADDR_OFFSET_0, &data, 6) != ESP_OK)
    {
        ESP_LOGW("I2C", "read ERROR");
        return ESP_FAIL;
    }
    bmi_delay();

    ESP_LOGI("offset acc", "x=%d y=%d z=%d", data[0], data[1], data[2]);
    ESP_LOGI("offset gyro", "x=%d y=%d z=%d", data[3], data[4], data[5]);

    return ESP_OK;
}

static esp_err_t bmi_setFOC()
{
    uint8_t reg;
    reg = 0b01010101;
    if (dz_i2c_write_reg(BMI160_ADDR_FOC_CONF, &reg, 1) != ESP_OK)
    {
        ESP_LOGW("I2C", "write ERROR");
        return ESP_FAIL;
    }
    bmi_delay();

    if (dz_i2c_read_reg(BMI160_ADDR_STATUS, &reg, 1) != ESP_OK)
    {
        ESP_LOGW("I2C", "read ERROR");
        return ESP_FAIL;
    }
    reg &= 0b00001000;
    ESP_LOGI("FOC RDY", "%d", reg);

    reg = 0x03; // START FOC
    if (dz_i2c_write_reg(BMI160_ADDR_CMD, &reg, 1) != ESP_OK)
    {
        ESP_LOGW("I2C", "write ERROR");
        return ESP_FAIL;
    }
    bmi_delay();
    bmi_delay(); // en fazla 250ms 'lik bekleme

    while (1)
    {
        if (dz_i2c_read_reg(BMI160_ADDR_STATUS, &reg, 1) != ESP_OK)
        {
            ESP_LOGW("I2C", "read ERROR");
            return ESP_FAIL;
        }
        reg &= 0b00001000;
        ESP_LOGI("FOC RDY", "%d", reg);
        if (reg == 8)
        {
            break;
        }
        bmi_delay();
    }
    // gyro off en and acc off en kontrol

    if (dz_i2c_read_reg(BMI160_ADDR_OFFSET_6, &reg, 1) != ESP_OK)
    {
        ESP_LOGW("I2C", "read ERROR");
        return ESP_FAIL;
    }

    ESP_LOGI("gyro-acc en", "%d", reg & 0b11000000);

    reg |= 0b11000000;
    if (dz_i2c_write_reg(BMI160_ADDR_OFFSET_6, &reg, 1) != ESP_OK)
    {
        ESP_LOGW("I2C", "write ERROR");
        return ESP_FAIL;
    }

    if (dz_i2c_read_reg(BMI160_ADDR_OFFSET_6, &reg, 1) != ESP_OK)
    {
        ESP_LOGW("I2C", "read ERROR");
        return ESP_FAIL;
    }

    ESP_LOGI("gyro-acc en", "%d", reg & 0b11000000);

    return ESP_OK;
}

static esp_err_t bmi_getErr()
{
    uint8_t reg;
    if (dz_i2c_read_reg(BMI160_ADDR_ERR_REG, &reg, 1) != ESP_OK)
    {
        ESP_LOGW("I2C", "read ERROR");
        return ESP_FAIL;
    }

    reg = reg & 0b00011110;
    if (reg != 0x00)
    {
        ESP_LOGW("BMI_ERR_REG", "%hhx", reg);
        return ESP_FAIL;
    }
    // ESP_LOGW("BMI_ERR", "%hhx", reg);

    return ESP_OK;
}

esp_err_t dz_bmi_init()
{
    dz_i2c_init();

    uint8_t reg;
    if (dz_i2c_read_reg(BMI160_ADDR_CHIPID, &reg, 1) != ESP_OK)
    {
        ESP_LOGW("I2C", "read ERROR");
        return ESP_FAIL;
    }
    bmi_delay();

    if (reg != BMI160_VALUE_CHIPID)
    {
        ESP_LOGW("BMI", "chip id ERROR");
        return ESP_FAIL;
    }
    bmi_delay();

    bmi_getErr();
    bmi_setPmu();
    bmi_getPMu();
    bmi_setPmu();
    bmi_setAccGyro();
    bmi_getOffset();
    bmi_setFOC();
    bmi_getOffset();

    ESP_LOGI("BMI", "init success");
    return ESP_OK;
}

esp_err_t dz_bmi_read(struct bmi_dev *devx)
{
    uint8_t data[12] = {0};
    uint8_t reg;

    if (dz_i2c_read_reg(BMI160_ADDR_STATUS, &reg, 1) != ESP_OK)
    {
        ESP_LOGW("I2C", "read ERROR");
        return ESP_FAIL;
    }
    // ESP_LOGI("stat", "%d", reg);
    reg = reg & 0b11010000; // acc gyro data read
    if (reg != 208)
    {
        return ESP_FAIL;
    }

    if (dz_i2c_read_reg(BMI160_ADDR_GYR_X_LSB, data, 12) != ESP_OK)
    {
        ESP_LOGW("I2C", "read ERROR");
        return ESP_FAIL;
    }
    devx->gyrox = (data[1] << 8) | data[0];
    devx->gyroy = (data[3] << 8) | data[2];
    devx->gyroz = (data[5] << 8) | data[4];

    devx->accx = (data[7] << 8) | data[6];
    devx->accy = (data[9] << 8) | data[8];
    devx->accz = (data[11] << 8) | data[10];

    bmi_getErr();

    return ESP_OK;
}

esp_err_t dz_bmi_read_euler(struct bmi_euler *euler, uint32_t deltaTime)
{
    if (dz_bmi_read(&dev) != ESP_OK)
    {
        return ESP_FAIL;
    }

    //roll = x
    double gyro_roll= (double)dev.gyrox / 32.8;
    double gyro_pitch  = (double)dev.gyroy / 32.8;
    double gyro_yaw= (double)dev.gyroz / 32.8;
    double dt = (double)deltaTime / 1000.00;
    /*
    euler->roll += gyro_roll * dt;
    euler->pitch += gyro_pitch * dt;
    euler->yaw += gyro_yaw * dt;*/
    
    double accx = (double)dev.accx / 16384.00;
    double accy = (double)dev.accy / 16384.00;
    double accz = (double)dev.accz / 16384.00;


    double omega_roll = atan2(accy,accz);
    double omega_pitch = atan2(-accx, sqrt(accy*accy + accz*accz));
    //ESP_LOGW("accl","x= %d, y= %d, z=%d", (int16_t)rad_to_deg(omega_roll), (int16_t)rad_to_deg(omega_pitch), dev.accz);
    //tamamlayacı filter
    double test = 1 * (euler->roll + gyro_roll*dt);
    double test2 = (rad_to_deg(omega_roll) * 1);
    //ESP_LOGW("roll","%d", (int16_t)test);
    //ESP_LOGW("rollacc","%d", (int16_t)test2);
    euler->roll = 0.93 * (euler->roll + gyro_roll*dt) + (rad_to_deg(omega_roll) * 0.07);
    euler->pitch = 0.93 * (euler->pitch + gyro_pitch*dt) + (rad_to_deg(omega_pitch) * 0.07);

    return ESP_OK;
}