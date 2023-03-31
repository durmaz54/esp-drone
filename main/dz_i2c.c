#include "dz_i2c.h"

esp_err_t dz_i2c_init()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 25,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = 26,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 1000000,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };

    if (i2c_param_config(I2C_NUM, &conf) != ESP_OK)
    {
        return ESP_FAIL;
    }
    if (i2c_driver_install(I2C_NUM, conf.mode, 0, 0, 0) != ESP_OK)
    {
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t dz_i2c_read_reg(uint8_t reg_addr, uint8_t *reg_val, uint32_t len)
{
    esp_err_t rslt = ESP_OK; // 0 başarılı, -1 başarısız

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Start condition
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (reg_addr), ACK_CHECK_EN);

    // Restart (stop + start)
    i2c_master_start(cmd);

    // Address + read
    i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);

    // ACK_VAL olduğunda sonraki okuma adımlarını devam ettiriyor.
    // NACK_VAL olduğunda okuma işlemini tamamlıyor.
    if(len > 1)
    {
		i2c_master_read(cmd, reg_val, len - 1, I2C_MASTER_ACK);
    }
    	i2c_master_read_byte(cmd, reg_val + len - 1, I2C_MASTER_NACK);
	i2c_master_stop(cmd);

    if (i2c_master_cmd_begin(I2C_NUM, cmd, I2C_DELAY) != ESP_OK)
    {
        rslt = ESP_FAIL;
    }
    else
    {
        rslt = ESP_OK;
    }
    i2c_cmd_link_delete(cmd);
    return rslt;
}

esp_err_t dz_i2c_write_reg(uint8_t reg_addr, uint8_t *reg_val, uint32_t len)
{
    esp_err_t rslt = ESP_OK; // 0 başarılı, -1 başarısız
    uint8_t *byte_val = reg_val;

    for (int32_t i = 0; i < len; i++)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();

        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
        i2c_master_write(cmd, reg_val,len, ACK_CHECK_EN);
        i2c_master_stop(cmd);

        if (i2c_master_cmd_begin(I2C_NUM, cmd, I2C_DELAY) != ESP_OK)
        {
            rslt = ESP_FAIL;
            break;
        }
        else
        {
            rslt = ESP_OK;
        }
        i2c_cmd_link_delete(cmd);
    }

    return rslt;
}


