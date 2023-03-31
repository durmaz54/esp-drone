#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"

#include "dz_bmi.h"
#include "math.h"

void app_main(void)
{
    /*
    uint32_t time = 0;
    uint32_t dtime = 0;
    double anglex = 0;
    int16_t angley = 0;
    struct bmi_dev dev;*/

    dz_bmi_init();
    struct bmi_euler euler;
    euler.pitch = 0;
    euler.roll = 0;
    euler.yaw = 0;

    while (1)
    {
        //ESP_LOGW("-------------", "------------------------");
        if (dz_bmi_read_euler(&euler) != ESP_FAIL)
        {
            //time = esp_log_timestamp();
            //dtime = time - dtime;
            //anglex += (((double)dev.gyrox/32.8)) * ((double)dtime/1000.00);
            // ESP_LOGI("gyro", "%d ", dev.gyrox);
            //ESP_LOGI("euler_roll", "%.2f ", euler.roll);
            //ESP_LOGI("anglex", "%.2f ", anglex);
            //ESP_LOGI("yaw", "%.2f ", euler.yaw);
            //ESP_LOGI("roll", "%.2f ", euler.roll);
            // ESP_LOGI("angley", "%d",dev.gyroy);
            // ESP_LOGI("gyro", "x= %d y=%d z=%d", dev.gyrox, dev.gyroy, dev.gyroz);
            // ESP_LOGI("acc", "x= %d y=%d z=%d", dev.accx, dev.accy, dev.accz);
            // ESP_LOGI("sa","%d",(uint16_t)dtime);
        }
    }
}
