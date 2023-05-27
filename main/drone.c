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
    dz_bmi_init();
    struct bmi_euler euler;
    euler.pitch = 0;
    euler.roll = 0;
    euler.yaw = 0;
    uint32_t time = 0;
    uint32_t dtime = 0;

    while (1)
    {
        dtime = time;
        time = esp_log_timestamp();
        dtime = time - dtime;
        if(dtime >= 1){
        dz_bmi_read_euler(&euler, dtime);
        ESP_LOGI("EULER", "%.2f ",euler.pitch);
        }
        }
}
