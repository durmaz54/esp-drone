#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"

#include "dz_bmi.h"
#include "math.h"
#include "dz_ibus.h"

static void fr_timer_callback(TimerHandle_t xTimer)
{
    ESP_LOGI("TIMER", "1s");
}

void app_main(void)
{
    dz_bmi_init();
    //dz_ibus_init();

    //freertos tickrate 1000 olarak ayarlandı. bu işlem sayesinde 100hzlik bir callback sağlayabildik.

    //pdTRUE parametresi ile timer'ın otomatik olarak tekrar başlatılmasını sağladık.
    TimerHandle_t fr_timer_handle = xTimerCreate("fr_timer", pdMS_TO_TICKS(1000), pdTRUE, (void*)0, fr_timer_callback);
    if(fr_timer_handle == NULL){
        ESP_LOGE("TIMER", "Timer creation failed");
    }
    else{
        ESP_LOGI("TIMER", "Timer created");
    }

    if(xTimerStart(fr_timer_handle, 0) != pdPASS){
        ESP_LOGE("TIMER", "Timer start failed");
    }
    else{
        ESP_LOGI("TIMER", "Timer started");
    }
    

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
