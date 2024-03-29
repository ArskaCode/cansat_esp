#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gptimer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "lora.h"
#include "sipm.h"
#include "ntc.h"
#include "sdcard.h"
#include "gps.h"
#include "ext/bmx280.h"


static const char* TAG = "cansat";


static bool IRAM_ATTR timer_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    TaskHandle_t wake_task = (TaskHandle_t) user_ctx;
    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(wake_task, &higher_priority_task_woken);
    return higher_priority_task_woken == pdTRUE;
}


static void init_timer(TaskHandle_t wake_task)
{
    gptimer_handle_t gptimer;
    gptimer_config_t gptimer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000 * 1000,
    };
    LORA_SEND_ERROR(TAG, gptimer_new_timer(&gptimer_config, &gptimer));

    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = 1 * 1000 * 1000,
        .flags.auto_reload_on_alarm = true,
    };
    LORA_SEND_ERROR(TAG, gptimer_set_alarm_action(gptimer, &alarm_config));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_cb,
    };

    LORA_SEND_ERROR(TAG, gptimer_register_event_callbacks(gptimer, &cbs, wake_task));
}


void app_main(void)
{
    lora_info_t lora_info;
    lora_init();
    lora_get_info(&lora_info);
    ESP_LOGI(TAG, "Lora model_number=%d, version=%d, features=%d", lora_info.model, lora_info.version, lora_info.features);
    lora_set_address(0x1111);
    lora_set_channel(0x17);

    char msg[128] = "hello";

    //char msg[128];

    while (1)
    {
        ESP_LOGI(TAG, "Sent hello");
        lora_transmit(msg, sizeof(msg));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    return;
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_1,
        .scl_io_num = CONFIG_I2C_SCL,
        .sda_io_num = CONFIG_I2C_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    LORA_SEND_ERROR(TAG, i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    float temp, pressure, humidity;

    bmx280_t *bmx = bmx280_create(bus_handle);
    bmx280_init(bmx);
    bmx280_configure(bmx, &BMX280_DEFAULT_CONFIG);
    bmx280_setMode(bmx, BMX280_MODE_CYCLE);
    //bmx280_setMode(bmx, BMX280_MODE_CYCLE);
    while (1) {
        bmx280_readoutFloat(bmx, &temp, &pressure, &humidity);

        ESP_LOGI(TAG, "T = %f , P = %f , Rh = %f, idk = %d", temp, pressure, humidity, bmx280_isSampling(bmx));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    return;

    // sd card init
    sd_init();
    sd_write("/sdcard/flight_data.txt", "Hello world!");

    // ntc init
    ntc_init();
    // exsample call to ntc read
    int ntcVoltage = ntc_read();

    lora_init();
    lora_get_info(&lora_info);

    ESP_LOGI(TAG, "Lora model_number=%d, version=%d, features=%d", lora_info.model, lora_info.version, lora_info.features);

    lora_set_address(0x1111);

    const char message[5] = "ping";
    char response[5];

    init_timer(xTaskGetCurrentTaskHandle());
    while (1)
    {
        lora_transmit(message, sizeof(message));
        lora_receive(response, sizeof(response));

        if (strcmp(response, "pong") != 0)
        {
            ESP_LOGI(TAG, "Didn't work :(");
        }
        else
        {
            ESP_LOGI(TAG, "Working");
        }


        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}