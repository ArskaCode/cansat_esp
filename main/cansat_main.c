#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gptimer.h"
#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "lora.h"
#include "sipm.h"
#include "ntc.h"


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
    ESP_ERROR_CHECK(gptimer_new_timer(&gptimer_config, &gptimer));

    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = 1 * 1000 * 1000,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_cb,
    };

    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, wake_task));
}


void app_main(void)
{
    lora_info_t lora_info;
    adc_oneshot_unit_handle_t adc2_handle;

    // ntc init
    ntc_init(&adc2_handle);
    // exsample call to ntc read
    double ntcVoltage = ntc_read(&adc2_handle);

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