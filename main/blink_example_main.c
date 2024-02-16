#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lora.h"
#include "sipm.h"


static const char* TAG = "satellite";


void app_main(void)
{
    lora_info_t lora_info;

    lora_init();
    lora_get_info(&lora_info);
    
    ESP_LOGI(TAG, "Lora model_number=%d, version=%d, features=%d", lora_info.model, lora_info.version, lora_info.features);

    lora_set_address(0x1111);

    const char message[5] = "ping";
    char response[5];

    lora_transmit(message, sizeof(message));
    lora_receive(response, sizeof(response));

    if (strcmp(response, "pong") != 0) {
        ESP_LOGI(TAG, "Didn't work :(");
    } else {
        ESP_LOGI(TAG, "Working");
    }
}