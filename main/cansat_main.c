#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gptimer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "lora.h"
#include "sipm.h"
#include "ntc.h"
#include "sdcard.h"
#include "gps.h"
#include "mpu9250.h"

static const char* TAG = "cansat";

// not sure but should be good
struct data_struct {
    int ntcOut;
    int bmxOut;
    int16_t gyroOut[3];
    int16_t accelOut[3];
    gps_data_t gps_data;
    int sipmOut;
    int time;
};


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

void serialize_data(const struct data_struct *data, char *buffer, size_t buffer_size) {
    snprintf(buffer, buffer_size, "%d,%d,%d,%d,%d,%d,%d,%d,%f,%f,%f,%d,%d",
         data->ntcOut, data->bmxOut,
         data->gyroOut[0], data->gyroOut[1], data->gyroOut[2],
         data->accelOut[0], data->accelOut[1], data->accelOut[2],
         (float)data->gps_data.latitude, (float)data->gps_data.longitude, (float)data->gps_data.altitude,
         data->sipmOut, data->time);
}



void app_main(void)
{
    // testing
    /*
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_1,
        .scl_io_num = CONFIG_I2C_SCL,
        .sda_io_num = CONFIG_I2C_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

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
    
    return;*/

    // real main
    struct data_struct output;
    char serialized_data[256]; // Adjust the buffer size accordingly

    lora_info_t lora_info;
    lora_init();
    lora_get_info(&lora_info);

    ESP_LOGI(TAG, "Lora model_number=%d, version=%d, features=%d", lora_info.model, lora_info.version, lora_info.features);

    lora_set_address(0x1111);

    //const char message[5] = "ping";
    //char response[5];
    
    // timer init
    init_timer(xTaskGetCurrentTaskHandle());

    // sd card init
    sd_init();
    //sd_write("/sdcard/flight_data.txt", "Hello world!");

    // sipm init x
    sipm_init();

    // ntc init
    ntc_init();

    // bmx init, missing the files
    //bmx280_t *bmx = bmx280_create(bus_handle);
    //bmx280_init(bmx);
    //bmx280_configure(bmx, &BMX280_DEFAULT_CONFIG);
    //bmx280_setMode(bmx, BMX280_MODE_CYCLE);

    // mpu9250 init
    int16_t gyro_cal[3];
    mpu9250_init(gyro_cal);


    // gps init
    gps_init();

    while (1)
    {
        /* testing
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


        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);*/
        // read sipm
        output.sipmOut = sipm_read_count();

        // read time
        output.time = esp_timer_get_time();

        // read gps
        gps_get_data(&output.gps_data);

        // read gyro
        mpu9250_get_gyro(output.gyroOut);
        output.gyroOut[0] -= gyro_cal[0];
        output.gyroOut[1] -= gyro_cal[1];
        output.gyroOut[2] -= gyro_cal[2];

        // read accerelation
        mpu9250_get_accel(output.accelOut);

        // read NTC
        output.ntcOut = ntc_read();

        // read bmx, missing files
        //output.bmxOut = bmx280_read();
        output.bmxOut = 10;

        serialize_data(&output, serialized_data, sizeof(serialized_data));
        lora_transmit(serialized_data, sizeof(serialized_data)); // just sends the ping

        sd_write("/sdcard/flight_data.txt", serialized_data); // replace with actual data which was transmitted
        
        vTaskDelay(pdMS_TO_TICKS(500)); // just waits for 100ms, need to make this dynamic so it waits in a way that it loops 10 times a second
    }
}