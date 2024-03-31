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
#include "ext/bmx280.h"
#include "mpu9250.h"

static const char* TAG = "cansat";

struct data_struct {
    int ntcOut;
    int bmxOut;
    int16_t gyroOut[3];
    int16_t accelOut[3];
    gps_data_t gps_data;
    int sipmOut;
    int time;
};

// Data binary serialization
// Utility function to write a 32-bit unsigned integer to buffer
void write_u32(uint8_t **ptr, uint32_t value) {
    *(uint32_t*)(*ptr) = value;
    (*ptr) += sizeof(uint32_t);
}

// Utility function to write a 16-bit signed integer to buffer
void write_s16(uint8_t **ptr, int16_t value) {
    *(int16_t*)(*ptr) = value;
    (*ptr) += sizeof(int16_t);
}

// Utility function to write a float to buffer
void write_float(uint8_t **ptr, float value) {
    *(float*)(*ptr) = value;
    (*ptr) += sizeof(float);
}

// Function to serialize data_struct into binary format
void serialize_data_struct(const struct data_struct *data, uint8_t *buffer) {
    // Serialize time
    *(int*)(buffer) = data->time;
    buffer += sizeof(int);

    // Serialize sipmOut
    *(int*)(buffer) = data->sipmOut;
    buffer += sizeof(int);

    // Serialize ntcOut
    *(int*)(buffer) = data->ntcOut;
    buffer += sizeof(int);

    // Serialize bmxOut
    *(int*)(buffer) = data->bmxOut;
    buffer += sizeof(int);

    // Serialize gyroOut
    for (int i = 0; i < 3; i++) {
        *(int16_t*)(buffer) = data->gyroOut[i];
        buffer += sizeof(int16_t);
    }

    // Serialize accelOut
    for (int i = 0; i < 3; i++) {
        *(int16_t*)(buffer) = data->accelOut[i];
        buffer += sizeof(int16_t);
    }

    // Serialize gps_data
    *(float*)(buffer) = data->gps_data.latitude;
    buffer += sizeof(float);
    *(float*)(buffer) = data->gps_data.longitude;
    buffer += sizeof(float);
    *(float*)(buffer) = data->gps_data.altitude;
} // end of data serialization

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
    snprintf(buffer, buffer_size, "1 %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f,%f,%f",
         data->time,data->sipmOut,
         data->ntcOut, data->bmxOut,
         data->gyroOut[0], data->gyroOut[1], data->gyroOut[2],
         data->accelOut[0], data->accelOut[1], data->accelOut[2],
         (float)data->gps_data.latitude, (float)data->gps_data.longitude, (float)data->gps_data.altitude);
}

void app_main(void)
{
    /*
    lora_info_t lora_info;
    lora_init();
    lora_get_info(&lora_info);
    ESP_LOGI(TAG, "Lora model_number=%d, version=%d, features=%d", lora_info.model, lora_info.version, lora_info.features);
    lora_set_address(0x1111);
    lora_set_channel(0x17);

    char msg[] = "hello";

    char msg[128];

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
    char serialized_data[256]; // Adjust the buffer size
    uint8_t binary_serialized_data[256]; // Adjust the buffer size

    // lora init
    lora_info_t lora_info;
    lora_init();
    lora_get_info(&lora_info);
    lora_set_address(0x1111);
    
    // timer init
    init_timer(xTaskGetCurrentTaskHandle());

    // sd card init
    sd_init();

    // sipm init
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

    // stuff for the 10/1s loop
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 100ms period

    // Initialize the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount(); // or should this be esp_timer_get_time?
    
    LORA_SEND_LOG(TAG, "Starting main loop.");
    while (1)
    {
        // read sipm
        output.sipmOut = sipm_read_count();

        // read time
        output.time = esp_timer_get_time();

        // read gps
        gps_get_data(&output.gps_data);

        // read gyro
        mpu9250_get_gyro(output.gyroOut);

        output.gyroOut[0] -= gyro_cal[0]; // eksdee this is def how you use the calibrated stuff
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

        lora_transmit(serialized_data, sizeof(serialized_data));
        sd_write("/sdcard/flight_data.txt", serialized_data);

        // binary stuff
        binary_serialize_data(&output, binary_serialized_data);
        lora_transmit(binary_serialized_data, sizeof(binary_serialized_data));

        vTaskDelayUntil(&xLastWakeTime, xFrequency); // idk might work
    }
}