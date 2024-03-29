#include <stdio.h>
#include <string.h>
#include "mpu9250.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/i2c_slave.h"

static const char* TAG = "mpu";

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;
i2c_slave_dev_handle_t slave_handle;

// todo
int16_t mpu9250_init(void){
    int16_t gyro_cal[3];
    
    // move these to main i2c controller ig
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = CONFIG_I2C_SCL,
        .sda_io_num = CONFIG_I2C_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    // config
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7, // todo
        .device_address = 0x58, // todo
        .scl_speed_hz = 100000,
    };
    
    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
    
    // i2c config mpu, idk prolly need another one of these
    ESP_LOGI(TAG, "Config for mpu i2c");
    i2c_slave_config_t i2c_slv_config = {
        .addr_bit_len = I2C_ADDR_BIT_LEN_7, // dunno
        .clk_source = I2C_CLK_SRC_DEFAULT, // dunno
        .i2c_port = I2C_NUM_0, // dunno
        .send_buf_depth = 256, // dunno either
        .scl_io_num = CONFIG_I2C_SCL,
        .sda_io_num = CONFIG_I2C_SDA,
        .slave_addr = 0x58, // dunno
    };

    i2c_slave_dev_handle_t slave_handle;
    ESP_ERROR_CHECK(i2c_new_slave_device(&i2c_slv_config, &slave_handle));
    // init the shit idk https://github.com/ricardozago/GY91-MPU9250-BMP280/blob/master/MPU9250/MPU9250.cpp#L274

    // calibrates the gyro based on 5 measurements
    ESP_LOGI(TAG, "Gyro calibration.");
    calibrate_gyro(gyro_cal, 5); 
    ESP_LOGI(TAG, "Gyro calibrated.");

    return gyro_cal;
}

// todo
void mpu9250_get_accel(int16_t accel[3]){
    uint8_t buffer[6];
    ESP_LOGI(TAG, "Reading acceleration.");

    i2c_master_receive(dev_handle, buffer, 100, -1); // IDK if this is correct at all but ight. -1 = forever

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }
}

// todo
void mpu9250_get_gyro(int16_t gyro[3]){
    uint8_t buffer[6];
    ESP_LOGI(TAG, "Reading gyro.");
    
    i2c_master_receive(dev_handle, buffer, 100, -1); // IDK if this is correct at all but ight. -1 = forever

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }
}

// this is done
void calibrate_gyro(int16_t gyroCal[3], int loop){
    int16_t temp[3];
    for (int i = 0; i < loop; i++)
    {
        mpu9250_get_gyro(temp);
        gyroCal[0] += temp[0];
        gyroCal[1] += temp[1];
        gyroCal[2] += temp[2];
    }
    gyroCal[0] /= loop;
    gyroCal[1] /= loop;
    gyroCal[2] /= loop;
}
