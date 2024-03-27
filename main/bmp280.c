/**
 * BMP280 - BMP280 Driver for Esspressif ESP-32.
 *
 * MIT License
 *
 * Copyright (C) 2020 Halit Utku Maden
 * Please contact at <utkumaden@hotmail.com>
 * Mostly just stolen from https://github.com/utkumaden/esp-idf-bmp280
 */
#include "bmp280.h"
#include "esp_log.h"

#include <stdlib.h>
#include <stdint.h>
#include <string.h>


static const char* TAG = "cansat_bmp280";

// TODO none of these are setup correctly 

// Register address of pressure fraction significant byte.
#define BMP280_REG_PRES_XSB 0xF9
// Register address of pressure least significant byte.
#define BMP280_REG_PRES_LSB 0xF8
// Register address of pressure most significant byte.
#define BMP280_REG_PRES_MSB 0xF7

// Register address of sensor configuration.
#define BMP280_REG_CONFIG 0xF5
// Register address of sensor measurement control.
#define BMP280_REG_MESCTL 0xF4
// Register address of sensor status.
#define BMP280_REG_STATUS 0xF3

// Register address for sensor reset.
#define BMP280_REG_RESET 0xE0
// Chip reset vector.
#define BMP280_RESET_VEC 0xB6

// [BME280] Register address of calibration constants. (high bank)
#define BMP280_REG_CAL_HI 0xE1
// Register address of calibration constants. (low bank)
#define BMP280_REG_CAL_LO 0x88

// Register address for chip identification number.
#define BMP280_REG_CHPID 0xD0
// Value of REG_CHPID for BMP280 (Production)
#define BMP280_ID2 0x58

struct bmp280_t{
    // I2C port.
    i2c_port_t i2c_port;
    // Slave Address of sensor.
    uint8_t slave;
    // Chip ID of sensor
    uint8_t chip_id;
    // Compensation data
    struct {
        uint16_t T1;
        int16_t T2;
        int16_t T3;
        uint16_t P1;
        int16_t P2;
        int16_t P3;
        int16_t P4;
        int16_t P5;
        int16_t P6;
        int16_t P7;
        int16_t P8;
        int16_t P9;
    } cmps;
    // Storage for a variable proportional to temperature.
    int32_t t_fine;
};

/**
 * Macro to verify a the chip id matches with the expected value for BMP280.
 * @note Use when the chip needs to be verified as a BMP280.
 * @see bmp280_verify
 * @param chip_id The chip id.
 */
#define bmp280_verify(chip_id) ((chip_id) == BMP280_ID2)

/**
 * Returns false if the sensor was not found.
 * @param bmp280 The driver structure.
 */
#define bmp280_validate(bmp280) (!(bmp280->slave == 0xDE && bmp280->chip_id == 0xAD))

/**
 * Read from sensor.
 * @param bmp280 Driver Sturcture.
 * @param addr Register address.
 * @param dout Data to read.
 * @param size The number of bytes to read.
 * @returns Error codes.
 */
static esp_err_t bmp280_read(bmp280_t *bmp280, uint8_t addr, uint8_t *dout, size_t size)
{
    esp_err_t err;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd)
    {
        // Write register address
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, bmp280->slave | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, addr, true);

        // Read Registers
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, bmp280->slave | I2C_MASTER_READ, true);
        i2c_master_read(cmd, dout, size, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);

        TickType_t ticks = pdMS_TO_TICKS(CONFIG_BMP280_TIMEOUT);
        err = i2c_master_cmd_begin(bmp280->i2c_port, cmd, ticks);
        i2c_cmd_link_delete(cmd);
        return err;
    }
    else
    {
        return ESP_ERR_NO_MEM;
    }
}
/**
 * Write to sensor
 * 
 * @param bmp280 Driver Sturcture.
 * @param addr Register address.
 * @param din Data to write.
 * @param size The number of bytes to write.
 * @returns Error codes.
*/
static esp_err_t bmp280_write(bmp280_t* bmp280, uint8_t addr, const uint8_t *din, size_t size)
{
    esp_err_t err;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd)
    {
        for (int i = 0; i < size; i++)
        {
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, bmp280->slave | I2C_MASTER_WRITE, true);
            // Register
            i2c_master_write_byte(cmd, addr + i, true);
            //Data
            i2c_master_write_byte(cmd, din[i], true);
        }
        i2c_master_stop(cmd);

        
        TickType_t ticks = pdMS_TO_TICKS(CONFIG_BMP280_TIMEOUT);
        err = i2c_master_cmd_begin(bmp280->i2c_port, cmd, ticks);
        i2c_cmd_link_delete(cmd);
        return err;
    }
    else
    {
        return ESP_ERR_NO_MEM;
    }
}

/**
 * Probes a given address
 * @param bmp280 Driver Sturcture.
 * @returns Error codes.
*/
static esp_err_t bmp280_probe_address(bmp280_t *bmp280)
{
    esp_err_t err = bmp280_read(bmp280, BMP280_REG_CHPID, &bmp280->chip_id, sizeof bmp280->chip_id);

    if (err == ESP_OK)
    {
        if (bmp280_verify(bmp280->chip_id))
        {
            ESP_LOGI(TAG, "Probe success: address=%hhx, id=%hhx", bmp280->slave, bmp280->chip_id);
           return ESP_OK;
        }
        else
        {
            err = ESP_ERR_NOT_FOUND;
        }
    }

    ESP_LOGW("bmp280", "Probe failure: address=%hhx, id=%hhx, reason=%s", bmp280->slave, bmp280->chip_id, esp_err_to_name(err));
    return err;
}
/**
 * Probes the sensor
 * @param bmp280 Driver Sturcture.
 * @returns Error codes.
*/
static esp_err_t bmp280_probe(bmp280_t *bmp280)
{
    ESP_LOGI(TAG, "Probing for BMP280 sensor on I2C %d", bmp280->i2c_port);

    esp_err_t err;
    bmp280->slave = 0xEC;

    if ((err = bmp280_probe_address(bmp280)) != ESP_OK)
    {
        ESP_LOGE(TAG, "Sensor not found.");
        bmp280->slave = 0xDE;
        bmp280->chip_id = 0xAD;
    }

    return err;
}

/**
 * Resets the sensor.
 * @param bmp280 Driver Sturcture.
 * @returns Error codes.
*/
static esp_err_t bmp280_reset(bmp280_t *bmp280)
{
    const static uint8_t din[] = { BMP280_RESET_VEC };
    return bmp280_write(bmp280, BMP280_REG_RESET, din, sizeof din);
}

/**
 * Calibrates the sensor
 * @param bmp280 Driver Sturcture.
 * @returns Error codes.
*/
static esp_err_t bmp280_calibrate(bmp280_t *bmp280)
{
    ESP_LOGI(TAG, "Reading out calibration values...");

    esp_err_t err;
    uint8_t buf[26];

    err = bmp280_read(bmp280, BMP280_REG_CAL_LO, buf, sizeof buf);

    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "Read Low Bank.");

    bmp280->cmps.T1 = buf[0] | (buf[1] << 8);
    bmp280->cmps.T2 = buf[2] | (buf[3] << 8);
    bmp280->cmps.T3 = buf[4] | (buf[5] << 8);
    bmp280->cmps.P1 = buf[6] | (buf[7] << 8);
    bmp280->cmps.P2 = buf[8] | (buf[9] << 8);
    bmp280->cmps.P3 = buf[10] | (buf[11] << 8);
    bmp280->cmps.P4 = buf[12] | (buf[13] << 8);
    bmp280->cmps.P5 = buf[14] | (buf[15] << 8);
    bmp280->cmps.P6 = buf[16] | (buf[17] << 8);
    bmp280->cmps.P7 = buf[18] | (buf[19] << 8);
    bmp280->cmps.P8 = buf[20] | (buf[21] << 8);
    bmp280->cmps.P9 = buf[22] | (buf[23] << 8);

    return ESP_OK;
}

/**
 * Creates the driver structure
 * @param i2c_port i2c port of the sensor.
 * @returns bmp280 Driver Sturcture.
*/
bmp280_t* bmp280_create(i2c_port_t port)
{
    bmp280_t* bmp280 = malloc(sizeof(bmp280_t));
    if (bmp280)
    {
        memset(bmp280, 0, sizeof(bmp280_t));

        bmp280->i2c_port = port;
        bmp280->slave = 0xDE;
        bmp280->chip_id = 0xAD;
    }
    return bmp280;
}

/**
 * Initializes the sensor
 * @param bmp280 Driver Sturcture.
 * @returns Error codes.
*/
esp_err_t bmp280_init(bmp280_t* bmp280)
{
    if (bmp280 == NULL) return ESP_ERR_INVALID_ARG;

    esp_err_t error = bmp280_probe(bmp280) || bmp280_reset(bmp280);

    if (error == ESP_OK)
    {
        // Give the sensor 10 ms delay to reset.
        vTaskDelay(pdMS_TO_TICKS(10));

        // Read calibration data.
        bmp280_calibrate(bmp280);

        ESP_LOGI(TAG, "Dumping calibration...");
        ESP_LOG_BUFFER_HEX("bmp280", &bmp280->cmps, sizeof(bmp280->cmps));
    }

    return error;
}

/**
 * Configures the sensor
 * @param bmp280 Driver Sturcture.
 * @param cfg Driver Sturcture config.
 * @returns Error codes.
*/
esp_err_t bmp280_configure(bmp280_t* bmp280, bmp280_config_t *cfg)
{
    if (bmp280 == NULL || cfg == NULL) return ESP_ERR_INVALID_ARG;
    if (!bmp280_validate(bmp280)) return ESP_ERR_INVALID_STATE;

    uint8_t num = (cfg->p_sampling << 2) | BMP280_MODE_SLEEP;
    return bmp280_write(bmp280, BMP280_REG_MESCTL, &num, sizeof num);
}

/**
 * Used to set the mode of the sensor. Not sure which number equals to what. 1 or 0 is prolly "Normal"
 * @param bmp280 Driver Sturcture.
 * @param mode Driver mode.
 * @returns Error codes.
*/
esp_err_t bmp280_setMode(bmp280_t* bmp280, bmp280_mode_t mode)
{
    uint8_t ctrl_mes;
    esp_err_t err;

    if ((err = bmp280_read(bmp280, BMP280_REG_MESCTL, &ctrl_mes, 1)) != ESP_OK)
        return err;

    ctrl_mes = (ctrl_mes & (~3)) | mode;

    return bmp280_write(bmp280, BMP280_REG_MESCTL, &ctrl_mes, 1);
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t BMP280_compensate_P_int64(bmp280_t *bmp280, int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)bmp280->t_fine) -128000;
    var2 = var1 * var1 * (int64_t)bmp280->cmps.P6;
    var2 = var2 + ((var1*(int64_t)bmp280->cmps.P5)<<17);
    var2 = var2 + (((int64_t)bmp280->cmps.P4)<<35);
    var1 = ((var1 * var1 * (int64_t)bmp280->cmps.P3)>>8) + ((var1 * (int64_t)bmp280->cmps.P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)bmp280->cmps.P1)>>33;
    if(var1 == 0){ 
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576-adc_P;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((int64_t)bmp280->cmps.P9) * (p>>13) * (p>>13)) >> 25;
    var2 =(((int64_t)bmp280->cmps.P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp280->cmps.P7)<<4);
    return (uint32_t)p;
}

/**
 * Used to actually read the current pressure
 * @param bmp280 Driver Sturcture.
 * @returns Error codes.
*/
esp_err_t bmp280_readout(bmp280_t *bmp280, uint32_t *pressure)
{
    if (bmp280 == NULL) return ESP_ERR_INVALID_ARG;
    if (!bmp280_validate(bmp280)) return ESP_ERR_INVALID_STATE;

    uint8_t buffer[3];
    esp_err_t error;

    if (pressure)
    {
        if ((error = bmp280_read(bmp280, BMP280_REG_PRES_MSB, buffer, sizeof buffer)) != ESP_OK)
            return error;

        int32_t adc_P = (buffer[0] << 12) | (buffer[1] << 4) | (buffer[2] >> 4);
        *pressure = BMP280_compensate_P_int64(bmp280, adc_P);
    }

    return ESP_OK;
}
