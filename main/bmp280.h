/**
 * BMX280 - BME280 & BMP280 Driver for Esspressif ESP-32.
 *
 * MIT License
 *
 * Copyright (C) 2020 Halit Utku Maden
 * Please contact at <utkumaden@hotmail.com>
 */

#ifndef _BMX280_H_
#define _BMX280_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <limits.h>
#include "driver/i2c.h"
#include "sdkconfig.h"

#define BMXAPI extern

/**
 * Anonymous structure to driver settings.
 */
typedef struct bmx280_t bmx280_t;

#include "bmp280_bits.h"

/**
 * Create an instance of the BMX280 driver.
 * @param port The I2C port to use.
 * @return A non-null pointer to the driver structure on success.
 */
BMXAPI bmx280_t* bmx280_create(i2c_port_t port);

/**
 * Probe for the sensor and read calibration data.
 * @param bmx280 Driver structure.
 */
BMXAPI esp_err_t bmx280_init(bmx280_t* bmx280);

/**
 * Configure the sensor with the given parameters.
 * @param bmx280 Driver structure.
 * @param configuration The configuration to use.
 */
BMXAPI esp_err_t bmx280_configure(bmx280_t* bmx280, bmx280_config_t *cfg);

/**
 * Set the sensor mode of operation.
 * @param bmx280 Driver structure.
 * @param mode The mode to set the sensor to.
 */
BMXAPI esp_err_t bmx280_setMode(bmx280_t* bmx280, bmx280_mode_t mode);

/**
 * Read sensor values as fixed point numbers.
 * @param bmx280 Driver structure.
 * @param temperature The temperature in C (0.01 degree C increments)
 * @param pressure The pressure in Pa (1/256 Pa increments)
 * @param humidity The humidity in %RH (1/1024 %RH increments) (UINT32_MAX when invlaid.)
 */
esp_err_t bmx280_readout(bmx280_t *bmx280, uint32_t *pressure);

#ifdef __cplusplus
};
#endif

#endif