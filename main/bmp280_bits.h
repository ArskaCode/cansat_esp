/**
 * BMP280 - BME280 & BMP280 Driver for Esspressif ESP-32.
 *
 * MIT License
 *
 * Copyright (C) 2020 Halit Utku Maden
 * Please contact at <utkumaden@hotmail.com>
 */

#ifndef _BMP280_DEFAULT_H_
#define _BMP280_DEFAULT_H_
#ifndef _BMP280_H_
#endif

typedef enum bmp280_tsmpl_t {
    BMP280_TEMPERATURE_OVERSAMPLING_NONE = 0x0,
    BMP280_TEMPERATURE_OVERSAMPLING_X1,
    BMP280_TEMPERATURE_OVERSAMPLING_X2,
    BMP280_TEMPERATURE_OVERSAMPLING_X4,
    BMP280_TEMPERATURE_OVERSAMPLING_X8,
    BMP280_TEMPERATURE_OVERSAMPLING_X16,
} bmp280_tsmpl_t;

typedef enum bmp280_psmpl_t {
    BMP280_PRESSURE_OVERSAMPLING_NONE = 0x0,
    BMP280_PRESSURE_OVERSAMPLING_X1,
    BMP280_PRESSURE_OVERSAMPLING_X2,
    BMP280_PRESSURE_OVERSAMPLING_X4,
    BMP280_PRESSURE_OVERSAMPLING_X8,
    BMP280_PRESSURE_OVERSAMPLING_X16,
} bmp280_psmpl_t;

#if !(CONFIG_BMP280_EXPECT_BMP280)
typedef enum bme280_hsmpl_t {
    BMP280_HUMIDITY_OVERSAMPLING_NONE = 0x0,
    BMP280_HUMIDITY_OVERSAMPLING_X1,
    BMP280_HUMIDITY_OVERSAMPLING_X2,
    BMP280_HUMIDITY_OVERSAMPLING_X4,
    BMP280_HUMIDITY_OVERSAMPLING_X8,
    BMP280_HUMIDITY_OVERSAMPLING_X16,
} bme280_hsmpl_t;
#endif

typedef enum bmp280_tstby_t {
    BMP280_STANDBY_0M5 = 0x0,
    BMP280_STANDBY_62M5,
    BMP280_STANDBY_125M,
    BMP280_STANDBY_250M,
    BMP280_STANDBY_500M,
    BMP280_STANDBY_1000M,
    BME280_STANDBY_10M,
    BME280_STANDBY_20M,
    BMP280_STANDBY_2000M = BME280_STANDBY_10M,
    BMP280_STANDBY_4000M = BME280_STANDBY_20M,
} bmp280_tstby_t;

typedef enum bmp280_iirf_t {
    BMP280_IIR_NONE = 0x0,
    BMP280_IIR_X1,
    BMP280_IIR_X2,
    BMP280_IIR_X4,
    BMP280_IIR_X8,
    BMP280_IIR_X16,
} bmp280_iirf_t;

typedef enum bmp280_mode_t {
    /** Sensor does no measurements. */
    BMP280_MODE_SLEEP = 0,
    /** Sensor is in a forced measurement cycle. Sleeps after finishing. */
    BMP280_MODE_FORCE = 1,
    /** Sensor does measurements. Never sleeps. */
    BMP280_MODE_CYCLE = 3,
} bmp280_mode_t;

typedef struct bmp280_config_t {
    bmp280_tsmpl_t t_sampling;
    bmp280_psmpl_t p_sampling;
    bmp280_tstby_t t_standby;
    bmp280_iirf_t iir_filter;
    #if !(CONFIG_BMP280_EXPECT_BMP280)
    bme280_hsmpl_t h_sampling;
    #endif
} bmp280_config_t;

#if (CONFIG_BMP280_DEFAULT_TEMPERATURE_OVERSAMPLING_NONE)
#define BMP280_DEFAULT_TEMPERATURE_OVERSAMPLING BMP280_TEMPERATURE_OVERSAMPLING_NONE
#elif (CONFIG_BMP280_DEFAULT_TEMPERATURE_OVERSAMPLING_X1)
#define BMP280_DEFAULT_TEMPERATURE_OVERSAMPLING BMP280_TEMPERATURE_OVERSAMPLING_X1
#elif (CONFIG_BMP280_DEFAULT_TEMPERATURE_OVERSAMPLING_X2)
#define BMP280_DEFAULT_TEMPERATURE_OVERSAMPLING BMP280_TEMPERATURE_OVERSAMPLING_X2
#elif (CONFIG_BMP280_DEFAULT_TEMPERATURE_OVERSAMPLING_X4)
#define BMP280_DEFAULT_TEMPERATURE_OVERSAMPLING BMP280_TEMPERATURE_OVERSAMPLING_X4
#elif (CONFIG_BMP280_DEFAULT_TEMPERATURE_OVERSAMPLING_X8)
#define BMP280_DEFAULT_TEMPERATURE_OVERSAMPLING BMP280_TEMPERATURE_OVERSAMPLING_X8
#else
#define BMP280_DEFAULT_TEMPERATURE_OVERSAMPLING BMP280_TEMPERATURE_OVERSAMPLING_X16
#endif

#if CONFIG_BMP280_DEFAULT_PRESSURE_OVERSAMPLING_NONE
#define BMP280_DEFAULT_PRESSURE_OVERSAMPLING BMP280_PRESSURE_OVERSAMPLING_NONE
#elif CONFIG_BMP280_DEFAULT_PRESSURE_OVERSAMPLING_X1
#define BMP280_DEFAULT_PRESSURE_OVERSAMPLING BMP280_PRESSURE_OVERSAMPLING_X1
#elif CONFIG_BMP280_DEFAULT_PRESSURE_OVERSAMPLING_X2
#define BMP280_DEFAULT_PRESSURE_OVERSAMPLING BMP280_PRESSURE_OVERSAMPLING_X2
#elif CONFIG_BMP280_DEFAULT_PRESSURE_OVERSAMPLING_X4
#define BMP280_DEFAULT_PRESSURE_OVERSAMPLING BMP280_PRESSURE_OVERSAMPLING_X4
#elif CONFIG_BMP280_DEFAULT_PRESSURE_OVERSAMPLING_X8
#define BMP280_DEFAULT_PRESSURE_OVERSAMPLING BMP280_PRESSURE_OVERSAMPLING_X8
#else
#define BMP280_DEFAULT_PRESSURE_OVERSAMPLING BMP280_PRESSURE_OVERSAMPLING_X16
#endif

#if (CONFIG_BMP280_DEFAULT_STANDBY_0M5)
#define BMP280_DEFAULT_STANDBY BMP280_STANDBY_0M5
#elif (CONFIG_BMP280_DEFAULT_STANDBY_62M5)
#define BMP280_DEFAULT_STANDBY BMP280_STANDBY_62M5
#elif (CONFIG_BMP280_DEFAULT_STANDBY_125M)
#define BMP280_DEFAULT_STANDBY BMP280_STANDBY_125M
#elif (CONFIG_BMP280_DEFAULT_STANDBY_250M)
#define BMP280_DEFAULT_STANDBY BMP280_STANDBY_250M
#elif (CONFIG_BMP280_DEFAULT_STANDBY_500M)
#define BMP280_DEFAULT_STANDBY BMP280_STANDBY_500M
#elif (CONFIG_BMP280_DEFAULT_STANDBY_1000M)
#define BMP280_DEFAULT_STANDBY BMP280_STANDBY_1000M
#elif (CONFIG_BMP280_DEFAULT_STANDBY_10M)
#define BMP280_DEFAULT_STANDBY BME280_STANDBY_10M
#else
#define BMP280_DEFAULT_STANDBY BMP280_STANDBY_20M
#endif

#if (CONFIG_BMP280_DEFAULT_IIR_NONE)
#define BMP280_DEFAULT_IIR BMP280_IIR_NONE
#elif (CONFIG_BMP280_DEFAULT_IIR_X2)
#define BMP280_DEFAULT_IIR BMP280_IIR_X2
#elif (CONFIG_BMP280_DEFAULT_IIR_X4)
#define BMP280_DEFAULT_IIR BMP280_IIR_X4
#elif (CONFIG_BMP280_DEFAULT_IIR_X8)
#define BMP280_DEFAULT_IIR BMP280_IIR_X8
#else
#define BMP280_DEFAULT_IIR BMP280_IIR_X16
#endif

#ifndef CONFIG_BMP280_EXPECT_BMP280
    #if (CONFIG_BMP280_DEFAULT_HUMIDITY_OVERSAMPLING_NONE)
        #define BMP280_DEFAULT_HUMIDITY_OVERSAMPLING BMP280_HUMIDITY_OVERSAMPLING_NONE
    #elif (CONFIG_BMP280_DEFAULT_HUMIDITY_OVERSAMPLING_X1)
        #define BMP280_DEFAULT_HUMIDITY_OVERSAMPLING BMP280_HUMIDITY_OVERSAMPLING_X1
    #elif (CONFIG_BMP280_DEFAULT_HUMIDITY_OVERSAMPLING_X2)
        #define BMP280_DEFAULT_HUMIDITY_OVERSAMPLING BMP280_HUMIDITY_OVERSAMPLING_X2
    #elif (CONFIG_BMP280_DEFAULT_HUMIDITY_OVERSAMPLING_X4)
        #define BMP280_DEFAULT_HUMIDITY_OVERSAMPLING BMP280_HUMIDITY_OVERSAMPLING_X4
    #elif (CONFIG_BMP280_DEFAULT_HUMIDITY_OVERSAMPLING_X8)
        #define BMP280_DEFAULT_HUMIDITY_OVERSAMPLING BMP280_HUMIDITY_OVERSAMPLING_X8
    #else
        #define BMP280_DEFAULT_HUMIDITY_OVERSAMPLING BMP280_HUMIDITY_OVERSAMPLING_X16
    #endif
#endif

#if !(CONFIG_BMP280_EXPECT_BMP280)
    #define BMP280_DEFAULT_CONFIG ((bmp280_config_t) { BMP280_DEFAULT_TEMPERATURE_OVERSAMPLING, BMP280_DEFAULT_PRESSURE_OVERSAMPLING, BMP280_DEFAULT_STANDBY, BMP280_DEFAULT_IIR, BMP280_DEFAULT_HUMIDITY_OVERSAMPLING })
#else
    #define BMP280_DEFAULT_CONFIG ((bmp280_config_t) { BMP280_DEFAULT_TEMPERATURE_OVERSAMPLING, BMP280_DEFAULT_PRESSURE_OVERSAMPLING, BMP280_DEFAULT_STANDBY, BMP280_DEFAULT_IIR})
#endif

#endif