#include <stdio.h>
#include <string.h>

#ifndef MPU9250_H
#define MPU9250_H

/*
* Used to init the mpu stuff
*/
void mpu9250_init(int16_t *gyro_cal);

/*
* Used to get the acceleration values from the mpu
*/
void mpu9250_get_accel(int16_t accel[3]);

/*
* Used to get the gyro values from the mpu
*/
void mpu9250_get_gyro(int16_t gyro[3]);

/*
* Used to calibrate the gyro values for the mpu
*/
void calibrate_gyro(int16_t gyroCal[3], int loop);

#endif