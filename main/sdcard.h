#ifndef SDCARD_H
#define SDCARD_H

#include "esp_err.h"

/*
* Used to write to the sd card
*/
esp_err_t sd_write(const char *path, char *data);

/*
* Initlizes the sd card stuff
*/
void sd_init(void);

/*
* Used to unmount the partition and to disable the SPI peripheral
*/
void sd_uninit(void);

#endif