#ifndef GPS_H
#define GPS_H

#include <string.h>
#include "esp_err.h"

typedef struct {
    float latitude;
    float longitude;
    float altitude;
} gps_data_t;

void gps_init(void);

void gps_get_data(gps_data_t* gps_data);

#endif