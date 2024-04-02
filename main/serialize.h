#ifndef SERIALIZE_H
#define SERIALIZE_H

#include <stdio.h>
#include "gps.h"

typedef struct {
    uint32_t ntcOut;
    uint32_t bmxOut;
    int16_t gyroOut[3];
    int16_t accelOut[3];
    gps_data_t gps_data;
    uint32_t sipmOut;
    uint32_t time;
} data_struct_t;

size_t serialize_data(void* buf, size_t length, data_struct_t* data);
size_t serialize_log(void* buf, size_t length, char* str);

#endif