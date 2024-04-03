#include "serialize.h"
#include "gps.h"
#include "esp_log.h"

const char* TAG = "serialize";

typedef struct {
    void* ptr;
    size_t idx;
    size_t len;
} serializer_t;


void ensure_len(serializer_t* ser, size_t sz)
{
    if (ser->idx + sz > ser->len)
    {
        ESP_LOGE(TAG, "buffer overflow");
        abort();
    }
}

void write_bytes(serializer_t* ser, void* src, size_t len)
{
    ensure_len(ser, len);
    memcpy(ser->ptr + ser->idx, src, len);
    ser->idx += len;
}

// Utility function to write a 32-bit unsigned integer to buffer
void write_u32(serializer_t* ser, uint32_t value)
{
    write_bytes(ser, &value, sizeof(uint32_t));
}

// Utility function to write a 32-bit unsigned integer to buffer
void write_i16(serializer_t* ser, int16_t value) 
{
    write_bytes(ser, &value, sizeof(int16_t));
}

void write_u8(serializer_t* ser, uint8_t value)
{
    write_bytes(ser, &value, sizeof(uint8_t));
}

// Utility function to write a 32-bit unsigned integer to buffer
void write_f32(serializer_t *ser, float value)
{
    write_bytes(ser, &value, sizeof(float));
}

size_t serialize_data(void* buf, size_t len, data_struct_t* data)
{
    serializer_t ser = {
        .ptr = buf,
        .idx = 0,
        .len = len,
    };

    write_u8(&ser, 0);
    write_u32(&ser, data->time);
    write_u32(&ser, data->sipmOut);
    write_u32(&ser, data->ntcOut);
    write_u32(&ser, data->bmxOut);

    for (int i = 0; i < 3; i++) {
        write_i16(&ser, data->gyroOut[i]);
    }

    for (int i = 0; i < 3; i++) {
        write_i16(&ser, data->accelOut[i]);
    }

    write_f32(&ser, data->gps_data.latitude);
    write_f32(&ser, data->gps_data.longitude);
    write_f32(&ser, data->gps_data.altitude);

    return ser.idx;
}

size_t serialize_log(void* buf, size_t len, char* str)
{
    serializer_t ser = {
        .ptr = buf,
        .idx = 0,
        .len = len,
    };

    size_t msgLen = strlen(str);

    if (msgLen > 0xFF)
    {
        ESP_LOGW(TAG, "too long error string");
        msgLen = 0xFF;
    }

    write_u8(&ser, 1);
    write_u8(&ser, msgLen);
    write_bytes(&ser, str, msgLen);

    return ser.idx;
}