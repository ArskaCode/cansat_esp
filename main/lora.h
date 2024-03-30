/*
 * EByte E220-900T22D LoRa
 */

#ifndef LORA_H
#define LORA_H

#include <stdlib.h>
#include <string.h>

#define LORA_SEND_ERROR(tag, x) do {            \
    esp_err_t err_rc_ = (x);                    \
    char combined[128];                         \
    snprintf(combined, sizeof(combined), "%s: %s", (tag), esp_err_to_name(err_rc_)); \
    lora_transmit(&combined, strlen(combined)); \
    if (err_rc_ == ESP_OK){                     \
        true;                                   \
    }                                           \
    false;                                      \
} while(0)

#define LORA_SEND_LOG(tag, text) do {           \
    char combined[128];                         \
    sniprintf(combined, sizeof(combined), "0%s: %s", (tag), (text)); \
    lora_transmit(&combined, strlen(combined)); \
} while(0)                                      \

/*
 * Contains info about the lora module
 */
typedef struct __attribute__((packed)) {
    uint8_t model;
    uint8_t version;
    uint8_t features;
} lora_info_t;

/*
 * Initialize lora stuff
 */
void lora_init(void);

/*
 * Get lora info
 */
void lora_get_info(lora_info_t* info);

/*
 * Set lora address
 */
void lora_set_address(uint16_t);

/*
 * Set lora channel
 */
void lora_set_channel(uint8_t);

/*
 * Transmit data
 */
void lora_transmit(const void*, size_t);

/*
 * Receive data
 */
void lora_receive(void*, size_t);

#endif