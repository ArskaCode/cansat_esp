#ifndef LORA_H
#define LORA_H

#include <stdlib.h>

/*
 * EByte E220-900T22D LoRa
 */


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
 * Transmit data
 */
void lora_transmit(const void*, size_t);

/*
 * Receive data
 */
void lora_receive(void*, size_t);

#endif