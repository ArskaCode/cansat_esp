#ifndef NTC_H
#define NTC_H

#include <stdint.h>

/*
 * Initialise the ntc stuff
 */
void ntc_init(void);

/*
 * Read the V/Vref value from the NTC resistor.
 */
uint16_t ntc_read(void);

#endif