/*
 * Code for sipm board
 */

#ifndef SIPM_H
#define SIPM_H

#include <stdint.h>

/*
 * Initialize sipm stuff
 */
void sipm_init(void);

/*
 * Sample the sipm

 */
int16_t sipm_sample(void);

#endif