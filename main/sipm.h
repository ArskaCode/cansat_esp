/*
 * Code for sipm board
 */
#include <stdbool.h>

#ifndef SIPM_H
#define SIPM_H

/*
 * Initialize sipm stuff
 */
void sipm_init(bool *inits);

int sipm_read_count(void);

void sipm_clear_count(void);

#endif