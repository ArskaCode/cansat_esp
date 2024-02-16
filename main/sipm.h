/*
 * Code for sipm board
 */

#ifndef SIPM_H
#define SIPM_H

/*
 * Initialize sipm stuff
 */
void sipm_init(void);

int sipm_read_count(void);

void sipm_clear_count(void);

#endif