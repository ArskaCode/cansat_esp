#ifndef NTC_H
#define NTC_H

#include "esp_adc/adc_cali.h"
/*
 * Read the value from the NTC resistor.
 */
int ntc_read(void);

/*
 * Initialise the ntc stuff
 */
void ntc_init(void);

/*
* Used to calibrate the adc stuff
*/
bool NTC_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
#endif