#include <stdlib.h>

/*
 * Read the value from the NTC resistor.
 */
double ntc_read(adc_oneshot_unit_handle_t* adc2_handle);

/*
 * Initialise the ntc stuff
 */
void ntc_init(adc_oneshot_unit_handle_t* adc2_handle);
