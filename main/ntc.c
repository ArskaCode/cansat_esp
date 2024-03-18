#include "ntc.h"

#include <string.h>
#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"

// Reads the current voltage from the ntc pin
double ntc_read(adc_oneshot_unit_handle_t* adc2_handle){
    double voltage;
    // Read the pin
    ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, ADC_CHANNEL_9, &voltage));

    return voltage;
}

// Initialises the ADC channels for the ntc resistor
void ntc_init(adc_oneshot_unit_handle_t* adc2_handle){
    // Init the adc_oneshot stuff
    adc_oneshot_unit_init_cfg_t init_config2 = {
        .unit_id = ADC_UNIT_2,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc2_handle));

    // Configure ADC2 capture width
    // 12 bit decimal value from 0 to 4095
    adc2_config_width(ADC_WIDTH_BIT_12);
    // Configure the ADC2 channel (ADC2_CHANNEL_9 - pin 26), and setting attenuation (ADC_ATTEN_DB_11) 
    adc2_config_channel_atten(ADC2_CHANNEL_9, ADC_ATTEN_DB_11);
}
