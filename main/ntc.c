#include "ntc.h"

#include <string.h>
#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "driver/adc.h"

const int resistorValue = 10000; // Resistor value in ohms

// Reads the current voltage from the pin
double ntc_read(void){
    // Read the pin
    int voltage = adc_oneshot_read(ADC2_CHANNEL_9);
    return voltage;
}

// Initialises the ADC channels for the ntc resistor
void ntc_init(void){
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    
    // Configure ADC2 capture width
    // 12 bit decimal value from 0 to 4095
    adc2_config_width(ADC_WIDTH_BIT_12);
    // Configure the ADC2 channel (ADC2_CHANNEL_9 - pin 26), and setting attenuation (ADC_ATTEN_DB_11) 
    adc2_config_channel_atten(ADC2_CHANNEL_9, ADC_ATTEN_DB_11);
}