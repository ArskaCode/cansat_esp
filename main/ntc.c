#include "ntc.h"

#include <string.h>
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "hal/adc_ll.h"


static const char* TAG = "NTC";


static adc_oneshot_unit_handle_t adc2_handle;


static const adc_unit_t ADC_UNIT = ADC_UNIT_2;
static const adc_channel_t NTC_VREF = ADC_CHANNEL_7;
static const adc_channel_t NTC_OUTPUT = ADC_CHANNEL_9; 


static const size_t N_SAMPLES = 8;


void ntc_init()
{
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_0,
    };

    adc_oneshot_unit_init_cfg_t init_config2 = {
        .unit_id = ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc2_handle));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, NTC_OUTPUT, &config));
    adc_ll_vref_output(ADC_UNIT, NTC_VREF, true);
}

uint16_t ntc_read()
{
    uint16_t result = 0;
    for (int i = 0; i < N_SAMPLES; ++i)
    {
        int value;
        ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, NTC_OUTPUT, &value));
        result += value;
    }
    return result / N_SAMPLES;
}