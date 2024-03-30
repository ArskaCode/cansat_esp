#include "ntc.h"

#include <string.h>
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "lora.h"

static const char* TAG = "ntc";

//adc2_handle
adc_cali_handle_t adc2_cali_handle = NULL;
adc_oneshot_unit_handle_t adc2_handle;
#define NTC_ADC2_CHAN ADC_CHANNEL_9 // adc2_chan9 = gpio 26
#define NTC_ADC_ATTEN ADC_ATTEN_DB_12


static int adc_raw[2][10];
static int voltage[2][10];

// Reads the current voltage from the ntc pin
int ntc_read(){
    LORA_SEND_LOG(TAG, "Reading ntc voltage");
    LORA_SEND_ERROR(TAG, adc_oneshot_read(adc2_handle, NTC_ADC2_CHAN, &adc_raw[1][0]));
    LORA_SEND_ERROR(TAG, adc_cali_raw_to_voltage(adc2_cali_handle, adc_raw[1][0], &voltage[1][0]));
    return voltage[1][0]; //prolly return what the esp_error_check returns like on the sd card stuff
}

// Initialises the ADC channels for the ntc resistor
void ntc_init(){
    //-------------ADC2 config---------------//
    LORA_SEND_LOG(TAG, "Configuring adc2");
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = NTC_ADC_ATTEN,
    };

    //-------------ADC2 Init---------------//
    LORA_SEND_LOG(TAG, "Initilazing adc2");
    adc_oneshot_unit_init_cfg_t init_config2 = {
        .unit_id = ADC_UNIT_2,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    LORA_SEND_ERROR(TAG, adc_oneshot_new_unit(&init_config2, &adc2_handle));

    //-------------ADC2 Calibration Init---------------//
    LORA_SEND_LOG(TAG, "Initializing adc2 calibration");
    bool do_calibration2 = NTC_adc_calibration_init(ADC_UNIT_2, NTC_ADC2_CHAN, NTC_ADC_ATTEN, &adc2_cali_handle);

    //-------------ADC2 Config---------------//
    LORA_SEND_LOG(TAG, "Adc2 config");
    LORA_SEND_ERROR(TAG, adc_oneshot_config_channel(adc2_handle, NTC_ADC2_CHAN, &config));
    
    //-------------ADC2 Calibratin-------------//
    LORA_SEND_LOG(TAG, "Calibrating adc2");
    if (do_calibration2) {
        LORA_SEND_ERROR(TAG, adc_cali_raw_to_voltage(adc2_cali_handle, adc_raw[1][0], &voltage[1][0]));
        //LORA_SEND_LOG(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_2 + 1, NTC_ADC2_CHAN, voltage[1][0]); 
    }
}

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool NTC_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

    if (!calibrated) {
        LORA_SEND_LOG(TAG, "calibration scheme version is Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
    *out_handle = handle;
    if (ret == ESP_OK) {
        LORA_SEND_LOG(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        LORA_SEND_LOG(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}