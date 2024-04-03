#include "gps.h"
#include "ext/nmea_parser.h"
#include "esp_log.h"

nmea_parser_handle_t nmea_hdl;

static const char *TAG = "GPS";

void gps_init(void)
{
    ESP_LOGI(TAG, "Initializing.");
    nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT();
    nmea_hdl = nmea_parser_init(&config);
    ESP_LOGI(TAG, "Init done.");
}

void gps_get_data(gps_data_t* gps_data)
{
    gps_t* gps = nmea_get_latest_gps(nmea_hdl);

    gps_data->latitude = gps->latitude;
    gps_data->longitude = gps->longitude;
    gps_data->altitude = gps->altitude;
}
