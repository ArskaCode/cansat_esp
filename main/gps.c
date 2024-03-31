#include "gps.h"
#include "ext/nmea_parser.h"

nmea_parser_handle_t nmea_hdl;

static const char *TAG = "gps";

void gps_init(void)
{
    nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT();
    nmea_hdl = nmea_parser_init(&config);
}

void gps_get_data(gps_data_t* gps_data)
{
    gps_t* gps = nmea_get_latest_gps(nmea_hdl);

    gps_data->latitude = gps->latitude;
    gps_data->longitude = gps->longitude;
    gps_data->altitude = gps->altitude;
}
