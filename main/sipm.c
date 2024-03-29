#include "sipm.h"

#include "driver/i2c.h"
#include "driver/pulse_cnt.h"


typedef struct {
    pcnt_unit_handle_t unit;
    pcnt_channel_handle_t chan;
} sipm_state_t;

static const char *TAG = "sipm";

static sipm_state_t sipm_state;


void sipm_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = CONFIG_I2C_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = CONFIG_I2C_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
        .clk_flags = 0,
    };

    ESP_ERROR_CHECK(i2c_param_config(0, &conf));

    pcnt_unit_config_t unit_config = {
        .low_limit = 0,
        .high_limit = SHRT_MAX,
    };

    pcnt_unit_handle_t unit;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &unit));

    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = CONFIG_SIPM_O1,
        .level_gpio_num = -1,

        .flags.virt_level_io_level = 0,
    };

    pcnt_channel_handle_t chan;
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &chan_config, &chan));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP));

    sipm_state.unit = unit;
    sipm_state.chan = chan;
}

int sipm_read_count(void)
{
    int value;
    ESP_ERROR_CHECK(pcnt_unit_get_count(sipm_state.unit, &value));
    return value;
}

void sipm_clear_count(void)
{
    ESP_ERROR_CHECK(pcnt_unit_clear_count(sipm_state.unit));
}