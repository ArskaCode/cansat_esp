#include "sdcard.h"

#include <string.h>
#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "lora.h"

#define MOUNT_POINT "/sdcard"
#define EXAMPLE_MAX_CHAR_SIZE    64

static const char *TAG = "Sd: ";

// Few global variables used
static sdmmc_card_t *card;
static const char mount_point[] = MOUNT_POINT;

const sdmmc_host_t host = SDSPI_HOST_DEFAULT();

/*
* Used to write to the sd card
*/
esp_err_t sd_write(const char *path, char *data){
    LORA_SEND_LOG(TAG, "Opening file.");
    FILE *f = fopen(path, "a");
    if (f == NULL) {
        LORA_SEND_LOG(TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }
    fprintf(f, data);
    fclose(f);
    LORA_SEND_LOG(TAG, "File written");

    return ESP_OK;
}

/*
* Initlizes the sd card stuff
*/
void sd_init(void){
    // Filesystem mount config
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5, // Todo check these values
        .allocation_unit_size = 16 * 1024 // Todo check these values
    };

    // sd card init
    LORA_SEND_LOG(TAG, "Initializing SD card");

    // Spi bus init
    LORA_SEND_LOG(TAG, "Spi bus initialization started.");
    esp_err_t ret;

    // Spi bus configs
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = CONFIG_SD_MOSI,
        .miso_io_num = CONFIG_SD_MISO,
        .sclk_io_num = CONFIG_SD_SCLK, 
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        LORA_SEND_LOG(TAG, "Failed to initialize bus.");
        return;
    }
    LORA_SEND_LOG(TAG, "Spi bus initialized succesfully.");

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    LORA_SEND_LOG(TAG, "Initilizing the sd card slot.");
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = CONFIG_SD_CS;
    slot_config.host_id = host.slot;
    LORA_SEND_LOG(TAG, "Sd card slot initlialized succesfully.");

    // Mounting the filesystem
    LORA_SEND_LOG(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            LORA_SEND_LOG(TAG, "Failed to mount filesystem. ");
        } else {
            LORA_SEND_LOG(TAG, "Failed to initialize the card (%s). ");
        }
        return;
    }
    LORA_SEND_LOG(TAG, "Filesystem mounted");
    
    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);
    
    // Initializing the flight data file
    LORA_SEND_LOG(TAG, "Initializing the flight data file.");
    const char *file_data = MOUNT_POINT"/flight_data.txt";
    ret = sd_write(file_data, "\n");
    if (ret != ESP_OK) {
        return;
    }
    LORA_SEND_LOG(TAG, "Flight data file initialized.");
}

/*
* Used to unmount the partition and to disable the SPI peripheral
*/
void sd_uninit(void){
    // Unmount partition and disable SPI peripheral
    LORA_SEND_LOG(TAG, "Unmounting sd card.");
    esp_vfs_fat_sdcard_unmount(mount_point, card);
    LORA_SEND_LOG(TAG, "Card unmounted");

    // deinitialize the bus after all devices are removed
    LORA_SEND_LOG(TAG, "Deinitializing the spi bus.");
    spi_bus_free(host.slot);
    LORA_SEND_LOG(TAG, "SPI bus deinitialized.");
}