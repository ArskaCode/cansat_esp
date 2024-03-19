#include <string.h>
#include "esp_err.h"

esp_err_t sd_write(const char *path, char *data);
void sd_init(void);
void sd_uninit(void);
