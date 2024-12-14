#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

uint8_t DC_Init();

uint8_t DC_Start();

esp_err_t print_real_time_stats();