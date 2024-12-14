#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "math.h"

#define TXD_PIN 17
#define RXD_PIN 18

#define TXD_PIN_2 4
#define RXD_PIN_2 5

void init_uart(void);
int sendData_uart(const char* logName, const char* data);