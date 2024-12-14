#include "esp_log.h"
#include "uart.h"

static const int RX_BUF_SIZE = 1024;
static uint8_t buffer_to_send[100];

void init_uart(void) {
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    const uart_config_t uart_config_2 = {
        .baud_rate = 1200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &uart_config_2);
    uart_set_pin(UART_NUM_2, TXD_PIN_2, RXD_PIN_2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

int sendData_uart(const char* logName, const char* data)
{
    printf("\n\n");
    unsigned int checksum = data[0];
    unsigned int i = 1;
    for(i=1; i< strlen(data); ++i) {
        checksum ^= (data[i]);
    }

    memset(buffer_to_send, 0, 100);
    sprintf((char *)buffer_to_send,"$%s*%02X\r\n",data,checksum);

    const unsigned int len = strlen((char *)buffer_to_send);
    const unsigned int txBytes = uart_write_bytes(UART_NUM_1, buffer_to_send, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
    printf("\n\n");
}