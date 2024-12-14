/* MQTT (over TCP) Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include <driver/gpio.h>
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_mac.h"

#include "si1133.h"
#include "si1133.c"

#include "mqtt.c"

#include "sht35.h"
#include "bme280.h"
#include "gps.h"
#include "uart.h"
#include "teros.h"


#define MQTT_ENABLED ( 1 )

#define LED_GPIO 47

static SHT35_sensor_t* sensor; 

int led_state = 0;
void setLedSwitchState(){
    
}

void app_main(void)
{
    esp_rom_gpio_pad_select_gpio(LED_GPIO); 
	gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0);

    esp_rom_gpio_pad_select_gpio(TEROS_CTRL_PIN); 
	gpio_set_direction(TEROS_CTRL_PIN, GPIO_MODE_OUTPUT);
#if MQTT_ENABLED == 1
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());
    
    // uint8_t base_mac[6] = {0};

    // esp_err_t ret = esp_read_mac(base_mac, ESP_MAC_EFUSE_FACTORY);
    // if(ret == ESP_OK)
    // {
    //     printf("MAC ADDRESS: %02X:%02X:%02X:%02X:%02X:%02X\n", base_mac[0],base_mac[1],base_mac[2],base_mac[3],base_mac[4],base_mac[5]);
    // }else {
    //     printf("Faild to read MAC");
    // }

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("outbox", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect()); // WIFI connection
#endif

    i2c_master_settings.i2c_master_scl = I2C_MASTER_SCL_IO;
    i2c_master_settings.i2c_master_sda = I2C_MASTER_SDA_IO;
    i2c_master_settings.i2c_master_freq = I2C_MASTER_FREQ_HZ;
    i2c_master_settings.i2c_master_rx_dis = I2C_MASTER_RX_BUF_DISABLE;
    i2c_master_settings.i2c_master_tx_dis = I2C_MASTER_TX_BUF_DISABLE;
    i2c_master_settings.i2c_master_timeout = I2C_MASTER_TIMEOUT_MS;


    if(D_I2CM_Init(I2C0_INDEX, &i2c_master_settings) == RET_OK) {
         // I2C initalization
        vTaskDelay(100/portTICK_PERIOD_MS);
        printf(" Si1133 Test Code \r\n");
        printf(" Initializing ...............\r\n");

        if (!si1133_init()) { // Sensor initalization
            printf("Check Serial Communication\r\n");
            while (1);
        }

        // if ((sensor = SHT35_init_sensor())) { // Sensor initalization
        //     printf("sht35 Data Communication Ready\r\n");
        //     printf("Reading .......................\r\n");
        // }

        // if (!SHT35_init_sensor_t()) { // Sensor initalization
        //     printf("Check Serial Communication\r\n");
        //     while (1);
        // }
        if (!bme280_init()) { // Sensor initalization
            printf("Check Serial Communication\r\n");
            while (1);
        }
        init_uart();
        init_gps();

        printf("Si1133 Data Communication Ready\r\n");
        printf("Reading .......................\r\n");
        vTaskDelay(10/portTICK_PERIOD_MS);
    } else {
        ESP_LOGI(TAG, "I2C init FAILED");
    }


#if MQTT_ENABLED == 1
    mqtt_app_start(); //mqtt init, measurement start and publish
#endif
}

