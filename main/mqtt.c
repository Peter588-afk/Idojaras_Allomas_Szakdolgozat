#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
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

#include "data_collector.h"

static const char *TAG = "MQTT_EXAMPLE";

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

#define SERVER_THINGSBOARD  ( 0 )

#define SERVER_OPTIN        ( 1 )

#define MQTT_SERVER_SELECT  ( SERVER_THINGSBOARD )



static esp_mqtt_client_handle_t client;

extern QueueHandle_t queue1;

typedef struct ADatas
{
    char sensor_LUX[ 100 ];
    char sensor_UV[ 100 ];
    char sensor_TEMP[ 100 ];
    char sensor_PRESSURE[ 100 ];
    char sensor_ALT[ 100 ];
    char sensor_HUMI[ 100 ];

    char sensor_LAT[ 100 ];
    char sensor_LONG[ 100 ];
    char sensor_VWC [ 100 ];
    char sensor_SOIL_TEMPERATURE [ 100 ];
}xDatas;




static void main_task(void *arg)
{
    //char lux_string[30] = {"{fgv_lux:2000}\0"};
    xDatas sensor_datas_to_send = {"UV","LUX","TEMP","PRESSURE","ALT","HUMI","LAT","LONG","VWC","SOIL_TEMPERATURE"};

    struct ADatas *peekedDatas;

    xDatas peeked_sensor_datas_to_send = {"UV","LUX","TEMP","PRESSURE","ALT","HUMI","LAT","LONG","VWC","SOIL_TEMPERATURE"};

    int c = 0;
    while(1) {
        vTaskDelay(10 / portTICK_PERIOD_MS);

        if( xQueuePeek( queue1, &( peeked_sensor_datas_to_send ), ( TickType_t ) 10 ) )
            {
                printf("\n\n Peeked Datas: UV: %s\n", peeked_sensor_datas_to_send.sensor_UV);
                printf("\n\n Peeked Datas: LUX: %s\n\n", peeked_sensor_datas_to_send.sensor_LUX);
                printf("\n\n Peeked Datas: TEMP: %s\n", peeked_sensor_datas_to_send.sensor_TEMP);
                printf("\n\n Peeked Datas: PRESSURE: %s\n\n", peeked_sensor_datas_to_send.sensor_PRESSURE);
                printf("\n\n Peeked Datas: ALT: %s\n", peeked_sensor_datas_to_send.sensor_ALT);
                printf("\n\n Peeked Datas: HUMIDITY: %s\n\n", peeked_sensor_datas_to_send.sensor_HUMI);
                printf("\n\n Peeked Datas: LATITUDE: %s\n\n", peeked_sensor_datas_to_send.sensor_LAT);
                printf("\n\n Peeked Datas: LONGITUDE: %s\n\n", peeked_sensor_datas_to_send.sensor_LONG);
                printf("\n\n Peeked Datas: VWC: %s\n\n", peeked_sensor_datas_to_send.sensor_VWC);
                printf("\n\n Peeked Datas: SOIL_TEMPERATURE: %s\n\n", peeked_sensor_datas_to_send.sensor_SOIL_TEMPERATURE);

                int msg_UV = esp_mqtt_client_publish(client, "v1/devices/me/telemetry", peeked_sensor_datas_to_send.sensor_UV, strlen(peeked_sensor_datas_to_send.sensor_UV), 1, 0);

                int msg_LUX = esp_mqtt_client_publish(client, "v1/devices/me/telemetry", peeked_sensor_datas_to_send.sensor_LUX, strlen(peeked_sensor_datas_to_send.sensor_LUX), 1, 0);

                int msg_TEMP = esp_mqtt_client_publish(client, "v1/devices/me/telemetry", peeked_sensor_datas_to_send.sensor_TEMP, strlen(peeked_sensor_datas_to_send.sensor_TEMP), 1, 0);

                int msg_PRESSURE = esp_mqtt_client_publish(client, "v1/devices/me/telemetry", peeked_sensor_datas_to_send.sensor_PRESSURE, strlen(peeked_sensor_datas_to_send.sensor_PRESSURE), 1, 0);

                int msg_ALT = esp_mqtt_client_publish(client, "v1/devices/me/telemetry", peeked_sensor_datas_to_send.sensor_ALT, strlen(peeked_sensor_datas_to_send.sensor_ALT), 1, 0);

                int msg_HUMI = esp_mqtt_client_publish(client, "v1/devices/me/telemetry", peeked_sensor_datas_to_send.sensor_HUMI, strlen(peeked_sensor_datas_to_send.sensor_HUMI), 1, 0);

                int msg_LAT = esp_mqtt_client_publish(client, "v1/devices/me/telemetry", peeked_sensor_datas_to_send.sensor_LAT, strlen(peeked_sensor_datas_to_send.sensor_LAT), 1, 0);

                int msg_LONG = esp_mqtt_client_publish(client, "v1/devices/me/telemetry", peeked_sensor_datas_to_send.sensor_LONG, strlen(peeked_sensor_datas_to_send.sensor_LONG), 1, 0);

                int msg_VWC = esp_mqtt_client_publish(client, "v1/devices/me/telemetry", peeked_sensor_datas_to_send.sensor_VWC, strlen(peeked_sensor_datas_to_send.sensor_VWC), 1, 0);

                int msg_SOIL_TEMPERATURE = esp_mqtt_client_publish(client, "v1/devices/me/telemetry", peeked_sensor_datas_to_send.sensor_SOIL_TEMPERATURE, strlen(peeked_sensor_datas_to_send.sensor_SOIL_TEMPERATURE), 1, 0);

                if( c >= 5)
                    {
                        if (print_real_time_stats() == ESP_OK) {
                        printf("\nReal time stats obtained\n");
                        } 
                        else {
                            printf("\nError getting real time stats\n");
                        }
                        c = 0;
                    }
                    c++;

                if((msg_UV != -1 || msg_UV != -2 || msg_UV != 0) && (msg_LUX != -1 || msg_LUX != -2 || msg_LUX != 0) && (msg_TEMP != -1 || msg_TEMP != -2 || msg_TEMP != 0) && (msg_PRESSURE != -1 || msg_PRESSURE != -2 || msg_PRESSURE != 0) && (msg_ALT != -1 || msg_ALT != -2 || msg_ALT != 0) && (msg_HUMI != -1 || msg_HUMI != -2 || msg_HUMI != 0) && (msg_LAT != -1 || msg_LAT != -2 || msg_LAT != 0) && (msg_LONG != -1 || msg_LONG != -2 || msg_LONG != 0))
                {
                    xQueueReceive(queue1, &peeked_sensor_datas_to_send, portMAX_DELAY);
                    printf("\n------------------RECEIVED----------------\n\n");
                }


                // if(xQueueReceive(queue1, &sensor_datas_to_send, portMAX_DELAY) == pdPASS)
                //     {
                //         printf("\n\n RECEIVED: UV: %s\n", sensor_datas_to_send.sensor_UV);
                //         printf("\n\n RECEIVED: LUX: %s\n\n", sensor_datas_to_send.sensor_LUX);
                //         esp_mqtt_client_publish(client, "v1/devices/me/telemetry", sensor_datas_to_send.sensor_UV, strlen(sensor_datas_to_send.sensor_UV), 1, 0);
                //         esp_mqtt_client_publish(client, "v1/devices/me/telemetry", sensor_datas_to_send.sensor_LUX, strlen(sensor_datas_to_send.sensor_LUX), 1, 0);
                //         if( c >= 5)
                //         {
                //             if (print_real_time_stats() == ESP_OK) {
                //             printf("\nReal time stats obtained\n");
                //             } 
                //             else {
                //                 printf("\nError getting real time stats\n");
                //             }
                //             c = 0;
                //         }
                //         c++;
                //     }
            }

        
    }
}


/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");


#if MQTT_SERVER_SELECT == SERVER_OPTIN
        msg_id = esp_mqtt_client_publish(client, "optinTeszt/valami", "data_3", 0, 1, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "ddc-iot/deviceproto/testpid1", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "optinTeszt/valami", 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_unsubscribe(client, "optinTeszt/valami");
        ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
#elif MQTT_SERVER_SELECT == SERVER_THINGSBOARD
        msg_id = esp_mqtt_client_publish(client, "v1/devices/me/telemetry", "{UJ_temperature:30}", 0, 1, 0);
        msg_id = esp_mqtt_client_publish(client, "v1/devices/me/telemetry", "{UJ_aaa:40}", 0, 1, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "v1/devices/me/telemetry", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        msg_id = esp_mqtt_client_subscribe(client, "v1/devices/me/rpc/response", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        msg_id = esp_mqtt_client_publish(client, "v1/devices/me/rpc/request", "{method:setLedSwitchState}", 0, 1, 0);
        
#endif

        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, "ddc-iot/deviceproto/testpid1", "data", 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("Proba_TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("Proba_DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,
    };
#if CONFIG_BROKER_URL_FROM_STDIN
    char line[128];

    if (strcmp(mqtt_cfg.broker.address.uri, "FROM_STDIN") == 0) {
        int count = 0;
        printf("Please enter url of mqtt broker\n");
        while (count < 128) {
            int c = fgetc(stdin);
            if (c == '\n') {
                line[count] = '\0';
                break;
            } else if (c > 0 && c < 127) {
                line[count] = c;
                ++count;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        mqtt_cfg.broker.address.uri = line;
        printf("Broker url: %s\n", line);
    } else {
        ESP_LOGE(TAG, "Configuration mismatch: wrong broker url");
        abort();
    }
#endif /* CONFIG_BROKER_URL_FROM_STDIN */

    client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);

	xTaskCreate(main_task, "MAINT", 16368, NULL, 3,NULL);

    // vTaskDelay(5000/ portTICK_PERIOD_MS); 
    DC_Init();
    xDatas sensor_datas_to_send = {"UV","LUX"};
    while(1)
    {

        vTaskDelay(10 / portTICK_PERIOD_MS);
        //vTaskDelay(100 / portTICK_PERIOD_MS);
        // if(xQueueReceive(queue1, &sensor_datas_to_send, portMAX_DELAY) == pdPASS)
        // {
        //     printf("\n\n RECEIVED: UV: %s\n", sensor_datas_to_send.sensor_UV);
        //     printf("\n\n RECEIVED: LUX: %s\n\n", sensor_datas_to_send.sensor_LUX);
        //     esp_mqtt_client_publish(client, "v1/devices/me/telemetry", sensor_datas_to_send.sensor_UV, strlen(sensor_datas_to_send.sensor_UV), 1, 0);
        //     esp_mqtt_client_publish(client, "v1/devices/me/telemetry", sensor_datas_to_send.sensor_LUX, strlen(sensor_datas_to_send.sensor_LUX), 1, 0);
            
        // }

        //------------------------------------------------------
            // float UVIndex = (float)get_uv_index();
            // float lux = (float)get_light_level();
            
            // char UV_string[30];
            // char lux_string[30];
            // sprintf(UV_string, "{fgv_UVIndex:%.1f}", UVIndex);
            // sprintf(lux_string, "{fgv_lux:%.1f}", lux);
            // ESP_LOGI(TAG,"sending %s \r\n", UV_string);
            // esp_mqtt_client_publish(client, "v1/devices/me/telemetry", UV_string, strlen(UV_string), 1, 0);
            // esp_mqtt_client_publish(client, "v1/devices/me/telemetry", lux_string, strlen(lux_string), 1, 0);
            // vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}
        