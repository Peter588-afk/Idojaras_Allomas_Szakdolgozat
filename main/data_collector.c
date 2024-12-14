#include <data_collector.h>

#include "si1133.h"
#include "bme280.h"
#include "gps.h"
#include "uart.h"
#include "teros.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_log.h"

#define ARRAY_SIZE_OFFSET   5

QueueHandle_t queue1 = 0;

static const char *TAG = "MQTT_EXAMPLE";

float UVIndex = 0;
float lux = 0;
float temp = 0;
float pressure = 0;
float alt = 0;
float humi = 0;

float lat = 0;
float lng = 0;
float vwc = 0;
float soil_temperature = 0;
float soil_temperature_temp = 0;

typedef struct
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



static void data_collector_task(void *arg)
{
	//vTaskDelay(5000/portTICK_PERIOD_MS);
    int i = 0;
    int c = 0;
    xDatas sensor_datas_to_send = {"UV","LUX","TEMP","PRESSURE","ALT","HUMI","LAT","LONG","VWC","SOIL_TEMPERATURE"};
    while (1) {
            i++;
        if(i >= 10)
        {
			UVIndex = (float)get_uv_index();
			lux = (float)get_light_level();

            temp = bme280_getTemperature();
            vTaskDelay(300/portTICK_PERIOD_MS);
            pressure = (bme280_getPressure()/100);
            vTaskDelay(300/portTICK_PERIOD_MS);
            alt = bme280_calcAltitude(pressure);
            vTaskDelay(300/portTICK_PERIOD_MS);
            humi = bme280_getHumidity();
            vTaskDelay(300/portTICK_PERIOD_MS);	

            lat = get_lat();
            vTaskDelay(300/portTICK_PERIOD_MS);	
            lng = get_long();
            vTaskDelay(300/portTICK_PERIOD_MS);	
            soil_temperature_temp = get_soil_temp();
            vTaskDelay(300/portTICK_PERIOD_MS);	
            soil_temperature = get_soil_temp();
            vTaskDelay(300/portTICK_PERIOD_MS);	
            vwc = get_vwc();
            vTaskDelay(300/portTICK_PERIOD_MS);	

        	sprintf(sensor_datas_to_send.sensor_UV, "{fgv_UVIndex:%.2f}", UVIndex);
			sprintf(sensor_datas_to_send.sensor_LUX, "{fgv_lux:%.2f}", lux);
            sprintf(sensor_datas_to_send.sensor_TEMP, "{fgv_temperature:%.2f}", temp);
			sprintf(sensor_datas_to_send.sensor_PRESSURE, "{fgv_pressure:%.2f}", pressure);
            sprintf(sensor_datas_to_send.sensor_ALT, "{fgv_alt:%.2f}", alt);
			sprintf(sensor_datas_to_send.sensor_HUMI, "{fgv_humidity:%.2f}", humi);
            sprintf(sensor_datas_to_send.sensor_LAT, "{fgv_lat:%.10f}", lat);
			sprintf(sensor_datas_to_send.sensor_LONG, "{fgv_long:%.10f}", lng);
            sprintf(sensor_datas_to_send.sensor_VWC, "{fgv_vwc:%.5f}", vwc);
			sprintf(sensor_datas_to_send.sensor_SOIL_TEMPERATURE, "{fgv_soil_temperature:%.2f}", soil_temperature);
            printf("\n\nSEND: UV: %s\n", sensor_datas_to_send.sensor_UV);
			printf("\n\nSEND: LUX: %s\n\n", sensor_datas_to_send.sensor_LUX);
            printf("\n\nSEND: TEMP: %s\n", sensor_datas_to_send.sensor_TEMP);
			printf("\n\nSEND: PRESSURE: %s\n\n", sensor_datas_to_send.sensor_PRESSURE);
            printf("\n\nSEND: ALT: %s\n", sensor_datas_to_send.sensor_ALT);
			printf("\n\nSEND: HUMIDITY: %s\n\n", sensor_datas_to_send.sensor_HUMI);
            printf("\n\nSEND: LAT: %s\n", sensor_datas_to_send.sensor_LAT);
			printf("\n\nSEND: LONG: %s\n\n", sensor_datas_to_send.sensor_LONG);
            printf("\n\nSEND: VWC: %s\n", sensor_datas_to_send.sensor_VWC);
			printf("\n\nSEND: temperature: %s\n\n", sensor_datas_to_send.sensor_SOIL_TEMPERATURE);

            if(c >= 5)
            {
                if (print_real_time_stats() == ESP_OK) {
                printf("\nReal time stats obtained\n");
                } 
                else {
                    printf("\nError getting real time stats\n");
                }
                c = 0;
            }
            
            xQueueSend(queue1, &sensor_datas_to_send, 0);
            i = 0;
            c++;
        }
        vTaskDelay(10/portTICK_PERIOD_MS);
		ESP_LOGD(TAG,"DC tick");
    }
}

esp_err_t print_real_time_stats()
{
    TaskStatus_t *start_array = NULL, *end_array = NULL;
    UBaseType_t start_array_size, end_array_size;
    uint32_t start_run_time, end_run_time;
    esp_err_t ret;

    //Allocate array to store current task states
    start_array_size = uxTaskGetNumberOfTasks() + ARRAY_SIZE_OFFSET;
    start_array = malloc(sizeof(TaskStatus_t) * start_array_size);
    if (start_array == NULL) {
        ret = ESP_ERR_NO_MEM;
        goto exit;
    }
    //Get current task states
    start_array_size = uxTaskGetSystemState(start_array, start_array_size, &start_run_time);
    if (start_array_size == 0) {
        ret = ESP_ERR_INVALID_SIZE;
        goto exit;
    }


    printf("\r\n%20s  %20s", "Task name", "free heap");
    //Match each task in start_array to those in the end_array
    for (int i = 0; i < start_array_size; i++) {

        printf("\r\n%20s  %20lu", start_array[i].pcTaskName, start_array[i].usStackHighWaterMark);
        
    }

    ret = ESP_OK;

exit:    //Common return path
    free(start_array);
    free(end_array);
    return ret;
}

uint8_t DC_Init() {
	queue1 = xQueueCreate(10, sizeof(xDatas));

    //xTaskCreatePinnedToCore(data_collector_task, "DATA", 1024, NULL, 3, NULL, 0);

	xTaskCreate(data_collector_task, "DATA", 16368, NULL, 3,NULL);

	return 0;
}

uint8_t DC_Start() {

	return 0;
}