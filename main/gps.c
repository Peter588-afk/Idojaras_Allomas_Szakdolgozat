#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "math.h"
#include "uart.h"
#include "gps.h"
#include "i2c_master.h"

static const int RX_BUF_SIZE = 1024;


static int split_string_by_comma(char *string, char **values, int max_values)
{
	int i = 0;

	values[i++] = string;
	while (i < max_values && NULL != (string = strchr(string, ','))) {
		*string = '\0';
		values[i++] = ++string;
        
	}

	return i;
}

static int split_string_by_dollar(char *string, char **values, int max_values)
{
	int i = 0;

	values[i++] = string;
	while (i < max_values && NULL != (string = strchr(string, '$'))) {
		*string = '\0';
		values[i++] = ++string;
        
	}
	return i;
}


uint8_t buffer_to_nmea[100];

unsigned int c_vals=0;

ret_code_t get_GPS_data(float *buff){
    RMC_code gprmc_data;
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    char *sentences[255];
    char *by_comma[255];

    const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            //ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            
            //ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);

        }
        //46.25162024309419, 20.146305438631522
    const char str[] = "$GPRMC,203522.00,A,4625.16202430,N,2014.63054386,W,0.004,133.4,130522,0.0,E,D*2B";
        unsigned int n_vals = split_string_by_dollar((char *)str, sentences, sizeof(sentences));
        //printf("\n\n%u\n\n",n_vals);
        for(int j = 0; j < n_vals; j++){
            //printf("\n\nElement: %d : %s\n\n",j,sentences[j]);
            if(strncmp(sentences[j], "GPRMC", 5) == 0){
                c_vals = split_string_by_comma((char *)sentences[j], by_comma, sizeof(by_comma));
            }
        }
        
        
        //printf("\n\n%u\n\n",c_vals);
        // for(int k = 0; k < c_vals; k++){
        //     printf("\n\nElement by_comma: %d : %s\n\n",k,by_comma[k]);
            
        // }
        gprmc_data.latitude = atof(by_comma[3]);
        gprmc_data.longitude = atof(by_comma[5]);
        //gprmc_data.time_stamp = atof(by_comma[1]);
        //gprmc_data.date = atof(by_comma[9]);
        printf("\n----------Latitude: %f\r\n",gprmc_data.latitude);
        //printf("----------timestamp: %f\r\n",gprmc_data.time_stamp);
        //printf("----------date: %f\r\n",gprmc_data.date);
        buff[0] = gprmc_data.latitude/100;
        buff[1] = gprmc_data.longitude/100;
        buff[3] = gprmc_data.time_stamp;
        buff[4] = gprmc_data.date;

        free(data);
        return RET_OK;
}

void init_gps(void)
{
    printf("\n\n");
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    sendData_uart(TX_TASK_TAG, "PMTK225,0");
    vTaskDelay(200 / portTICK_PERIOD_MS);
    sendData_uart(TX_TASK_TAG, "PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0");
    vTaskDelay(200 / portTICK_PERIOD_MS);
    sendData_uart(TX_TASK_TAG, "PPMTK869,1,1");
    vTaskDelay(200 / portTICK_PERIOD_MS);
    sendData_uart(TX_TASK_TAG, "PQTXT,W,1,1");
    vTaskDelay(200 / portTICK_PERIOD_MS);
    sendData_uart(TX_TASK_TAG, "PMTK 286,1");
    vTaskDelay(200 / portTICK_PERIOD_MS);
    
    printf("\n\n");
}

float get_lat(void){
    float datas[2];
	get_GPS_data(datas);
	return datas[0];
}

float get_long(void){
    float datas[2];
	get_GPS_data(datas);
	return datas[1];
}