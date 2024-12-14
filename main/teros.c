#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "math.h"
#include "uart.h"
#include "teros.h"
#include "i2c_master.h"

static const int RX_BUF_SIZE = 1024;

float vwc_value = 0;
float soil_temp = 0;

ret_code_t get_data_teros(float *buff){
    printf("BENNE VAGXUNK");
    uint8_t* data_2 = (uint8_t*) malloc(RX_BUF_SIZE+1);
    char *by_space[255];
    int i = 0;
    
        
        const int rxBytes_2 = uart_read_bytes(UART_NUM_2, data_2, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        printf("----RXBYTES_222:%d",rxBytes_2);
        if (rxBytes_2 > 0) {
            data_2[rxBytes_2] = 0;

            //ESP_LOGI(RX_TASK_TAG, "TEROS Read %d bytes: '%s'", rxBytes_2, data_2);
            int first_occurrence_space = 0;
            int first_occurrence_per_r = 0;
            int first_occurrence_per_n = 0;
            int c = 0;
            for(int i=0; i<rxBytes_2; i++){
                printf("----------id:%d -- %c\r\n",i,data_2[i]);
                if(data_2[i] == ' '){
                    first_occurrence_space = i;
                    //printf("first_occurrence: %d",first_occurrence_space);
                }
                if(data_2[i] == '\r' && c == 0){
                    first_occurrence_per_r = i;
                    //printf("first_occurrence_per_r: %d",first_occurrence_per_r);
                    c++;
                }
                if(data_2[i] == '\n'){
                    first_occurrence_per_n = i;
                    //printf("first_occurrence_per_n: %d",first_occurrence_per_n);
                }
                
            }

            char vwc[10];
            if(first_occurrence_space != 0 && first_occurrence_per_r != 0 && first_occurrence_per_n != 0){
                strncpy(vwc,(char *)data_2+2,first_occurrence_space-2);
                vwc_value = atof(vwc);
                printf("\n----------VOLUMETRIC WATER CONTENT: %.1f\n", vwc_value);
                char soil_temp_char[10];
                strncpy(soil_temp_char,(char *)data_2+(first_occurrence_space+1),first_occurrence_per_r-(first_occurrence_space+1));
                soil_temp = atof(soil_temp_char);
                printf("\n----------SOIL TEMPERATURE: %.1f\n\n", soil_temp);

                char EC_value[10];
                strncpy(EC_value,(char *)data_2+(first_occurrence_per_r+1),first_occurrence_per_n-(first_occurrence_per_r+1)-1);
                for(int i=0; i<sizeof(EC_value); i++){
                    printf("----------EC_VALUE: id:%d -- %c\r\n",i,EC_value[i]);
                }

                float vwc_calib = 0.0f;
                printf("----VWC_VALUE:%f",vwc_value);
                printf("----soi_temp:%f",soil_temp);
                vwc_calib = 3.879 * pow(10,-4) * vwc_value - 0.6956;
				buff[0] = vwc_calib;
				buff[1] = soil_temp;
                printf("\n---- CALIBRATED VOLUMETRIC WATER CONTENT: %.2f [m^3/m^3]\n", vwc_calib);
                //ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data_2, rxBytes_2, ESP_LOG_INFO);
                //printf("------TEROS_DATA:'%s'\n\n",data_2[rxBytes_2]);
            }else{
                buff[0] = 0.0;
				buff[1] = 0.0;
            }
        }
        //printf("------TEROS_DATA:'%s'\n\n",data_2);
        //gpio_set_level(TEROS_CTRL_PIN, 0);
        i++;

        free(data_2);

        return RET_OK;
}

float get_vwc(void){

    gpio_set_level(TEROS_CTRL_PIN, 1);
    vTaskDelay(500 /portTICK_PERIOD_MS);

	float datas[2];
    printf("----------------------------------get_vwc");
	get_data_teros(datas);
    printf("----get_vwc:%f",datas[0]);

    gpio_set_level(TEROS_CTRL_PIN, 0);
    vTaskDelay(500 /portTICK_PERIOD_MS);

	return datas[0];
}

float get_soil_temp(void){
    gpio_set_level(TEROS_CTRL_PIN, 1);
    vTaskDelay(500 /portTICK_PERIOD_MS);

	float datas[2];
	get_data_teros(datas);
    printf("----get_soil:%f",datas[1]);

    gpio_set_level(TEROS_CTRL_PIN, 0);
    vTaskDelay(500 /portTICK_PERIOD_MS);

	return datas[1];
}