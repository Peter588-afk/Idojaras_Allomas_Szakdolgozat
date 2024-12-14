#include "i2c_master.h"

#define TEROS_CTRL_PIN 7

ret_code_t get_data_teros(float *buff);

float get_vwc(void);
float get_soil_temp(void);