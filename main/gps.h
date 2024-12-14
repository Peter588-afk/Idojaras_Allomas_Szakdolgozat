#include "i2c_master.h"

typedef struct{
    char talkerID[20];
    float time_stamp;
    char status;
    float latitude;
    char lat_dir;
    float longitude;
    char long_dir;
    float date;
}RMC_code;


void init_gps(void);
ret_code_t get_GPS_data(float *buff);
float get_lat(void);
float get_long(void);