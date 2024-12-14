#include <string.h>
#include <stdlib.h>

#include "sht35.h"



smb_address_t sht35_target_t = SHT35_ADDR;

/// VARIABLES ///

static optn_io_buffer_t buffer_sht35;

uint8_t wa_data_buffer_sht35[10] = {0};

uint8_t ra_data_buffer_sht35[10] = {0};




/** Forward declaration of function for internal use */

static bool SHT35_is_measuring  (SHT35_sensor_t*);
static bool SHT35_send_command  (uint16_t);
static bool SHT35_read_data     (uint8_t*,  uint32_t);
static bool SHT35_read_data_for_status     (uint8_t*,  uint32_t);
static bool SHT35_get_status    (uint16_t*);
static bool SHT35_reset         ();
static float SHT35_get_temp(uint16_t temp);
static float SHT35_get_hum(uint16_t hum);

static uint8_t crc8 (uint8_t data[], int len);

/** ------------------------------------------------ */


bool SHT35_init_sensor_t(void)
{

	buffer_sht35.rbuffer_size = 10;
    buffer_sht35.wbuffer_size = 10;
    buffer_sht35.wdata = wa_data_buffer_sht35;
    buffer_sht35.rdata = ra_data_buffer_sht35;

    SHT35_sensor_t* dev= malloc (sizeof(SHT35_sensor_t));

    if ((dev = malloc (sizeof(SHT35_sensor_t))) == NULL)
        return NULL;
    
    // inititalize sensor data structure
    dev->bus  = I2C0_INDEX;
    dev->addr = sht35_target_t;
    dev->mode = SHT35_single_shot;
    dev->meas_start_time = 0;
    dev->meas_started = false;
    dev->meas_first = false;

    uint16_t status;

    // try to reset the sensor
    if (!SHT35_reset())
    {
        printf("could not reset the sensor");
    }
	SHT35_get_status(&status);
	// wa_data_buffer_sht35[0] = (uint16_t)CMD_READ_SREG;
    // wa_data_buffer_sht35[1] = (uint16_t)0xffff;

    // buffer_sht35.to_write = 2;
    // buffer_sht35.to_read = 0;

    // D_I2CM_Write(I2C0_INDEX, sht35_target_t, &buffer_sht35);

	// uint16_t id = 0;

    // wa_data_buffer_sht35[0] = (uint16_t)CMD_READ_SREG;
    // buffer_sht35.to_write = 1;
    // buffer_sht35.to_read = 1;

    // D_I2CM_WriteRead(I2C0_INDEX, sht35_target_t, &buffer_sht35);
    // id = ra_data_buffer_sht35[0];
    // printf("\n\n----------status_reg:---------%d\n\n",id);

    printf("sensor initialized\n");
    return true;
}

static bool SHT35_send_command(uint16_t cmd)
{
    // if (!dev) return false;

    // uint8_t data[2] = { cmd >> 8, cmd & 0xff };

    // int err = i2c_slave_write(dev->bus, dev->addr, 0, data, 2);

	vTaskDelay(100/portTICK_PERIOD_MS);

    wa_data_buffer_sht35[0] = cmd >> 8;
    wa_data_buffer_sht35[1] = cmd & 0xff;

    buffer_sht35.to_write = 2;
    buffer_sht35.to_read = 0;

    D_I2CM_Write(I2C0_INDEX, sht35_target_t, &buffer_sht35);

    vTaskDelay(100/portTICK_PERIOD_MS);

	return true;
}

static bool SHT35_reset ()
{
   
    // send reset command
    if (!SHT35_send_command(CMD_SOFT_RST))
    {
        printf("Sikertelen RESET");
        return false;
    }   
    // wait for small amount of time needed (according to datasheet 0.5ms)
    vTaskDelay (100 / portTICK_PERIOD_MS);

    return true;    
}


static bool SHT35_get_status (uint16_t* status)
{

    uint8_t  data[3];

    if (!SHT35_send_command(CMD_READ_SREG) || !SHT35_read_data_for_status(data, 3))
    {
        printf("Sikertelen status_register olvasas");
        return false;
    }

    for (size_t i = 0; i < sizeof(data); i++)
        {
            printf("\n\n---------------read_data_array__in_get_status:%d---------------\n\n",data[i]);
        }

    *status = data[0] << 8 | data[1];
	printf("\n\n-----------------STATUS: %d------------\n\n",*status);
    return true;
}

static bool SHT35_read_data(uint8_t *data,  uint32_t len)
{
    // if (!dev) return false;
    // int err = i2c_slave_read(dev->bus, dev->addr, 0, data, len);

    // return true;


	//uint8_t response_stored = 0;
    // wa_data_buffer_sht35[0] = 0;
    // wa_data_buffer_sht35[1] = 0;
	//wa_data_buffer_sht35[0] = CMD_READ_SREG >> 8;
   // wa_data_buffer_sht35[1] = CMD_READ_SREG && 0xff;
	buffer_sht35.to_read = len;
	buffer_sht35.to_write = len;
	vTaskDelay(100/portTICK_PERIOD_MS);

    wa_data_buffer_sht35[0] = crc8(ra_data_buffer_sht35,len);

    buffer_sht35.to_write = len;
    buffer_sht35.to_read = 0;

    D_I2CM_Write(I2C0_INDEX, sht35_target_t, &buffer_sht35);

	ret_code_t ret = D_I2CM_WriteRead(I2C0_INDEX,sht35_target_t,&buffer_sht35);

	

    vTaskDelay(100/portTICK_PERIOD_MS);

	if(ret == RET_OK) {
		//response_stored = ra_data_buffer_sht35[0];
        for (size_t i = 0; i < sizeof(data); i++)
        {
            data[i] = ra_data_buffer_sht35[i];
			wa_data_buffer_sht35[i] = 0;
			ra_data_buffer_sht35[i] = 0;
            printf("\n\n---------------read_data_array_0:%d---------------\n\n",data[i]);
        }
        
        printf("\n\n---------------read_data_0:%d---------------\n\n",ra_data_buffer_sht35[0]);
        printf("\n\n---------------read_data_1:%d---------------\n\n",ra_data_buffer_sht35[1]);
        printf("\n\n---------------read_data_2:%d---------------\n\n",ra_data_buffer_sht35[2]);
	} else {
		printf("elseben");
		ra_data_buffer_sht35[0] = 0xFF;
	}

    vTaskDelay(100/portTICK_PERIOD_MS);
	return true;
}

static bool SHT35_read_data_for_status(uint8_t *data,  uint32_t len)
{
    // if (!dev) return false;
    // int err = i2c_slave_read(dev->bus, dev->addr, 0, data, len);

    // return true;


	//uint8_t response_stored = 0;
    // wa_data_buffer_sht35[0] = 0;
    // wa_data_buffer_sht35[1] = 0;
	//wa_data_buffer_sht35[0] = CMD_READ_SREG >> 8;
   // wa_data_buffer_sht35[1] = CMD_READ_SREG && 0xff;
	buffer_sht35.to_read = len;
	buffer_sht35.to_write = len;

	ret_code_t ret = D_I2CM_WriteRead(I2C0_INDEX,sht35_target_t,&buffer_sht35);

	vTaskDelay(100/portTICK_PERIOD_MS);

    wa_data_buffer_sht35[0] = crc8(ra_data_buffer_sht35,len);

    buffer_sht35.to_write = 1;
    buffer_sht35.to_read = 0;

    D_I2CM_Write(I2C0_INDEX, sht35_target_t, &buffer_sht35);

    vTaskDelay(100/portTICK_PERIOD_MS);

	if(ret == RET_OK) {
		//response_stored = ra_data_buffer_sht35[0];
        for (size_t i = 0; i < sizeof(data); i++)
        {
            data[i] = ra_data_buffer_sht35[i];
			wa_data_buffer_sht35[i] = 0;
			ra_data_buffer_sht35[i] = 0;
            printf("\n\n---------------read_data_array_0:%d---------------\n\n",data[i]);
        }
        
        printf("\n\n---------------read_data_0:%d---------------\n\n",ra_data_buffer_sht35[0]);
        printf("\n\n---------------read_data_1:%d---------------\n\n",ra_data_buffer_sht35[1]);
        printf("\n\n---------------read_data_2:%d---------------\n\n",ra_data_buffer_sht35[2]);
	} else {
		printf("elseben");
		ra_data_buffer_sht35[0] = 0xFF;
	}

    vTaskDelay(100/portTICK_PERIOD_MS);
	return true;
}

ret_code_t SHT35_read_meas_data_single_shot(uint16_t cfg_cmd, float *temp, float *humi){
	uint8_t data[6] = {0};
	uint16_t temp_hex = 0;
	uint16_t humi_hex = 0;

	SHT35_send_command(cfg_cmd);
	SHT35_read_data(data, sizeof(data));

	temp_hex=(data[0] << 8) | data[1];
	humi_hex =(data[3] << 8) | data[4];

	*temp =SHT35_get_temp(temp_hex);
	*humi=SHT35_get_hum(humi_hex);

	return RET_OK;
}


static float SHT35_get_temp(uint16_t temp){
	return(temp / 65535.00) * 175 - 45;
}

static float SHT35_get_hum(uint16_t humi){
	return(humi / 65535.00) * 100.0;
}

uint16_t SHT35_temp_to_hex(float temp)
{
	return (uint16_t)((temp+45)*65535.0/175);
}

uint16_t  SHT35_hum_to_hex(float humi)
{
	return (uint16_t)(humi/100.0*65535);
}

// bool SHT35_measure (float* temperature, float* humidity)
// {
//     if ((!temperature && !humidity)) return false;

//     if (!SHT35_start_measurement (SHT35_single_shot, SHT35_high))
//         return false;

//     vTaskDelay(SHT35_MEAS_DURATION_TICKS[SHT35_high]);

//     SHT35_raw_data_t raw_data;
    
//     if (!SHT35_get_raw_data (raw_data))
//         return false;
        
//     return SHT35_compute_values (raw_data, temperature, humidity);
// }


// bool SHT35_start_measurement (SHT35_sensor_t* dev, SHT35_mode_t mode, SHT35_repeat_t repeat)
// {
//     if (!dev) return false;
    
//     dev->error_code = SHT35_OK;
//     dev->mode = mode;
//     dev->repeatability = repeat;
    
//     // start measurement according to selected mode and return an duration estimate
//     if (!SHT35_send_command(dev, SHT35_MEASURE_CMD[mode][repeat]))
//     {
        
//     }

//     dev->meas_start_time = 0;
//     dev->meas_started = true;
//     dev->meas_first = true;

//     return true;
// }


// uint8_t SHT35_get_measurement_duration (SHT35_repeat_t repeat)
// {
//     return SHT35_MEAS_DURATION_TICKS[repeat];  // in RTOS ticks
// }


// bool SHT35_get_raw_data(SHT35_sensor_t* dev, SHT35_raw_data_t raw_data)
// {
//     if (!dev || !raw_data) return false;

//     dev->error_code = SHT35_OK;

//     if (!dev->meas_started)
//     {
//         dev->error_code = SHT35_MEAS_NOT_STARTED;
//         return false;
//     }

//     // if (SHT35_is_measuring(dev))
//     // {
//     //     dev->error_code = SHT35_MEAS_STILL_RUNNING;
//     //     return false;
//     // }

//     // send fetch command in any periodic mode (mode > 0) before read raw data
//     //SHT35_send_command(dev, CMD_FETCH_DATA);
//     if (dev->mode && !SHT35_send_command(dev, CMD_FETCH_DATA))
//     {
//         dev->error_code |= SHT35_SEND_FETCH_CMD_FAILED;
//         return false;
//     }
//     float temperature;
//     float humidity;
    
//     // read raw data
//     if (!SHT35_read_data(dev, raw_data, 6))
//     {
//         dev->error_code |= SHT35_READ_RAW_DATA_FAILED;
//         return false;
//     }
//     for (size_t i = 0; i < 6; i++)
//     {
//         printf("\n\n---------raw_data:%d-------------\n\n", raw_data[i]);
//     }
//     SHT35_compute_values(raw_data,&temperature,&humidity);
//         printf("fgv-ben - SHT35 Sensor: %.2f °C, %.2f %% \n", 
//                                 temperature, humidity);
    
//     // reset first measurement flag
//     dev->meas_first = false;
    
//     // reset measurement started flag in single shot mode
//     if (dev->mode == SHT35_single_shot)
//         dev->meas_started = false;
    
//     // check temperature crc
//     if (crc8(raw_data,2) != raw_data[2])
//     {
//         dev->error_code |= SHT35_WRONG_CRC_TEMPERATURE;
//         return false;
//     }

//     // check humidity crc
//     if (crc8(raw_data+3,2) != raw_data[5])
//     {
//         dev->error_code |= SHT35_WRONG_CRC_HUMIDITY;
//         return false;
//     }

//     return true;
// }


// bool SHT35_compute_values (SHT35_raw_data_t raw_data, float* temperature, float* humidity)
// {
//     if (!raw_data) return false;

//     if (temperature){
//         *temperature = ((((raw_data[0] * 256.0) + raw_data[1]) * 175) / 65535.0) - 45;
//         printf("\n\n%.2f\n\n",*temperature);
//     }
        

//     if (humidity){
//         *humidity = ((((raw_data[3] * 256.0) + raw_data[4]) * 100) / 65535.0);
//         printf("\n\n%.2f\n\n",*humidity);
//     }
        
//     return true;    
// }


// bool SHT35_get_results (SHT35_sensor_t* dev, float* temperature, float* humidity)
// {
//     if (!dev || (!temperature && !humidity)) return false;

//     SHT35_raw_data_t raw_data;
    
//     if (!SHT35_get_raw_data (dev, raw_data))
//         return false;
        
//     return SHT35_compute_values (raw_data, temperature, humidity);
// }

// /* Functions for internal use only */

// // static bool SHT35_is_measuring (SHT35_sensor_t* dev)
// // {
// //     if (!dev) return false;

// //     dev->error_code = SHT35_OK;

// //     // not running if measurement is not started at all or 
// //     // it is not the first measurement in periodic mode
// //     if (!dev->meas_started || !dev->meas_first)
// //       return false;
    
// //     // not running if time elapsed is greater than duration
// //     uint32_t elapsed = sdk_system_get_time() - dev->meas_start_time;

// //     return elapsed < SHT35_MEAS_DURATION_US[dev->repeatability];
// // }









const uint8_t g_polynom_t = 0x31;

static uint8_t crc8 (uint8_t data[], int len)
{
    // initialization value
    uint8_t crc = 0xff;
    
    // iterate over all bytes
    for (int i=0; i < len; i++)
    {
        crc ^= data[i];  
    
        for (int i = 0; i < 8; i++)
        {
            bool xor = crc & 0x80;
            crc = crc << 1;
            crc = xor ? crc ^ g_polynom_t : crc;
        }
    }

    return crc;
} 