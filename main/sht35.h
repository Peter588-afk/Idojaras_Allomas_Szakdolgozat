#ifndef __SHT35_H__
#define __SHT35_H__

#include "stdint.h"
#include "stdbool.h"
#include "i2c_master.h"

// definition of possible I2C slave addresses
#define SHT35_ADDR 0x45        // ADDR pin connected to VDD

// definition of error codes
#define SHT35_OK                     0
#define SHT35_NOK                    -1

#define SHT35_I2C_ERROR_MASK         0x000f
#define SHT35_DRV_ERROR_MASK         0xfff0

// error codes for I2C interface ORed with SHT35 error codes
#define SHT35_I2C_READ_FAILED        1
#define SHT35_I2C_SEND_CMD_FAILED    2
#define SHT35_I2C_BUSY               3

// SHT35 driver error codes OR ed with error codes for I2C interface
#define SHT35_MEAS_NOT_STARTED       (1  << 8)
#define SHT35_MEAS_ALREADY_RUNNING   (2  << 8)
#define SHT35_MEAS_STILL_RUNNING     (3  << 8)
#define SHT35_READ_RAW_DATA_FAILED   (4  << 8)

#define SHT35_SEND_MEAS_CMD_FAILED   (5  << 8)
#define SHT35_SEND_RESET_CMD_FAILED  (6  << 8)
#define SHT35_SEND_STATUS_CMD_FAILED (7  << 8)
#define SHT35_SEND_FETCH_CMD_FAILED  (8  << 8)

#define SHT35_WRONG_CRC_TEMPERATURE  (9  << 8)
#define SHT35_WRONG_CRC_HUMIDITY     (10 << 8)

#define		CMD_BREAK			0x3093
#define		CMD_SOFT_RST		0x30A2
#define		CMD_ENABLE_HEAT		0x306D
#define		CMD_DISABLE_HEAT	0x3066	
#define		CMD_READ_SREG		0xF32D
#define		CMD_CLEAR_SREG		0x3041
#define		CMD_FETCH_DATA		0xE000 

#define     CMD_READ_HIGH_ALERT_LIMIT_SET_VALUE     0XE11F
#define     CMD_READ_HIGH_ALERT_LIMIT_CLEAR_VALUE   0XE114
#define     CMD_READ_LOW_ALERT_LIMIT_SET_VALUE      0XE102
#define     CMD_READ_LOW_ALERT_LIMIT_CLEAR_VALUE    0XE109

#define     CMD_WRITE_HIGH_ALERT_LIMIT_SET_VALUE    0X611D
#define     CMD_WRITE_HIGH_ALERT_LIMIT_CLEAR_VALUE  0X6116
#define     CMD_WRITE_LOW_ALERT_LIMIT_SET_VALUE     0X6100
#define     CMD_WRITE_LOW_ALERT_LIMIT_CLEAR_VALUE     0X610B

#define     HIGH_REP_WITH_STRCH      0x2C06

#define SHT35_RAW_DATA_SIZE 6

/**
 * @brief	raw data type
 */
typedef uint8_t SHT35_raw_data_t [SHT35_RAW_DATA_SIZE];


/**
 * @brief   possible measurement modes
 */
typedef enum {
    SHT35_single_shot = 0,  // one single measurement
    SHT35_periodic_05mps,   // periodic with 0.5 measurements per second (mps)
    SHT35_periodic_1mps,    // periodic with   1 measurements per second (mps)
    SHT35_periodic_2mps,    // periodic with   2 measurements per second (mps)
    SHT35_periodic_4mps,    // periodic with   4 measurements per second (mps)
    SHT35_periodic_10mps    // periodic with  10 measurements per second (mps)
} SHT35_mode_t;
    
    
/**
 * @brief   possible repeatability modes
 */
typedef enum {
    SHT35_high = 0,
    SHT35_medium,
    SHT35_low
} SHT35_repeat_t;

/**
 * @brief 	SHT35 sensor device data structure type
 */
typedef struct {

    uint32_t        error_code;      // combined error codes
    
    uint8_t         bus;             // I2C bus at which sensor is connected
    uint8_t         addr;            // I2C slave address of the sensor
    
    SHT35_mode_t    mode;            // used measurement mode
    SHT35_repeat_t  repeatability;   // used repeatability
 
    bool            meas_started;    // indicates whether measurement started
    uint32_t        meas_start_time; // measurement start time in us
    bool            meas_first;      // first measurement in periodic mode
    
} SHT35_sensor_t;    



ret_code_t SHT35_read_meas_data_single_shot(uint16_t cfg_cmd, float* temp, float* hum);
uint16_t SHT35_temp_to_hex(float temp);
uint16_t SHT35_hum_to_hex(float hum);

/**
 * @brief	Initialize a SHT35 sensor
 * 
 * The function creates a data structure describing the sensor and
 * initializes the sensor device.
 *  
 * @param   bus       I2C bus at which the sensor is connected
 * @param   addr      I2C slave address of the sensor
 * @return            pointer to sensor data structure, or NULL on error
 */
SHT35_sensor_t* SHT35_init_sensor (void);
bool SHT35_init_sensor_t (void);


/**
 * @brief   High level measurement function
 *
 * For convenience this function comprises all three steps to perform
 * one measurement in only one function:
 *
 * 1. Starts a measurement in single shot mode with high reliability
 * 2. Waits using *vTaskDelay* until measurement results are available 
 * 3. Returns the results in kind of floating point sensor values 
 *
 * This function is the easiest way to use the sensor. It is most suitable
 * for users that don't want to have the control on sensor details.
 *
 * Please note: The function delays the calling task up to 30 ms to wait for
 * the  the measurement results. This might lead to problems when the function
 * is called from a software timer callback function.
 *
 * @param   dev         pointer to sensor device data structure
 * @param   temperature returns temperature in degree Celsius   
 * @param   humidity    returns humidity in percent
 * @return              true on success, false on error
 */
bool SHT35_measure (SHT35_sensor_t* dev, float* temperature, float* humidity);


/**
 * @brief	Start the measurement in single shot or periodic mode
 *
 * The function starts the measurement either in *single shot mode* 
 * (exactly one measurement) or *periodic mode* (periodic measurements)
 * with given repeatabilty.
 *
 * In the *single shot mode*, this function has to be called for each
 * measurement. The measurement duration has to be waited every time
 * before the results can be fetched. 
 *
 * In the *periodic mode*, this function has to be called only once. Also 
 * the measurement duration has to be waited only once until the first
 * results are available. After this first measurement, the sensor then
 * automatically performs all subsequent measurements. The rate of periodic
 * measurements can be 10, 4, 2, 1 or 0.5 measurements per second (mps).
 * 
 * Please note: Due to inaccuracies in timing of the sensor, the user task
 * should fetch the results at a lower rate. The rate of the periodic
 * measurements is defined by the parameter *mode*.
 *
 * @param   dev         pointer to sensor device data structure
 * @param   mode        measurement mode, see type *SHT35_mode_t*
 * @param   repeat      repeatability, see type *SHT35_repeat_t*
 * @return              true on success, false on error
 */
bool SHT35_start_measurement (SHT35_sensor_t* dev, SHT35_mode_t mode,
                              SHT35_repeat_t repeat);

/**
 * @brief   Get the duration of a measurement in RTOS ticks.
 *
 * The function returns the duration in RTOS ticks required by the sensor to
 * perform a measurement for the given repeatability. Once a measurement is
 * started with function *SHT35_start_measurement* the user task can use this
 * duration in RTOS ticks directly to wait with function *vTaskDelay* until
 * the measurement results can be fetched.
 *
 * Please note: The duration only depends on repeatability level. Therefore,
 * it can be considered as constant for a repeatibility.
 *
 * @param   repeat      repeatability, see type *SHT35_repeat_t*
 * @return              measurement duration given in RTOS ticks
 */
uint8_t SHT35_get_measurement_duration (SHT35_repeat_t repeat);


/**
 * @brief	Read measurement results from sensor as raw data
 *
 * The function read measurement results from the sensor, checks the CRC
 * checksum and stores them in the byte array as following.
 *
 *      data[0] = Temperature MSB
 *      data[1] = Temperature LSB
 *      data[2] = Temperature CRC
 *      data[3] = Humidity  MSB
 *      data[4] = Humidity  LSB 
 *      data[2] = Humidity  CRC
 *
 * In case that there are no new data that can be read, the function fails.
 * 
 * @param   dev         pointer to sensor device data structure
 * @param   raw_data    byte array in which raw data are stored 
 * @return              true on success, false on error
 */
bool SHT35_get_raw_data(SHT35_sensor_t* dev, SHT35_raw_data_t raw_data);


/**
 * @brief	Computes sensor values from raw data
 *
 * @param   raw_data    byte array that contains raw data  
 * @param   temperature returns temperature in degree Celsius   
 * @param   humidity    returns humidity in percent
 * @return              true on success, false on error
 */
bool SHT35_compute_values (SHT35_raw_data_t raw_data, 
                           float* temperature, float* humidity);


/**
 * @brief	Get measurement results in form of sensor values
 *
 * The function combines function *SHT35_read_raw_data* and function 
 * *SHT35_compute_values* to get the measurement results.
 * 
 * In case that there are no results that can be read, the function fails.
 *
 * @param   dev         pointer to sensor device data structure
 * @param   temperature returns temperature in degree Celsius   
 * @param   humidity    returns humidity in percent
 * @return              true on success, false on error
 */
bool SHT35_get_results (SHT35_sensor_t* dev, 
                        float* temperature, float* humidity);

#endif /* __SHT35_H__ */