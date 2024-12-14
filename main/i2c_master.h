/**
 * @file i2c_master.h
 * @brief Provides basic functions to handle the I2C peripheral in master mode.
 *
 */

//-----------------------------------------------------------------------------
// MODULE INCLUDE GUARDS
//-----------------------------------------------------------------------------

#ifndef DRIVER_I2C_MASTER_H_
#define DRIVER_I2C_MASTER_H_


//-----------------------------------------------------------------------------
// HEADER LEVEL INCLUDES
//-----------------------------------------------------------------------------
#include <stdio.h>
#include <stdint.h>
#include "i2c_master.h"
#include "driver/i2c.h"
#include <freertos/FreeRTOS.h>
//-----------------------------------------------------------------------------
// TYPES, ENUMS, CONSTANTS
//-----------------------------------------------------------------------------


typedef uint32_t ret_code_t;

#define RET_OK          0      
#define RET_FAIL        -1

typedef uint32_t smb_address_t;

/**
 * General struct used for buffering data
 * The data field must be set up with a statically/dynamically allocated buffer before use!
 */
typedef struct {
    // General buffer properties
    // Total size of the write buffer
    uint32_t wbuffer_size : 16;
    // Total size of the read buffer
    uint32_t rbuffer_size : 16;

    // Used during buffer transactions
    // Number of bytes to write
    uint16_t to_write : 8;
    // Number of bytes to read
    uint16_t to_read : 8;

    // Buffer containing data to write
    uint8_t * wdata;
    // Buffer to read data into
    uint8_t * rdata;
} optn_io_buffer_t;


typedef struct {
	// uint16_t baudrate;
	// uint16_t glitch_filter;
	// uint8_t pinselect;
    int32_t i2c_master_scl;
    int32_t i2c_master_sda;
    uint32_t i2c_master_freq;
    uint8_t i2c_master_rx_dis;
    uint8_t i2c_master_tx_dis;
    uint8_t i2c_master_timeout;
}i2cm_settings_t;

#ifndef I2C_ENUM_DEFINED
#define I2C_ENUM_DEFINED
typedef enum {
	I2C0_INDEX = 0,
    I2C0_INDEX_1 = 1,
	I2C_INDEX_MAX
	// TODO
} i2c_num_t;
#endif /* I2C_ENUM_DEFINED */

//-----------------------------------------------------------------------------
// VARIABLES
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// API FUNCTIONS AND MACROS
//-----------------------------------------------------------------------------

/**
 * Write data to the specified target. This is a thread-safe, blocking method.
 * @param i2c The number of the i2c peripheral to use.
 * @param settings Settings for the I2C peripheral.
 * @return RET_OK on success, error code otherwise.
 */
ret_code_t D_I2CM_Init(i2c_num_t i2c, i2cm_settings_t * settings);

/**
 * Enable the I2C peripheral.
 * @param i2c The number of the i2c peripheral to use.
 * @return RET_OK on success, error code otherwise.
 */
ret_code_t D_I2CM_Enable(i2c_num_t i2c);

/**
 * Disable the I2C peripheral.
 * @param i2c The number of the i2c peripheral to use.
 * @return RET_OK on success, error code otherwise.
 */
ret_code_t D_I2CM_Disable(i2c_num_t i2c);

/**
 * Write data to the specified target. This is a thread-safe, blocking method.
 * @param i2c The number of the i2c peripheral to use.
 * @param target The address of the target node (at the first 7 bits).
 * @param buffer The buffer that contains the data to send.
 * @return RET_OK on success, error code if the transaction failed.
 * @warn If transaction is considered to be failed, I2C_AbortTransmission is called.
 */
ret_code_t D_I2CM_Write(i2c_num_t i2c, smb_address_t target, const optn_io_buffer_t * buffer);

/**
 * Read data from the specified target. This is a thread-safe, blocking method.
 * @param i2c The number of the i2c peripheral to use.
 * @param target The address of the target node (at the first 7 bits).
 * @param buffer The buffer to read into.
 * @param len The number of bytes to read.
 * @return RET_OK on success, error code if the transaction failed.
 * @warn If transaction is considered to be failed, I2C_AbortTransmission is called.
 */
ret_code_t D_I2CM_Read(i2c_num_t i2c, smb_address_t target, optn_io_buffer_t * buffer);

/**
 * Write, then read data to and from the specified target. This is a thread-safe, blocking method.
 * @param i2c The number of the i2c peripheral to use.
 * @param target The address of the target node (at the first 7 bits).
 * @param buffer The buffer that contains the data to write down. After completion, stores the data received.
 * @param out_len The number of bytes to write.
 * @param in_len The number of bytes to read.
 * @return RET_OK on success, error code if the transaction failed.
 * @note This function utilizes Repeated Start.
 * @warn If a transaction is considered to be failed, I2C_AbortTransmission is called.
 */
ret_code_t D_I2CM_WriteRead(i2c_num_t i2c, smb_address_t target, const optn_io_buffer_t * buffer);

#endif /* DRIVER_I2C_MASTER_H_ */
