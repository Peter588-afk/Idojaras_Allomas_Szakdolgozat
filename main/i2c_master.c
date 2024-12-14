#include "i2c_master.h"

#define I2C_MASTER_SCL_IO           9      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           8      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          200000                   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       (uint8_t)1000U

#define si1133_SENSOR_ADDR          0x55        /*!< Slave address of the si1133 sensor */

SemaphoreHandle_t i2c_mutex = NULL;

i2cm_settings_t master_settings = {0};

static ret_code_t i2cLock(uint32_t timeout);

static ret_code_t i2cUnlock();

ret_code_t D_I2CM_Init(i2c_num_t i2c, i2cm_settings_t * settings)
{

	i2c_mutex = xSemaphoreCreateMutex();

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = settings->i2c_master_sda,
        .scl_io_num = settings->i2c_master_scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = settings->i2c_master_freq,
    };

    i2c_param_config(i2c, &conf);

   	i2c_driver_install(i2c, conf.mode, settings->i2c_master_rx_dis, settings->i2c_master_tx_dis, 0);
	
	 return RET_OK;
}

ret_code_t D_I2CM_Write(i2c_num_t i2c, smb_address_t target, const optn_io_buffer_t * buffer)
{
	ret_code_t ret;
	ret_code_t esp_ret;
	ret = i2cLock(1000);
	if(ret == RET_OK) {
		esp_ret = i2c_master_write_to_device(i2c, target, buffer->wdata, buffer->to_write, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
		ret = esp_ret == RET_OK ? RET_OK : RET_FAIL;
		i2cUnlock();
	}
	return ret;
}

ret_code_t D_I2CM_WriteRead(i2c_num_t i2c, smb_address_t target, const optn_io_buffer_t * buffer)
{
	ret_code_t ret;
	ret_code_t esp_ret;
	ret = i2cLock(1000);
	if(ret == RET_OK) {
	    esp_ret = i2c_master_write_read_device(i2c, target, buffer->wdata, buffer->to_write, buffer->rdata, buffer->to_read,I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
		ret = esp_ret == RET_OK ? RET_OK : RET_FAIL;
		i2cUnlock();
	}
	return ret;
}

ret_code_t i2cLock(uint32_t timeout)
{
	ret_code_t ret;

	if(i2c_mutex != NULL)
	{
		ret = (xSemaphoreTake(i2c_mutex,timeout / portTICK_PERIOD_MS) == pdTRUE) ? RET_OK : RET_FAIL;
	}else{
		ret = RET_FAIL;
	}

	return ret;
}

ret_code_t i2cUnlock()
{
	ret_code_t ret;

	ret = (xSemaphoreGive(i2c_mutex) == pdTRUE) ? RET_OK : RET_FAIL;
	
	return ret;
}
