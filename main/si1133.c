#include "si1133.h"
#include "si1133_config.h"
#include "driver/i2c.h"
#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_master.h"


//static const char *TAG = "i2c-simple-example";

#define I2C_MASTER_SCL_IO           9      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           8      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          100000                /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       (uint8_t)1000U

#define si1133_SENSOR_ADDR                 0x55        /*!< Slave address of the si1133 sensor */

#define si1133_RESET_BIT                   7
#define SI1133_RESET_SW 0X01

smb_address_t si_1133_target = si1133_SENSOR_ADDR;
i2cm_settings_t i2c_master_settings = {0};

/// VARIABLES ///

static optn_io_buffer_t buffer_t;

uint8_t w_data_buffer[10] = {0};

uint8_t r_data_buffer[10] = {0};

const SI1133_LuxCoeff_TypeDef lk = {
    {   {     0, 209 },           /**< coeff_high[0]   */
        {  1665, 93  },           /**< coeff_high[1]   */
        {  2064, 65  },           /**< coeff_high[2]   */
        { -2671, 234 } },         /**< coeff_high[3]   */
    {   {     0, 0   },           /**< coeff_low[0]    */
        {  1921, 29053 },         /**< coeff_low[1]    */
        { -1022, 36363 },         /**< coeff_low[2]    */
        {  2320, 20789 },         /**< coeff_low[3]    */
        {  -367, 57909 },         /**< coeff_low[4]    */
        { -1774, 38240 },         /**< coeff_low[5]    */
        {  -608, 46775 },         /**< coeff_low[6]    */
        { -1503, 51831 },         /**< coeff_low[7]    */
        { -1886, 58928 } }        /**< coeff_low[8]    */
};

/***************************************************************************//**
 * @brief
 *    Coefficients for UV index calculation
 ******************************************************************************/
const SI1133_Coeff_TypeDef uk[2] = {
    { 1281, 30902 },            /**< coeff[0]        */
    { -638, 46301 }             /**< coeff[1]        */
};
    
static esp_err_t si1133_writeParam(enum Parameter address, uint8_t value)
{
    uint8_t response_stored = 0;

    w_data_buffer[0] = SI1133_REG_RESPONSE0;
    buffer_t.to_read = 1;
    buffer_t.to_write = 1;

    ret_code_t ret = D_I2CM_WriteRead(I2C0_INDEX,si_1133_target,&buffer_t);
    if(ret == RET_OK) {
        response_stored = r_data_buffer[0];
        response_stored &= (uint8_t)RSP0_COUNTER_MASK;
    } else {
        r_data_buffer[0] = 0xFF;
    }


    w_data_buffer[0] = SI1133_REG_HOSTIN0;
    w_data_buffer[1] = value;

    buffer_t.to_write = 2;
    buffer_t.to_read = 0;

    ret = D_I2CM_Write(I2C0_INDEX, si_1133_target, &buffer_t);

//-------------------- HOSTIN visszaolvasás
    /*uint8_t host = 0;

    w_data_buffer[0] = SI1133_REG_HOSTIN0;
    //r_data_buffer[0] = 0;
    buffer_t.to_read = 1;
    buffer_t.to_write = 1;

    ret = D_I2CM_WriteRead(I2C0_INDEX,si_1133_target,&buffer_t);
    if(ret == RET_OK) {
        host = r_data_buffer[0];
        printf("\n\n--------hostin ifben:------%d\n\n",host);
        //host &= (uint8_t)RSP0_COUNTER_MASK;
    } else {
        r_data_buffer[0] = 0xFF;
    }*/
//--------------------
    w_data_buffer[0] = SI1133_REG_COMMAND;
    w_data_buffer[1] = 0x80 + ((uint8_t)address & 0x3F);

    buffer_t.to_write = 2;
    buffer_t.to_read = 0;
    ret |= D_I2CM_Write(I2C0_INDEX, si_1133_target, &buffer_t);


    if(ret != RET_OK) return RET_FAIL;
    return SI1133_OK;
}

static esp_err_t si1133_init(void)
{
    buffer_t.rbuffer_size = 10;
    buffer_t.wbuffer_size = 10;
    buffer_t.wdata = w_data_buffer;
    buffer_t.rdata = r_data_buffer;

    reset();

    uint8_t id = 0;

    w_data_buffer[0] = SI1133_REG_PARTID;
    buffer_t.to_write = 1;
    buffer_t.to_read = 1;

    D_I2CM_WriteRead(I2C0_INDEX, si_1133_target, &buffer_t);
    id = r_data_buffer[0];
    printf("\n\n----------part-id:---------%d\n\n",id);

    if(id != 0x33) {
          printf("Si1133 --- INIT ERROR");
          while(1);
    }

    //Selected Channel 0(16bits) - 1(24bits)
    si1133_writeParam(SI1133_PARAM_CHLIST, 0X0f);

  //=======================================================

    //Configuration for Channel 0
    //Set Photodiode Channel Rate
    si1133_writeParam(SI1133_PARAM_ADCCONFIG0, 0x78 );
    si1133_writeParam(SI1133_PARAM_ADCSENS0, 0x71);
    si1133_writeParam(SI1133_PARAM_ADCPSOT0, 0x40);
    //si1133_writeParam(SI1133_PARAM_MEASCONFIG0, 0x40);

    //=======================================================
    //chanel 1
    si1133_writeParam(SI1133_PARAM_ADCCONFIG1, 0x4d );
    si1133_writeParam(SI1133_PARAM_ADCSENS1, 0xe1);
    si1133_writeParam(SI1133_PARAM_ADCPSOT1, 0x40);

    //chanel 2
    si1133_writeParam(SI1133_PARAM_ADCCONFIG20, 0x41 );
    si1133_writeParam(SI1133_PARAM_ADCSENS2, 0xe1);
    si1133_writeParam(SI1133_PARAM_ADCPSOT2, 0x50);

    //chanel 3
    si1133_writeParam(SI1133_PARAM_ADCCONFIG3, 0x4d );
    si1133_writeParam(SI1133_PARAM_ADCSENS3, 0x87);
    si1133_writeParam(SI1133_PARAM_ADCPSOT3, 0x40);

    w_data_buffer[0] = SI1133_REG_IRQ_ENABLE;
    w_data_buffer[1] = 0x0f;

    buffer_t.to_write = 2;
    buffer_t.to_read = 0;

    D_I2CM_Write(I2C0_INDEX, si_1133_target, &buffer_t);

    vTaskDelay(100/portTICK_PERIOD_MS);

    /* IRQ visszolvasás
    uint8_t irq = 0;

    w_data_buffer[0] = SI1133_REG_IRQ_ENABLE;
    r_data_buffer[0] = 0;
    buffer_t.to_read = 1;
    buffer_t.to_write = 1;

    ret_code_t ret = D_I2CM_WriteRead(I2C0_INDEX,si_1133_target,&buffer_t);
    if(ret == RET_OK) {
        irq = r_data_buffer[0];
        printf("\n\n--------irqq ifben:------%d\n\n",irq);
    } else {
        r_data_buffer[0] = 0xFF;
    }
    printf("\n\n--------irqq:------%d\n\n",irq);
    */ 
  return true;
}

void reset() {
    vTaskDelay(100/portTICK_PERIOD_MS);

    w_data_buffer[0] = SI1133_REG_COMMAND;
    w_data_buffer[1] = SI1133_RESET_SW;

    buffer_t.to_write = 2;
    buffer_t.to_read = 0;

    D_I2CM_Write(I2C0_INDEX, si_1133_target, &buffer_t);

    vTaskDelay(100/portTICK_PERIOD_MS);
}

float get_light_level()
{
  float lux, uvi;
  measure_lux_uv(&lux, &uvi);
  return lux;
}

uint32_t measure_lux_uv(float *lux, float *uvi)
{
  SI1133_Samples_TypeDef samples;
  uint32_t retval = 0;

  vTaskDelay(100/portTICK_PERIOD_MS);

  w_data_buffer[0] = SI1133_REG_COMMAND;
  w_data_buffer[1] = SI1133_CMD_FORCE_CH;

  buffer_t.to_write = 2;
  buffer_t.to_read = 0;

  D_I2CM_Write(I2C0_INDEX, si_1133_target, &buffer_t);

  vTaskDelay(300/portTICK_PERIOD_MS);

  uint8_t response_stored = 0;

  w_data_buffer[0] = SI1133_REG_IRQ_STATUS;
  buffer_t.to_read = 1;
  buffer_t.to_write = 1;

  ret_code_t ret = D_I2CM_WriteRead(I2C0_INDEX,si_1133_target,&buffer_t);
  if(ret == RET_OK) {
      response_stored = r_data_buffer[0];
  } else {
      r_data_buffer[0] = 0xFF;
  }

    vTaskDelay(100/portTICK_PERIOD_MS);

  vTaskDelay(1000/portTICK_PERIOD_MS);
  retval = response_stored;
  int8_t c = 0; 
    while ( response_stored != 0x0f ) {
        if(c >= 100)
        {
            break;
        }
        vTaskDelay(100/portTICK_PERIOD_MS);
        w_data_buffer[0] = SI1133_REG_IRQ_STATUS;
        buffer_t.to_read = 1;
        buffer_t.to_write = 1;

        ret_code_t ret = D_I2CM_WriteRead(I2C0_INDEX,si_1133_target,&buffer_t);
        if(ret == RET_OK) {
            response_stored = r_data_buffer[0];
        } else {
            r_data_buffer[0] = 0xFF;
        }

        vTaskDelay(100/portTICK_PERIOD_MS);
        printf("\n\n----response: %d,%d------\n\n",response_stored,c);
        retval = response_stored;
        c++;
    }
  vTaskDelay(50/portTICK_PERIOD_MS);
  measure(&samples);

  /* Convert the readings to lux */
  *lux = (float) get_lux(samples.ch1, samples.ch3, samples.ch2);
  *lux = *lux / (1 << LUX_OUTPUT_FRACTION);
  //printf("Függvényben lux adat: %f\r\n", *lux);


  /* Convert the readings to UV index */
  *uvi = (float) get_uv(samples.ch0, uk);
  *uvi = *uvi / (1 << UV_OUTPUT_FRACTION);
  //printf("Függvényben a uvi adat: %f\r\n", *uvi);

  return retval;
}

uint32_t measure (SI1133_Samples_TypeDef *samples)
{
    uint32_t retval = 0;

    uint8_t out1 = 0;
    w_data_buffer[0] = SI1133_REG_HOSTOUT0;
    buffer_t.to_read = 1;
    buffer_t.to_write = 1;
    ret_code_t ret = D_I2CM_WriteRead(I2C0_INDEX,si_1133_target,&buffer_t);
    if(ret == RET_OK) {
        out1 = r_data_buffer[0];
    } else {
        r_data_buffer[0] = 0xFF;
    }

    uint8_t out2 = 0;
    w_data_buffer[0] = SI1133_REG_HOSTOUT1;
    buffer_t.to_read = 1;
    buffer_t.to_write = 1;
    ret = D_I2CM_WriteRead(I2C0_INDEX,si_1133_target,&buffer_t);
    if(ret == RET_OK) {
        out2 = r_data_buffer[0];
    } else {
        r_data_buffer[0] = 0xFF;
    }

    uint8_t out3 = 0;
    w_data_buffer[0] = SI1133_REG_HOSTOUT2;
    buffer_t.to_read = 1;
    buffer_t.to_write = 1;
    ret = D_I2CM_WriteRead(I2C0_INDEX,si_1133_target,&buffer_t);
    if(ret == RET_OK) {
        out3 = r_data_buffer[0];
    } else {
        r_data_buffer[0] = 0xFF;
    }

    uint8_t out4 = 0;
    w_data_buffer[0] = SI1133_REG_HOSTOUT3;
    buffer_t.to_read = 1;
    buffer_t.to_write = 1;
    ret = D_I2CM_WriteRead(I2C0_INDEX,si_1133_target,&buffer_t);
    if(ret == RET_OK) {
        out4 = r_data_buffer[0];
    } else {
        r_data_buffer[0] = 0xFF;
    }

    uint8_t out5 = 0;
    w_data_buffer[0] = SI1133_REG_HOSTOUT4;
    buffer_t.to_read = 1;
    buffer_t.to_write = 1;
    ret = D_I2CM_WriteRead(I2C0_INDEX,si_1133_target,&buffer_t);
    if(ret == RET_OK) {
        out5 = r_data_buffer[0];
    } else {
        r_data_buffer[0] = 0xFF;
    }

    uint8_t out6 = 0;
    w_data_buffer[0] = SI1133_REG_HOSTOUT5;
    buffer_t.to_read = 1;
    buffer_t.to_write = 1;
    ret = D_I2CM_WriteRead(I2C0_INDEX,si_1133_target,&buffer_t);
    if(ret == RET_OK) {
        out6 = r_data_buffer[0];
    } else {
        r_data_buffer[0] = 0xFF;
    }

    uint8_t out7 = 0;
    w_data_buffer[0] = SI1133_REG_HOSTOUT6;
    buffer_t.to_read = 1;
    buffer_t.to_write = 1;
    ret = D_I2CM_WriteRead(I2C0_INDEX,si_1133_target,&buffer_t);
    if(ret == RET_OK) {
        out7 = r_data_buffer[0];
    } else {
        r_data_buffer[0] = 0xFF;
    }

    uint8_t out8 = 0;
    w_data_buffer[0] = SI1133_REG_HOSTOUT7;
    buffer_t.to_read = 1;
    buffer_t.to_write = 1;
    ret = D_I2CM_WriteRead(I2C0_INDEX,si_1133_target,&buffer_t);
    if(ret == RET_OK) {
        out8 = r_data_buffer[0];
    } else {
        r_data_buffer[0] = 0xFF;
    }

    uint8_t out9 = 0;
    w_data_buffer[0] = SI1133_REG_HOSTOUT8;
    buffer_t.to_read = 1;
    buffer_t.to_write = 1;
    ret = D_I2CM_WriteRead(I2C0_INDEX,si_1133_target,&buffer_t);
    if(ret == RET_OK) {
        out9 = r_data_buffer[0];
    } else {
        r_data_buffer[0] = 0xFF;
    }

    uint8_t out10 = 0;
    w_data_buffer[0] = SI1133_REG_HOSTOUT9;
    buffer_t.to_read = 1;
    buffer_t.to_write = 1;
    ret = D_I2CM_WriteRead(I2C0_INDEX,si_1133_target,&buffer_t);
    if(ret == RET_OK) {
        out10 = r_data_buffer[0];
    } else {
        r_data_buffer[0] = 0xFF;
    }

    uint8_t out11 = 0;
    w_data_buffer[0] = SI1133_REG_HOSTOUT10;
    buffer_t.to_read = 1;
    buffer_t.to_write = 1;
    ret = D_I2CM_WriteRead(I2C0_INDEX,si_1133_target,&buffer_t);
    if(ret == RET_OK) {
        out11 = r_data_buffer[0];
    } else {
        r_data_buffer[0] = 0xFF;
    }

    uint8_t out12 = 0;
    w_data_buffer[0] = SI1133_REG_HOSTOUT11;
    buffer_t.to_read = 1;
    buffer_t.to_write = 1;
    ret = D_I2CM_WriteRead(I2C0_INDEX,si_1133_target,&buffer_t);
    if(ret == RET_OK) {
        out12 = r_data_buffer[0];
    } else {
        r_data_buffer[0] = 0xFF;
    }

    uint8_t out13 = 0;
    w_data_buffer[0] = SI1133_REG_HOSTOUT12;
    buffer_t.to_read = 1;
    buffer_t.to_write = 1;
    ret = D_I2CM_WriteRead(I2C0_INDEX,si_1133_target,&buffer_t);
    if(ret == RET_OK) {
        out13 = r_data_buffer[0];
    } else {
        r_data_buffer[0] = 0xFF;
    }


  samples->ch0 = out1 << 16;
  samples->ch0 |= out2 << 8;
  samples->ch0 |= out3;
  // xx|xx|xx|xx
  // xx|1000 0000|
  if ( samples->ch0 & 0x800000 ) {
    samples->ch0 |= 0xFF000000;
    //printf("Channel 0 ifben: ch0: %ld\r\n", samples->ch0);
  }
//printf("Channel 0: ch0: %ld\r\n", samples->ch0);
  

  samples->ch1 = out4 << 16;
  samples->ch1 |= out5 << 8;
  samples->ch1 |= out6;
  if ( samples->ch1 & 0x800000 ) {
    samples->ch1 |= 0xFF000000;
  }
  //printf("ch1: %lu\r\n", samples->ch1);

  samples->ch2 = out7 << 16;
  samples->ch2 |= out8 << 8;
  samples->ch2 |= out9;
  if ( samples->ch2 & 0x800000 ) {
    samples->ch2 |= 0xFF000000;
  }
  //printf("ch2: %lu\r\n", samples->ch2);


  samples->ch3 = out10 << 16;
  samples->ch3 |= out11 << 8;
  samples->ch3 |= out12;
  if ( samples->ch3 & 0x800000 ) {
    samples->ch3 |= 0xFF000000;
  }
  //printf("ch3: %lu\r\n",samples->ch3);


  return retval;
}

int32_t get_lux (int32_t vis_high, int32_t vis_low, int32_t ir)
{
  int32_t lux;
  if ( (vis_high > ADC_THRESHOLD) || (ir > ADC_THRESHOLD) ) {
        lux = calculate_polynomial(vis_high,
                                   ir,
                                   INPUT_FRACTION_HIGH,
                                   LUX_OUTPUT_FRACTION,
                                   NUMCOEFF_HIGH,
                                   &(lk.coeff_high[0]) );
    } else {
        lux = calculate_polynomial(vis_low,
                                   ir,
                                   INPUT_FRACTION_LOW,
                                   LUX_OUTPUT_FRACTION,
                                   NUMCOEFF_LOW,
                                   &(lk.coeff_low[0]) );
    }

    return lux;
}

int32_t get_uv (int32_t uv, SI1133_Coeff_TypeDef *uk)
{
  int32_t uvi;
  uvi = calculate_polynomial(0, uv, UV_INPUT_FRACTION, UV_OUTPUT_FRACTION, UV_NUMCOEFF, uk);

  return uvi;
}

float get_uv_index()
{
  float lux, uvi;
  measure_lux_uv(&lux, &uvi);
  return uvi;
}


int32_t calculate_polynomial_helper (int32_t input, int8_t fraction, uint16_t mag, int8_t shift)
{
    int32_t value;

    if ( shift < 0 ) {
        value = ( (input << fraction) / mag) >> -shift;
    } else {
        value = ( (input << fraction) / mag) << shift;
    }

    return value;
}

int32_t calculate_polynomial (int32_t x, int32_t y, uint8_t input_fraction, uint8_t output_fraction, uint8_t num_coeff, const SI1133_Coeff_TypeDef *kp)
{
    uint8_t info, x_order, y_order, counter;
    int8_t sign, shift;
    uint16_t mag;
    int32_t output = 0, x1, x2, y1, y2;

    for ( counter = 0; counter < num_coeff; counter++ ) {
        info = kp->info;
        x_order = get_x_order(info);
        y_order = get_y_order(info);

        shift = ( (uint16_t) kp->info & 0xff00) >> 8;
        shift ^= 0x00ff;
        shift += 1;
        shift = -shift;

        mag = kp->mag;

        if ( get_sign(info) ) {
            sign = -1;
        } else {
            sign = 1;
        }

        if ( (x_order == 0) && (y_order == 0) ) {
            output += sign * mag << output_fraction;
        } else {
            if ( x_order > 0 ) {
                x1 = calculate_polynomial_helper(x, input_fraction, mag, shift);
                if ( x_order > 1 ) {
                    x2 = calculate_polynomial_helper(x, input_fraction, mag, shift);
                } else {
                    x2 = 1;
                }
            } else {
                x1 = 1;
                x2 = 1;
            }

            if ( y_order > 0 ) {
                y1 = calculate_polynomial_helper(y, input_fraction, mag, shift);
                if ( y_order > 1 ) {
                    y2 = calculate_polynomial_helper(y, input_fraction, mag, shift);
                } else {
                    y2 = 1;
                }
            } else {
                y1 = 1;
                y2 = 1;
            }

            output += sign * x1 * x2 * y1 * y2;
        }

        kp++;
    }

    if ( output < 0 ) {
        output = -output;
    }

    return output;
}