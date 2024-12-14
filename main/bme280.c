#include "bme280.h"
#include "driver/i2c.h"
#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"
#include "i2c_master.h"

static optn_io_buffer_t bme280_buffer;
smb_address_t bme280_target = BME280_ADDRESS;
uint8_t bme280_w_data_buffer[10] = {0};

uint8_t bme280_r_data_buffer[10] = {0};

uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;
uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;
uint8_t dig_H1;
int16_t dig_H2;
uint8_t dig_H3;
int16_t dig_H4;
int16_t dig_H5;
int8_t  dig_H6;
int32_t t_fine;

bool bme280_init(void){
    bme280_buffer.rbuffer_size = 10;
    bme280_buffer.wbuffer_size = 10;
    bme280_buffer.rdata = bme280_r_data_buffer;
    bme280_buffer.wdata = bme280_w_data_buffer;

    uint8_t chip_id = 0;
    bme280_w_data_buffer[0] = BME280_REG_CHIPID;
    bme280_buffer.to_read = 1;
    bme280_buffer.to_write = 1;

    D_I2CM_WriteRead(I2C0_INDEX,bme280_target,&bme280_buffer);
    chip_id = bme280_r_data_buffer[0];
    printf("\n\nbme280_cip_id: %d\n\n", chip_id);

    if (chip_id != 0x60){
        printf("Read Chip ID fail!");
        return false;
    }

    dig_T1 = bme280_Read16LE(BME280_REG_DIG_T1);
    dig_T2 = bme280_ReadS16LE(BME280_REG_DIG_T2);
    dig_T3 = bme280_ReadS16LE(BME280_REG_DIG_T3);

    dig_P1 = bme280_Read16LE(BME280_REG_DIG_P1);
    dig_P2 = bme280_ReadS16LE(BME280_REG_DIG_P2);
    dig_P3 = bme280_ReadS16LE(BME280_REG_DIG_P3);
    dig_P4 = bme280_ReadS16LE(BME280_REG_DIG_P4);
    dig_P5 = bme280_ReadS16LE(BME280_REG_DIG_P5);
    dig_P6 = bme280_ReadS16LE(BME280_REG_DIG_P6);
    dig_P7 = bme280_ReadS16LE(BME280_REG_DIG_P7);
    dig_P8 = bme280_ReadS16LE(BME280_REG_DIG_P8);
    dig_P9 = bme280_ReadS16LE(BME280_REG_DIG_P9);

    dig_H1 = bme280_Read8(BME280_REG_DIG_H1);
    dig_H2 = bme280_Read16LE(BME280_REG_DIG_H2);
    dig_H3 = bme280_Read8(BME280_REG_DIG_H3);
    dig_H4 = (bme280_Read8(BME280_REG_DIG_H4) << 4) | (0x0F & bme280_Read8(BME280_REG_DIG_H4 + 1));
    dig_H5 = (bme280_Read8(BME280_REG_DIG_H5 + 1) << 4) | (0x0F & bme280_Read8(BME280_REG_DIG_H5) >> 4);
    dig_H6 = (int8_t)bme280_Read8(BME280_REG_DIG_H6);

    printf("dig_T1: %d\n", dig_T1);
    printf("dig_T2: %d\n", dig_T2);
    printf("dig_T3: %d\n", dig_T3);


    bme280_w_data_buffer[0] = BME280_REG_CONTROLHUMID; //16X oversampling
    bme280_w_data_buffer[1] = 0x05;

    bme280_buffer.to_write = 2;
    bme280_buffer.to_read = 0;
    D_I2CM_Write(I2C0_INDEX, bme280_target, &bme280_buffer);

    bme280_w_data_buffer[0] = BME280_REG_CONTROL; //16X oversampling
    bme280_w_data_buffer[1] = 0xB7;

    bme280_buffer.to_write = 2;
    bme280_buffer.to_read = 0;
    D_I2CM_Write(I2C0_INDEX, bme280_target, &bme280_buffer);

    return true;

}

float bme280_getTemperature(void) {
    int32_t var1, var2;

    int32_t adc_T = bme280_Read24(BME280_REG_TEMPDATA);
    
    adc_T >>= 4;
    var1 = (((adc_T >> 3) - ((int32_t)(dig_T1 << 1))) *
            ((int32_t)dig_T2)) >> 11;

    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) *
              ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
            ((int32_t)dig_T3)) >> 14;

    t_fine = var1 + var2;
    float T = (t_fine * 5 + 128) >> 8;
    return T / 100;
}


float bme280_getPressure(void) {
  int64_t var1, var2, p;

  bme280_getTemperature();
  
  int32_t adc_P = bme280_Read24(BME280_REG_PRESSUREDATA);
  adc_P >>= 4;
  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)dig_P6;
  var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
  var2 = var2 + (((int64_t)dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
  if (var1 == 0) {
    return 0; // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
  return (float)(p / 256.0);
}

float bme280_getHumidity(void) {
  int32_t v_x1_u32r, adc_H;
  // Call getTemperature to get t_fine
  bme280_getTemperature();

  adc_H = bme280_Read16(BME280_REG_HUMIDITYDATA);
  v_x1_u32r = (t_fine - ((int32_t)76800));
  v_x1_u32r = (
                ((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) +
                ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) *
                (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
                ((int32_t)dig_H2) + 8192) >> 14));
  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
  v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
  v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
  v_x1_u32r = v_x1_u32r >> 12;
  float h = v_x1_u32r / 1024.0;
  return h;
}

float bme280_calcAltitude(float pressure) {
    float A = pressure / 101325;
    float B = 1 / 5.25588;
    float C = pow(A, B);
    C = 1.0 - C;
    C = C / 0.0000225577;
    return C;
}

uint8_t bme280_Read8(uint8_t reg)
{
    ret_code_t ret;
    uint8_t response = 0;
    
    bme280_w_data_buffer[0] = reg;
    bme280_buffer.to_read = 1;
    bme280_buffer.to_write = 1;

    ret = D_I2CM_WriteRead(I2C0_INDEX, bme280_target, &bme280_buffer);
    if (ret == RET_OK) {
        response = bme280_r_data_buffer[0];
    }
    return response;
}

// uint16_t bme280_Read16(uint8_t reg){
//     uint8_t msb,lsb;
//     uint16_t response = 0;
//     ret_code_t ret;
//     bme280_w_data_buffer[0] = reg;
//     bme280_buffer.to_read = 2;
//     bme280_buffer.to_write = 1;

//     ret = D_I2CM_WriteRead(I2C0_INDEX, bme280_target, &bme280_buffer);
//     if (ret == RET_OK) {

//         response = (uint16_t)(bme280_r_data_buffer[1] << 8 | bme280_r_data_buffer[0]);
//     }
//     return response;
// }
uint16_t bme280_Read16(uint8_t reg){
    uint8_t msb = 0;
    uint8_t lsb = 0;
    uint16_t response = 0;
    ret_code_t ret;
    bme280_w_data_buffer[0] = reg;
    bme280_buffer.to_read = 2;
    bme280_buffer.to_write = 1;

    ret = D_I2CM_WriteRead(I2C0_INDEX, bme280_target, &bme280_buffer);
    if (ret == RET_OK) {
        msb = bme280_r_data_buffer[0];
        lsb = bme280_r_data_buffer[1];
    }
    return (uint16_t) msb << 8 | lsb;
}
uint16_t bme280_Read16LE(uint8_t reg){
    uint16_t data = bme280_Read16(reg);
    return (data >> 8) | (data << 8);
}


// uint16_t bme280_Read16LE(uint8_t reg)
// {
//     ret_code_t ret;
//     uint16_t response = 0;

//      //Írás: regiszter cím elküldése
//     bme280_w_data_buffer[0] = reg;
//     bme280_buffer.to_write = 1;

//      //Olvasás: 16 bites adat fogadása
//     bme280_buffer.to_read = 2;

//     ret = D_I2CM_WriteRead(I2C0_INDEX, bme280_target, &bme280_buffer);
//     if (ret == RET_OK) {
//         Nagyvégű rendszerű dekódolás
//         response = (uint16_t)(bme280_r_data_buffer[0] << 8 | bme280_r_data_buffer[1]);
//     }
//     return response;
// }

uint32_t bme280_Read24(uint8_t reg)
{
    ret_code_t ret;
    uint32_t response = 0;

    // Írás: regiszter cím elküldése
    bme280_w_data_buffer[0] = reg;
    bme280_buffer.to_write = 1;

    // Olvasás: 24 bites adat fogadása
    bme280_buffer.to_read = 3;

    ret = D_I2CM_WriteRead(I2C0_INDEX, bme280_target, &bme280_buffer);
    if (ret == RET_OK) {
        // Nagyvégű rendszerű dekódolás
        response = (uint32_t)(bme280_r_data_buffer[0] << 16 | bme280_r_data_buffer[1] << 8 | bme280_r_data_buffer[2]);
    }
    return response;
}

int16_t bme280_ReadS16(uint8_t reg) {
    return (int16_t)bme280_Read16(reg);
}

int16_t bme280_ReadS16LE(uint8_t reg) {
    return (int16_t)bme280_Read16LE(reg);
}