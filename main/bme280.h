#ifndef _SEEED_BME280_H_
#define _SEEED_BME280_H_

#include <stdio.h>
#include <stdbool.h>

#define BME280_ADDRESS   0x76

#define BME280_REG_DIG_T1    0x88
#define BME280_REG_DIG_T2    0x8A
#define BME280_REG_DIG_T3    0x8C

#define BME280_REG_DIG_P1    0x8E
#define BME280_REG_DIG_P2    0x90
#define BME280_REG_DIG_P3    0x92
#define BME280_REG_DIG_P4    0x94
#define BME280_REG_DIG_P5    0x96
#define BME280_REG_DIG_P6    0x98
#define BME280_REG_DIG_P7    0x9A
#define BME280_REG_DIG_P8    0x9C
#define BME280_REG_DIG_P9    0x9E

#define BME280_REG_DIG_H1    0xA1
#define BME280_REG_DIG_H2    0xE1
#define BME280_REG_DIG_H3    0xE3
#define BME280_REG_DIG_H4    0xE4
#define BME280_REG_DIG_H5    0xE5
#define BME280_REG_DIG_H6    0xE7

#define BME280_REG_CHIPID          0xD0
#define BME280_REG_VERSION         0xD1
#define BME280_REG_SOFTRESET       0xE0

#define BME280_REG_CAL26           0xE1

#define BME280_REG_CONTROLHUMID    0xF2
#define BME280_REG_CONTROL         0xF4
#define BME280_REG_CONFIG          0xF5
#define BME280_REG_PRESSUREDATA    0xF7
#define BME280_REG_TEMPDATA        0xFA
#define BME280_REG_HUMIDITYDATA    0xFD

bool bme280_init(void);
float bme280_getTemperature(void);
float bme280_getPressure(void);
float bme280_getHumidity(void);
float bme280_calcAltitude(float pressure);

extern int _devAddr;
extern bool isTransport_OK;

// Calibration data
extern uint16_t dig_T1;
extern int16_t dig_T2;
extern int16_t dig_T3;
extern uint16_t dig_P1;
extern int16_t dig_P2;
extern int16_t dig_P3;
extern int16_t dig_P4;
extern int16_t dig_P5;
extern int16_t dig_P6;
extern int16_t dig_P7;
extern int16_t dig_P8;
extern int16_t dig_P9;
extern uint8_t dig_H1;
extern int16_t dig_H2;
extern uint8_t dig_H3;
extern int16_t dig_H4;
extern int16_t dig_H5;
extern int8_t  dig_H6;
extern int32_t t_fine;

// private functions
uint8_t bme280_Read8(uint8_t reg);
uint16_t bme280_Read16(uint8_t reg);
uint16_t bme280_Read16LE(uint8_t reg);
int16_t bme280_ReadS16(uint8_t reg);
int16_t bme280_ReadS16LE(uint8_t reg);
uint32_t bme280_Read24(uint8_t reg);
void writeRegister(uint8_t reg, uint8_t val);


#endif