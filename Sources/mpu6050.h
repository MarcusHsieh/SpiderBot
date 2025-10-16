#ifndef SOURCES_MPU6050_H_
#define SOURCES_MPU6050_H_

#include "PE_Types.h"

#define MPU6050_I2C_ADDR 0x68

#define REG_ACCEL_XOUT_H    0x3B
#define REG_GYRO_XOUT_H     0x43
#define REG_PWR_MGMT_1      0x6B
#define REG_WHO_AM_I        0x75
#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C

#define MPU_SUCCESS 0
#define MPU_FAIL 1

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} SRAWDATA_t;

typedef struct {
	SRAWDATA_t accel;
	SRAWDATA_t gyro;
} SensorDataRaw_t;

uint8_t mpu6050_init(void);
uint8_t mpu6050_read_who_am_i(uint8_t *whoami);
void mpu6050_read_raw_data(SensorDataRaw_t* data);

#endif /* SOURCES_MPU6050_H_ */ 