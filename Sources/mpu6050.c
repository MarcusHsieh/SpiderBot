#include "mpu6050.h"
#include "WAIT1.h"
#include "GI2C1.h"

static uint8_t write_reg(uint8_t reg, uint8_t val);
static uint8_t read_regs(uint8_t reg, uint8_t *buffer, uint8_t len);

uint8_t mpu6050_init(void) {
    //wake up!!
    if (write_reg(REG_PWR_MGMT_1, 0x00) != ERR_OK) {
        return MPU_FAIL;
    }
    WAIT1_Waitms(100);

    //sample rate divider
    if (write_reg(REG_SMPLRT_DIV, 0x07) != ERR_OK) { //1khz / (1 + 7) = 125hz
        return MPU_FAIL;
    }
    
    //gyro and dlpf config
    if (write_reg(REG_CONFIG, 0x03) != ERR_OK) { //fsync disabled, dlpf_cfg = 3
		return MPU_FAIL;
	}

    //gyro range
    if (write_reg(REG_GYRO_CONFIG, 0x00) != ERR_OK) { //+/- 250 dps
        return MPU_FAIL;
    }

    //accel range
    if (write_reg(REG_ACCEL_CONFIG, 0x00) != ERR_OK) { //+/- 2g
        return MPU_FAIL;
    }

    return MPU_SUCCESS;
}

uint8_t mpu6050_read_who_am_i(uint8_t *whoami) {
    if (read_regs(REG_WHO_AM_I, whoami, 1) != ERR_OK) {
        return MPU_FAIL;
    }
    return MPU_SUCCESS;
}

void mpu6050_read_raw_data(SensorDataRaw_t* data) {
    uint8_t buffer[14];

    if (read_regs(REG_ACCEL_XOUT_H, buffer, 14) != ERR_OK) {
        data->accel.x = 0;
        data->accel.y = 0;
        data->accel.z = 0;
        data->gyro.x = 0;
        data->gyro.y = 0;
        data->gyro.z = 0;
        return;
    }

    data->accel.x = (int16_t)(buffer[0] << 8 | buffer[1]);
    data->accel.y = (int16_t)(buffer[2] << 8 | buffer[3]);
    data->accel.z = (int16_t)(buffer[4] << 8 | buffer[5]);
    data->gyro.x = (int16_t)(buffer[8] << 8 | buffer[9]);
    data->gyro.y = (int16_t)(buffer[10] << 8 | buffer[11]);
    data->gyro.z = (int16_t)(buffer[12] << 8 | buffer[13]);
}

static uint8_t write_reg(uint8_t reg, uint8_t val) {
    return GI2C1_WriteByteAddress8(MPU6050_I2C_ADDR, reg, val);
}

static uint8_t read_regs(uint8_t reg, uint8_t *buffer, uint8_t len) {
    return GI2C1_ReadAddress(MPU6050_I2C_ADDR, &reg, 1, buffer, len);
} 