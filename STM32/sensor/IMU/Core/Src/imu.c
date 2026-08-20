/**
  ******************************************************************************
  * @file    imu.c
  * @brief   WHEELTEC H30 IMU sensor driver implementation.
  *          Communicates via I2C1 to read sensor data registers.
  ******************************************************************************
  */

#include "imu.h"
#include "i2c.h"
#include <string.h>

imu_data_t g_imu = {0};

/* Internal helper: read signed 32-bit int from 4-byte little-endian buffer */
static inline int32_t buf_to_int32(const uint8_t *buf)
{
    return (int32_t)((buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0]);
}

/* Internal: read a register block from the IMU via I2C */
static HAL_StatusTypeDef IMU_ReadReg(uint8_t reg, uint8_t *data, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(IMU_I2C_ADDR << 1),
                            reg, I2C_MEMADD_SIZE_8BIT,
                            data, len, HAL_MAX_DELAY);
}

/**
  * @brief  Initialize IMU (currently just clears the data structure).
  *         The I2C bus must already be initialized by MX_I2C1_Init().
  */
void IMU_Init(void)
{
    memset(&g_imu, 0, sizeof(g_imu));
}

/**
  * @brief  Read Euler angles (pitch, roll, yaw) from IMU.
  * @param  data: pointer to imu_data_t to fill.
  * @retval 0 on success, -1 on I2C error.
  */
int IMU_ReadEuler(imu_data_t *data)
{
    uint8_t buf[IMU_EULER_DATA_LEN];

    if (IMU_ReadReg(IMU_REG_EULER, buf, IMU_EULER_DATA_LEN) != HAL_OK)
        return -1;

    data->attitude.pitch = (float)buf_to_int32(buf)      * IMU_DATA_FACTOR;
    data->attitude.roll  = (float)buf_to_int32(buf + 4)  * IMU_DATA_FACTOR;
    data->attitude.yaw   = (float)buf_to_int32(buf + 8)  * IMU_DATA_FACTOR;

    return 0;
}

/**
  * @brief  Read accelerometer data from IMU.
  * @param  data: pointer to imu_data_t to fill.
  * @retval 0 on success, -1 on I2C error.
  */
int IMU_ReadAccel(imu_data_t *data)
{
    uint8_t buf[IMU_ACCEL_DATA_LEN];

    if (IMU_ReadReg(IMU_REG_ACCEL, buf, IMU_ACCEL_DATA_LEN) != HAL_OK)
        return -1;

    data->accel.x = (float)buf_to_int32(buf)      * IMU_DATA_FACTOR;
    data->accel.y = (float)buf_to_int32(buf + 4)  * IMU_DATA_FACTOR;
    data->accel.z = (float)buf_to_int32(buf + 8)  * IMU_DATA_FACTOR;

    return 0;
}

/**
  * @brief  Read gyroscope data from IMU.
  * @param  data: pointer to imu_data_t to fill.
  * @retval 0 on success, -1 on I2C error.
  */
int IMU_ReadGyro(imu_data_t *data)
{
    uint8_t buf[IMU_GYRO_DATA_LEN];

    if (IMU_ReadReg(IMU_REG_GYRO, buf, IMU_GYRO_DATA_LEN) != HAL_OK)
        return -1;

    data->gyro.x = (float)buf_to_int32(buf)      * IMU_DATA_FACTOR;
    data->gyro.y = (float)buf_to_int32(buf + 4)  * IMU_DATA_FACTOR;
    data->gyro.z = (float)buf_to_int32(buf + 8)  * IMU_DATA_FACTOR;

    return 0;
}

/**
  * @brief  Read quaternion data from IMU.
  * @param  data: pointer to imu_data_t to fill.
  * @retval 0 on success, -1 on I2C error.
  */
int IMU_ReadQuaternion(imu_data_t *data)
{
    uint8_t buf[IMU_QUATERNION_DATA_LEN];

    if (IMU_ReadReg(IMU_REG_QUATERNION, buf, IMU_QUATERNION_DATA_LEN) != HAL_OK)
        return -1;

    data->attitude.q0 = (float)buf_to_int32(buf)       * IMU_DATA_FACTOR;
    data->attitude.q1 = (float)buf_to_int32(buf + 4)   * IMU_DATA_FACTOR;
    data->attitude.q2 = (float)buf_to_int32(buf + 8)   * IMU_DATA_FACTOR;
    data->attitude.q3 = (float)buf_to_int32(buf + 12)  * IMU_DATA_FACTOR;

    return 0;
}

/**
  * @brief  Read magnetometer data from IMU (normalized + raw).
  * @param  data: pointer to imu_data_t to fill.
  * @retval 0 on success, -1 on I2C error.
  */
int IMU_ReadMag(imu_data_t *data)
{
    uint8_t buf_norm[IMU_MAG_DATA_LEN];
    uint8_t buf_raw[IMU_MAG_DATA_LEN];

    if (IMU_ReadReg(IMU_REG_MAG_NORM, buf_norm, IMU_MAG_DATA_LEN) != HAL_OK)
        return -1;
    if (IMU_ReadReg(IMU_REG_MAG_RAW, buf_raw, IMU_MAG_DATA_LEN) != HAL_OK)
        return -1;

    data->mag_norm.x = (float)buf_to_int32(buf_norm)      * IMU_DATA_FACTOR;
    data->mag_norm.y = (float)buf_to_int32(buf_norm + 4)  * IMU_DATA_FACTOR;
    data->mag_norm.z = (float)buf_to_int32(buf_norm + 8)  * IMU_DATA_FACTOR;

    data->mag_raw.x = (float)buf_to_int32(buf_raw)      * IMU_MAG_RAW_FACTOR;
    data->mag_raw.y = (float)buf_to_int32(buf_raw + 4)  * IMU_MAG_RAW_FACTOR;
    data->mag_raw.z = (float)buf_to_int32(buf_raw + 8)  * IMU_MAG_RAW_FACTOR;

    return 0;
}

/**
  * @brief  Read all IMU sensor data at once.
  * @param  data: pointer to imu_data_t to fill with all sensor readings.
  * @retval 0 on success, -1 if any read failed.
  */
int IMU_ReadAll(imu_data_t *data)
{
    int ret = 0;

    ret |= IMU_ReadEuler(data);
    ret |= IMU_ReadAccel(data);
    ret |= IMU_ReadGyro(data);
    ret |= IMU_ReadQuaternion(data);

    return (ret != 0) ? -1 : 0;
}
