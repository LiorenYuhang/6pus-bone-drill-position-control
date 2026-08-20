/**
  ******************************************************************************
  * @file    imu.h
  * @brief   WHEELTEC H30 IMU sensor driver header.
  *          Supports I2C communication with the H30 inertial navigation module.
  *
  *          I2C Slave Address: 0x6A (7-bit)
  *
  *          Register Map:
  *            0x10 - Accelerometer data (12 bytes, 3x int32)
  *            0x20 - Gyroscope data (12 bytes, 3x int32)
  *            0x30 - Magnetometer normalized (12 bytes, 3x int32)
  *            0x31 - Magnetometer raw (12 bytes, 3x int32)
  *            0x40 - Euler angles: pitch, roll, yaw (12 bytes, 3x int32)
  *            0x41 - Quaternion data (16 bytes, 4x int32)
  *
  *          All int32 values are multiplied by 1e-6 to get floats.
  ******************************************************************************
  */

#ifndef __IMU_H
#define __IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

/* I2C address and register definitions */
#define IMU_I2C_ADDR               ((uint8_t)0x6A)

#define IMU_REG_ACCEL              ((uint8_t)0x10)
#define IMU_REG_GYRO               ((uint8_t)0x20)
#define IMU_REG_MAG_NORM           ((uint8_t)0x30)
#define IMU_REG_MAG_RAW            ((uint8_t)0x31)
#define IMU_REG_EULER              ((uint8_t)0x40)
#define IMU_REG_QUATERNION         ((uint8_t)0x41)

/* Data lengths for each register */
#define IMU_ACCEL_DATA_LEN         12U
#define IMU_GYRO_DATA_LEN          12U
#define IMU_MAG_DATA_LEN           12U
#define IMU_EULER_DATA_LEN         12U
#define IMU_QUATERNION_DATA_LEN    16U

/* Conversion factor: raw int32 value * FACTOR = float value */
#define IMU_DATA_FACTOR            0.000001f
#define IMU_MAG_RAW_FACTOR         0.001f

/**
 * @brief 3-axis data structure (common for accel, gyro, mag)
 */
typedef struct {
    float x;
    float y;
    float z;
} imu_axis_data_t;

/**
 * @brief Attitude data (Euler angles + quaternion)
 */
typedef struct {
    float pitch;   /* deg */
    float roll;    /* deg */
    float yaw;     /* deg */
    float q0;      /* quaternion w */
    float q1;      /* quaternion x */
    float q2;      /* quaternion y */
    float q3;      /* quaternion z */
} imu_attitude_t;

/**
 * @brief Complete IMU sensor data structure
 */
typedef struct {
    imu_axis_data_t accel;       /* m/s^2 */
    imu_axis_data_t gyro;        /* deg/s */
    imu_axis_data_t mag_norm;    /* normalized, unitless */
    imu_axis_data_t mag_raw;     /* mGauss */
    imu_attitude_t  attitude;
} imu_data_t;

/* Global IMU data instance */
extern imu_data_t g_imu;

/* Function prototypes */
void IMU_Init(void);
int  IMU_ReadAll(imu_data_t *data);
int  IMU_ReadEuler(imu_data_t *data);
int  IMU_ReadAccel(imu_data_t *data);
int  IMU_ReadGyro(imu_data_t *data);
int  IMU_ReadQuaternion(imu_data_t *data);
int  IMU_ReadMag(imu_data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_H */
