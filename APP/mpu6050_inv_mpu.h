/*
 $License:
    Copyright (C) 2011-2012 InvenSense Corporation, All Rights Reserved.
    See included License.txt for License information.
 $
 */
/**
 *  @addtogroup  DRIVERS Sensor Driver Layer
 *  @brief       Hardware drivers to communicate with sensors via I2C.
 *
 *  @{
 *      @file       inv_mpu.h
 *      @brief      An I2C-based driver for Invensense MPU6050 gyroscope.
 */

#ifndef MPU6050_INV_MPU_H
#define MPU6050_INV_MPU_H

#include "bsp_system.h"

#define DEFAULT_MPU_HZ  (100)

#define INV_X_GYRO      (0x40)
#define INV_Y_GYRO      (0x20)
#define INV_Z_GYRO      (0x10)
#define INV_XYZ_GYRO    (INV_X_GYRO | INV_Y_GYRO | INV_Z_GYRO)
#define INV_XYZ_ACCEL   (0x08)
#define INV_XYZ_COMPASS (0x01)

struct int_param_s {
    void *arg;
};

/* Set up APIs */
int mpu_init(struct int_param_s *int_param);

/* Configuration APIs */
int mpu_set_dmp_state(unsigned char enable);
int mpu_set_lpf(unsigned short lpf);
int mpu_set_gyro_fsr(unsigned short fsr);
int mpu_set_accel_fsr(unsigned char fsr);
int mpu_set_sample_rate(unsigned short rate);
int mpu_configure_fifo(unsigned char sensors);
int mpu_set_sensors(unsigned char sensors);

/* Sensitivity APIs */
int mpu_get_gyro_sens(float *sens);
int mpu_get_accel_sens(unsigned short *sens);
int mpu_get_accel_fsr(unsigned char *fsr);

/* FIFO / DMP memory APIs */
int mpu_reset_fifo(void);
int mpu_read_fifo_stream(unsigned short length, unsigned char *data,
    unsigned char *more);
int mpu_write_mem(unsigned short mem_addr, unsigned short length,
    unsigned char *data);
int mpu_read_mem(unsigned short mem_addr, unsigned short length,
    unsigned char *data);
int mpu_load_firmware(unsigned short length, const unsigned char *firmware,
    unsigned short start_addr, unsigned short sample_rate);

/* Self-test API */
int mpu_run_self_test(long *gyro, long *accel);

#endif /* MPU6050_INV_MPU_H */
