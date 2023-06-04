/*
 * MPU6050.h
 *
 *  Created on: May 24, 2023
 *      Author: Tan
 */

#ifndef MPU6050_H_

#include "stm32f1xx_hal.h"

#define MPU6050_H_

#define MPU6050_ADDR 0xD0


#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

#define ACCEL_CONFIG_REG_2 0x1D
#define CONFIG_REG 0x1A

void MPU6050_Init (void);
void Calculation_Acc_error(void);
void Calculation_Gyro_error(void);
void MPU6050_Read_Accel(void);
void MPU6050_Read_Gyro(void);



#endif /* MPU6050_H_ */
