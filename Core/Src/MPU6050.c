/*
 * MPU6050.h
 *
 *  Created on: May 24, 2023
 *      Author: Tan
 */

#include "stm32f1xx_hal.h"
#include "MPU6050.h"
#include "math.h"

extern I2C_HandleTypeDef hi2c2;

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

#define timeOut  100

float rad_to_deg = 180/3.141592654;

//Accel variables
int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;
float Ax, Ay, Az;
float Ax_error,Ay_error,Az_error;
float Acc_angle_x,Acc_angle_y;//deviation ang

//Gyro variables
int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;
float Gx,Gy,Gz;
float Gx_error,Gy_error,Gz_error;
float Gyro_angle_x, Gyro_angle_y;

float angle_pitch,angle_roll;

void MPU6050_Init (void)
{
	uint8_t check;
	uint8_t Data;

	// check device ID WHO_AM_I

	HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);

	if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ? 2g
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ? 250 ?/s
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
	}

}

void Calculation_Acc_error(void){
	uint8_t Rec_Data[6];
	int i=0;
	while (i<200){
		// Read 6 BYTES of data starting from ACCEL_XOUT_H register

		HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

		Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
		Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
		Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

		/*** convert the RAW values into acceleration in 'g'
				 we have to divide according to the Full scale value set in FS_SEL
				 I have configured FS_SEL = 0. So I am dividing by 16384.0
				 for more details check ACCEL_CONFIG Register              ****/

		Ax_error += Accel_X_RAW/16384.0;
		Ay_error += Accel_Y_RAW/16384.0;
		Az_error += Accel_Z_RAW/16384.0;
		i++;
	}
	Ax_error = Ax_error/200;
	Ay_error = Ay_error/200;
	Az_error = Az_error/200;
}

void Calculation_Gyro_error(void){
	uint8_t Rec_Data[6];
	int i=0;
	// Read 6 BYTES of data starting from GYRO_XOUT_H register
	while(i<200){
		HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

		Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
		Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
		Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

		/*** convert the RAW values into dps (?/s)
				 we have to divide according to the Full scale value set in FS_SEL
				 I have configured FS_SEL = 0. So I am dividing by 131.0
				 for more details check GYRO_CONFIG Register              ****/

		Gx_error += Gyro_X_RAW/131.0;
		Gy_error += Gyro_Y_RAW/131.0;
		Gz_error += Gyro_Z_RAW/131.0;
		i++;
	}
	Gx_error = Gx_error/200;
	Gy_error = Gy_error/200;
	Gz_error = Gz_error/200;
}


void MPU6050_Read_Accel (void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register              ****/

	Ax = Accel_X_RAW/16384.0-Ax_error;
	Ay = Accel_Y_RAW/16384.0-Ay_error;
	Az = Accel_Z_RAW/16384.0-Az_error;  

}


void MPU6050_Read_Gyro (void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from GYRO_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into dps (?/s)
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 131.0
	     for more details check GYRO_CONFIG Register              ****/

	Gx = Gyro_X_RAW/131.0-Gx_error;
	Gy = Gyro_Y_RAW/131.0-Gy_error;
	Gz = Gyro_Z_RAW/131.0-Gz_error;
	
	angle_pitch += Gx/250;
	angle_roll += Gy/250;
	
	angle_pitch -= angle_roll * sin(Gz / 250 *3.142/180);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin(Gz / 250 *3.142/180);                  //If the IMU has yawed transfer the pitch angle to the roll angel.

}

