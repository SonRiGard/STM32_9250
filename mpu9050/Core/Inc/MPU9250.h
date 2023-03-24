/*
 * MPU9250.h
 *
 *  Created on: 30 thg 4, 2020
 *      Author: PC
 */

#ifndef INC_IMU_MPU9250_H_
#define INC_IMU_MPU9250_H_

#include "stdbool.h"
#include "stm32f1xx_hal.h"

#define SAMPLE_RATE_1khz     7
#define gyro_sensitivity    131.0// 65.5//131.0   // =  LSB/degrees/sec
#define accel_sensitivity  16384.0//8192.0//16384.0      // =  LSB/g
#define mag_sensitivity    2.56 // Divide raw data by mag_sensitivity to change uT -> mG      raw_Data/(10*4912/32760)
#define scale_mag		 0.1499389499									   // 1 Micr�tesla [�T] =   10 Miligauss [mG]
#define alpha           0.99
#define RAD2DEC			57.29577951

#define MAX_PRECISION	(10)


typedef enum{
	MPU9250_CLOCK_INTERNAL        = 0 <<0 ,
	MPU9250_CLOCK_PLL_XGYRO       = 1 <<0,
	MPU9250_CLOCK_PLL_YGYRO       = 2 <<0,
	MPU9250_CLOCK_PLL_ZGYRO       = 3 <<0,
	MPU9250_CLOCK_PLL_EXT32K      = 4 <<0,
	MPU9250_CLOCK_PLL_EXT19M      = 5 <<0,
	MPU9250_CLOCK_KEEP_RESET      = 7 <<0,
}MPU9250_clock_source_t;

typedef enum{
	MPU9250_GYRO_FS_250       =  0 << 3,  //0x00   // � 250 �/s
	MPU9250_GYRO_FS_500       =  1 << 3,// 0x08   // � 500 �/s
	MPU9250_GYRO_FS_1000      =  2 << 3,// 0x10   // � 1000 �/s
	MPU9250_GYRO_FS_2000      =  3 << 3,// 0x18	  // � 2000 �/s
}MPU9250_GYRO_FULL_SCALE;

typedef enum{
	MPU9250_ACCEL_FS_2        =  0 <<3,//  0x00   // � 2g
	MPU9250_ACCEL_FS_4        =  1 << 3,// 0x08   // � 4g
	MPU9250_ACCEL_FS_8        =  2 << 3,// 0x10   // � 8g
	MPU9250_ACCEL_FS_16       =  3 << 3,// 0x18  // � 16g
}MPU9250_ACCEL_FULL_SCALE;
typedef enum{
	MFS_14BITS	= 0,
	MFS_16BITS	= 1,
}MPU9250_MAG_FULL_SCALE;

typedef enum  {
	Result_Ok = 0x00,          /*!< Everything OK */
	Result_Error,              /*!< Unknown error */
	Result_DeviceNotConnected, /*!< There is no device with valid slave address */
	Result_DeviceInvalid       /*!< Connected device with address is not MPU6050 */
}Result;

void init_IMU();
void init_magnetometer();
void Process_IMU();
void Set_Accel_Range(MPU9250_ACCEL_FULL_SCALE accel_FS);
void Set_Gyro_Range(MPU9250_GYRO_FULL_SCALE gyro_FS);
int Check_Connection(uint8_t return_true_val);
void mpu9250_set_clock_source(MPU9250_clock_source_t clock_source);
void Calibration_IMU();
void Get_magnetometer();
void Calib_magnetometer();
void Reset_MPU();
char * ftoa(double f, char * buf, int precision);
void Complementary_filter(float Gyro_x,float Gyro_y,float Gyro_z,float  Acc_x,float Acc_y,float Acc_z, float dt);
void Quaternion_to_EulerAngle(float w,float x,float y,float z);

#endif /* INC_IMU_MPU9250_H_ */
