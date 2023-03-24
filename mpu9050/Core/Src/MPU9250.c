/*
 * MPU9250.c
 *
 *  Created on: 30 thg 4, 2020
 *      Author: PC
 */

#include "MPU9250.h"
#include "MPU9250_register.h"
#include "Madgwick.h"
#include "kalman.h"
#include "stdbool.h"
#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "HMC5883L.h"



extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim3;

static const double rounders[MAX_PRECISION + 1] =
{
	0.5,				// 0
	0.05,				// 1
	0.005,				// 2
	0.0005,				// 3
	0.00005,			// 4
	0.000005,			// 5
	0.0000005,			// 6
	0.00000005,			// 7
	0.000000005,		// 8
	0.0000000005,		// 9
	0.00000000005		// 10
};

float roll,yaw,pitch;
uint8_t temp;
char buff;

int16_t Mag_x,Mag_y,Mag_z;

int16_t Accel_x,Accel_y,Accel_z,Gyro_x,Gyro_y,Gyro_z;
int32_t Accel_x_bias,Accel_y_bias,Accel_z_bias,Gyro_x_bias,Gyro_y_bias,Gyro_z_bias;
float asax,asay,asaz;
float mag_offset[3] ;
float Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z;
float Mag_X_calib,Mag_Y_calib,Mag_Z_calib;
int32_t mag_bias[3] , mag_scale[3];
float scale_x ,scale_y,scale_z;

void Calib_magnetometer();

extern float y ,new_yaw;
extern float roll_kal,pitch_kal,yaw_kal;

char * ftoa(double f, char * buf, int precision)
{
	char * ptr = buf;
	char * p = ptr;
	char * p1;
	char c;
	long intPart;

	// check precision bounds
	if (precision > MAX_PRECISION)
		precision = MAX_PRECISION;

	// sign stuff
	if (f < 0)
	{
		f = -f;
		*ptr++ = '-';
	}

	if (precision < 0)  // negative precision == automatic precision guess
	{
		if (f < 1.0) precision = 6;
		else if (f < 10.0) precision = 5;
		else if (f < 100.0) precision = 4;
		else if (f < 1000.0) precision = 3;
		else if (f < 10000.0) precision = 2;
		else if (f < 100000.0) precision = 1;
		else precision = 0;
	}

	// round value according the precision
	if (precision)
		f += rounders[precision];

	// integer part...
	intPart = f;
	f -= intPart;

	if (!intPart)
		*ptr++ = '0';
	else
	{
		// save start pointer
		p = ptr;

		// convert (reverse order)
		while (intPart)
		{
			*p++ = '0' + intPart % 10;
			intPart /= 10;
		}

		// save end pos
		p1 = p;

		// reverse result
		while (p > ptr)
		{
			c = *--p;
			*p = *ptr;
			*ptr++ = c;
		}

		// restore end pos
		ptr = p1;
	}

	// decimal part
	if (precision)
	{
		// place decimal point
		*ptr++ = '.';

		// convert
		while (precision--)
		{
			f *= 10.0;
			c = f;
			*ptr++ = '0' + c;
			f -= c;
		}
	}

	// terminating zero
	*ptr = 0;

	return buf;
}

void init_IMU()
{

	uint8_t mpu_address = MPU9250_ADDRESS_DEFAULT;
	uint8_t d[2];
	/* Check if device is connected */
	while(HAL_I2C_IsDeviceReady(&hi2c1,mpu_address,2,3) != HAL_OK);

	/* Wakeup MPU6050 */
	/* Try to transmit via I2C */
	d[0] = PWR_MGMT_1;
	d[1] = 0x00;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mpu_address,(uint8_t *)d,2,100) != HAL_OK);
	HAL_Delay(100);

	// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	mpu9250_set_clock_source(MPU9250_CLOCK_PLL_XGYRO);

	//CONGFIG
	d[0] = CONFIG;
	d[1] = 0x09;//0x05//0x03
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mpu_address,(uint8_t *)d,2,100) != HAL_OK);

	/* Set data sample rate */
	d[0] = SMPLRT_DIV;   // sample rate = SAMPLE_RATE/(1 + 7) = 1khz
	d[1] = SAMPLE_RATE_1khz;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mpu_address,(uint8_t *)d,2,100) != HAL_OK);
	// config accelerometer
	Set_Accel_Range(MPU9250_ACCEL_FS_2);
	// config gyro
	Set_Gyro_Range(MPU9250_GYRO_FS_250);
	d[0] = USER_CTRL;
	d[1] = 0x00;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mpu_address,(uint8_t *)d,2,100) != HAL_OK);
	d[0] = INT_ENABLE;
	d[1] = 0x01;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mpu_address,(uint8_t *)d,2,1000)!=HAL_OK);
	d[0] = INT_PIN_CFG;
	d[1] = 0x22;  // turn on AK8963
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mpu_address,(uint8_t *)d,2,1000)!=HAL_OK);
	HAL_Delay(300);
}
void init_magnetometer()
{
	uint8_t Data;
		
	Data = 0x70;//0 (8-average, 15 Hz default, normal measurement)
	HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDRESS , REGISTER_CONFIG_A , 1, &Data , 1,100);
	
	Data = 0xA0;//0 (8-average, 15 Hz default, normal measurement) gain = 5; Lsb/gauss =390; MGl/LSB = 2.56 ; Outputdata Range (0xF800->0x07FF) 
	HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDRESS , REGISTER_CONFIG_B , 1, &Data , 1,100);
	
	//Continuous-Measurement Mode
	Data = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDRESS , REGISTER_MODE , 1, &Data , 1,100);
}


void Process_IMU()
{
	//read raw data
	uint8_t data[14];
	uint8_t reg = ACCEL_XOUT_H;
	uint8_t mpu_address = MPU9250_ADDRESS_DEFAULT;


	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mpu_address,&reg,1,1000) != HAL_OK);
	while(HAL_I2C_Master_Receive(&hi2c1, (uint16_t)mpu_address, data, 14, 1000) != HAL_OK);

	/*-------- Accel ---------*/
	Accel_x = (int16_t)((int16_t)( data[0] << 8 ) | data[1]);
	Accel_y = (int16_t)((int16_t)( data[2] << 8 ) | data[3]);
	Accel_z = (int16_t)((int16_t)( data[4] << 8 ) | data[5]);

	/*-------- Gyrometer --------*/
	Gyro_x = (int16_t)((int16_t)( data[8] << 8  ) | data[9]);
	Gyro_y = (int16_t)((int16_t)( data[10] << 8 ) | data[11]);
	Gyro_z = (int16_t)((int16_t)( data[12] << 8 ) | data[13]);


	Accel_X = 10*(float)((int32_t)Accel_x - Accel_x_bias)/(float)accel_sensitivity;
	Accel_Y = 10*(float)((int32_t)Accel_y - Accel_y_bias)/(float)accel_sensitivity;
	Accel_Z =  10*(float)((int32_t)Accel_z - Accel_z_bias)/(float)accel_sensitivity ;

	Gyro_X =  (float)(((int32_t)Gyro_x - Gyro_x_bias)/(float)gyro_sensitivity)*M_PI/180.0f;
	Gyro_Y =  (float)(((int32_t)Gyro_y - Gyro_y_bias)/(float)gyro_sensitivity)*M_PI/180.0f;
	Gyro_Z =  (float)(((int32_t)Gyro_z - Gyro_z_bias)/(float)gyro_sensitivity)*M_PI/180.0f;

	// Get data of Magnetometer
	Get_magnetometer();
	//yaw = atan2(Accel_x,Accel_y) * RAD2DEC;
	//new_yaw = get_kalman_angle(yaw,Gyro_z/gyro_sensitivity,0.01);
	//MadgwickAHRSupdateIMU(Gyro_X,Gyro_Y,Gyro_Z,Accel_X,Accel_Y,Accel_Z);
	MadgwickAHRSupdate(Gyro_X*M_PI/180.0f,Gyro_Y*M_PI/180.0f,Gyro_Z*M_PI/180.0f,Accel_X,Accel_Y,Accel_Z,Mag_X_calib,Mag_Y_calib,-Mag_Z_calib);
	//MadgwickQuaternionUpdate(-Accel_X,Accel_Y,Accel_Z, Gyro_X*M_PI/180.0f,-Gyro_Y*M_PI/180.0f,-Gyro_Z*M_PI/180.0f, Mag_Y_calib,-Mag_X_calib,Mag_Z_calib);
	//MahonyAHRSupdate(Gyro_X*M_PI/180.0f,-Gyro_Y*M_PI/180.0f,-Gyro_Z*M_PI/180.0f,Accel_X,-Accel_Y,-Accel_Z,Mag_Y_calib,-Mag_X_calib,Mag_Z_calib);
}
void Set_Accel_Range(MPU9250_ACCEL_FULL_SCALE accel_FS)
{
	uint8_t mpu_address = MPU9250_ADDRESS_DEFAULT;
	uint8_t d[2];
	uint8_t reg1 = ACCEL_CONFIG;
	//uint8_t reg2 = ACCEL_CONFIG2;

	uint8_t c;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mpu_address,&reg1,1,1000) != HAL_OK);
	while(HAL_I2C_Master_Receive(&hi2c1, (uint16_t)mpu_address,&c, 1, 1000) != HAL_OK);
//	c[0] &=  ~0x18; // Clear AFS bits [4:3]
//	c[0] |= accel_FS; // Set full scale range for the gyro

	d[0] = ACCEL_CONFIG;
	d[1] = (c & 0xE7) | (uint8_t)accel_FS ;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mpu_address,(uint8_t *)d,2,100) != HAL_OK);


//	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mpu_address,&reg2,1,1000) != HAL_OK);
//	while(HAL_I2C_Master_Receive(&hi2c1, (uint16_t)mpu_address,c, 1, 1000) != HAL_OK);
//	c[0] &= ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
//	c[0] |=  0x03; // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	d[0] = ACCEL_CONFIG2;
	d[1] = 0x05;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mpu_address,(uint8_t *)d,2,1000)!=HAL_OK);
}
void Calibration_IMU()
{
	uint8_t data[14];
	uint8_t reg = ACCEL_XOUT_H;

	uint8_t mpu_address = MPU9250_ADDRESS_DEFAULT;

	for(int i=0; i < 100; i++){
		while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mpu_address,&reg,1,1000) != HAL_OK);
		while(HAL_I2C_Master_Receive(&hi2c1, (uint16_t)mpu_address, data, 14, 1000) != HAL_OK);

				/*-------- Accel ---------*/
		Accel_x = (int16_t)( (int16_t)( data[0] << 8 ) | data[1]);
		Accel_y = (int16_t)( (int16_t)( data[2] << 8 ) | data[3]);
		Accel_z = (int16_t)( (int16_t)( data[4] << 8 ) | data[5]);

				/*-------- Gyrometer --------*/
		Gyro_x = (int16_t)( (int16_t)( data[8]  << 8 ) | data[9]);
		Gyro_y = (int16_t)( (int16_t)( data[10] << 8 ) | data[11]);
		Gyro_z = (int16_t)( (int16_t)( data[12] << 8 ) | data[13]);

		Accel_x_bias += (int32_t)Accel_x;
		Accel_y_bias += (int32_t)Accel_y;
		Accel_z_bias += (int32_t)Accel_z;

		Gyro_x_bias += (int32_t)Gyro_x;
		Gyro_y_bias += (int32_t)Gyro_y;
		Gyro_z_bias += (int32_t)Gyro_z;
	}
	
	Accel_x_bias /= 100;
	Accel_y_bias /= 100;
	Accel_z_bias /= 100;

	Gyro_x_bias /= 100;
	Gyro_y_bias /= 100;
	Gyro_z_bias /= 100;

	if(Accel_z_bias > 0) //// Remove gravity from the z-axis accelerometer bias calculation
	{
		Accel_z_bias -= (int32_t)accel_sensitivity;
	}
	else
	{
		Accel_z_bias += (int32_t)accel_sensitivity;
	}
}

void Set_Gyro_Range(MPU9250_GYRO_FULL_SCALE gyro_FS)
{

//	uint8_t reg = GYRO_CONFIG;
	uint8_t mpu_address = MPU9250_ADDRESS_DEFAULT;
	uint8_t d[2];
//	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mpu_address,&reg,1,1000) != HAL_OK);
//	while(HAL_I2C_Master_Receive(&hi2c1, (uint16_t)mpu_address,c, 1, 1000) != HAL_OK);
//	c[0] &=  ~0x02; // Clear Fchoice bits [1:0]
//	c[0] &=  ~0x18; // Clear AFS bits [4:3]
//	c[0] |= gyro_FS;
	d[0] = GYRO_CONFIG;
	d[1] = gyro_FS;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mpu_address,(uint8_t *)d,2,100) != HAL_OK);

}
int Check_Connection(uint8_t return_true_val)
{
	// Check WHO_AM_I
	uint8_t who_i_am = (uint8_t)WHO_AM_I_MPU9250;
	uint8_t mpu_address = MPU9250_ADDRESS_DEFAULT;


    while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)mpu_address,&who_i_am,1,1000)!= HAL_OK);
    while(HAL_I2C_Master_Receive(&hi2c1,mpu_address,&temp,1,1000) != HAL_OK);
    if (temp == return_true_val)
    	return 1;
    else
    	return 0;

}
void mpu9250_set_clock_source(MPU9250_clock_source_t clock_source)
{
	uint8_t mpu_address = MPU9250_ADDRESS_DEFAULT;
	uint8_t d[2];

	d[0] = PWR_MGMT_1;
	d[1] = clock_source;
	while(HAL_I2C_Master_Transmit(&hi2c1,mpu_address,(uint8_t *)d,2,100)!= HAL_OK);
}

void Reset_MPU()
{
	uint8_t mpu_address = MPU9250_ADDRESS_DEFAULT;
	uint8_t d[2];
	d[0] = PWR_MGMT_1;
	d[1] = 0x00;
	while(HAL_I2C_Master_Transmit(&hi2c1,mpu_address,(uint8_t *)d,2,100)!= HAL_OK);
	HAL_Delay(1000);
}

void Complementary_filter(float Gyro_x,float Gyro_y,float Gyro_z,float  Acc_x,float Acc_y,float Acc_z, float dt)
{
	float accel_roll = atan2((double)Acc_y,(double)Acc_z)*RAD2DEC;
	float accel_pitch = atan2((double)-Acc_x,(double)sqrt(Acc_y*Acc_y + Acc_z*Acc_z))*RAD2DEC;
	roll = alpha*(roll + dt*Gyro_x) + (1-alpha)*accel_roll;
	pitch = alpha*(pitch + dt*Gyro_y) + (1-alpha)*accel_pitch;
}

void Get_magnetometer()
{
	uint8_t COMP[6];
	HAL_I2C_Mem_Read (&hi2c1, HMC5883L_ADDRESS, X_MSB , 1 ,(uint8_t *)COMP , 6 ,100 );

			Mag_x = (int16_t)((COMP[4]<<8) | COMP[5] );
			Mag_z = (int16_t)((COMP[2]<<8) | COMP[3] );
			Mag_y = -(int16_t)((COMP[0]<<8) | COMP[1] );

		Mag_X_calib = (float)Mag_x* mag_sensitivity - mag_offset[0];
		Mag_Y_calib = (float)Mag_y* mag_sensitivity - mag_offset[1];
		Mag_Z_calib = (float)Mag_z* mag_sensitivity - mag_offset[2];

}
void Calib_magnetometer()
{
	int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767};

	//int16_t mag_max[3] = {344.0,392.0,51.0},mag_min[3] = {-115.0,24.0,-323.0};

	//int16_t mag_max[3] = { ,0.0, 0.0},mag_min[3] = {200.00,200.00,200.0};
	mag_offset[0]=0.00;
	mag_offset[1]=0.00;
	mag_offset[2]=0.00;
	
	uint8_t raw_data[6];
	int32_t mag_temp[3] = {Mag_x,Mag_y,Mag_z};
	int32_t max_mag[3],min_mag[3];
	
	uint16_t i = 0;
	HAL_I2C_Mem_Read (&hi2c1, HMC5883L_ADDRESS, X_MSB , 1 ,(uint8_t *)raw_data , 6 ,100 );

			Mag_x = (int16_t)((raw_data[4]<<8) | raw_data[5] );
			Mag_z = (int16_t)((raw_data[2]<<8) | raw_data[3] );
			Mag_y = -(int16_t)((raw_data[0]<<8) | raw_data[1] );
	for (i = 0; i < 1500;i++)
	{
		HAL_I2C_Mem_Read (&hi2c1, HMC5883L_ADDRESS, X_MSB , 1 ,(uint8_t *)raw_data , 6 ,100 );

			Mag_x = (int16_t)((raw_data[4]<<8) | raw_data[5] );
			Mag_z = (int16_t)((raw_data[2]<<8) | raw_data[3] );
			Mag_y = -(int16_t)((raw_data[0]<<8) | raw_data[1] );
		//HAL_Delay(12);
		if (i==0){
			max_mag[0]=Mag_x;min_mag[0]=Mag_x;
			max_mag[1]=Mag_y;min_mag[1]=Mag_y;
			max_mag[2]=Mag_z;min_mag[2]=Mag_z;
		}
		else{
			if (Mag_x>max_mag[0]) max_mag[0]=Mag_x;
			if (Mag_y>max_mag[1]) max_mag[1]=Mag_y;
			if (Mag_z>max_mag[2]) max_mag[2]=Mag_z;
			if (Mag_x<min_mag[0]) min_mag[0]=Mag_x;
			if (Mag_y<min_mag[1]) min_mag[1]=Mag_y;
			if (Mag_z<min_mag[2]) min_mag[2]=Mag_z;
		}
		HAL_Delay(10);
	}
	
	
	// Get hard iron correction
//	mag_bias[0] = (mag_max[0] + mag_min[0])/2;
//	mag_bias[1] = (mag_max[1] + mag_min[1])/2;
//	mag_bias[2] = (mag_max[2] + mag_min[2])/2;

	mag_offset[0] = (float)(max_mag[0]- min_mag[0]) * mag_sensitivity ;
	mag_offset[1] = (float)(max_mag[1]- min_mag[1]) * mag_sensitivity ;
	mag_offset[2] = (float)(max_mag[2]- min_mag[2]) * mag_sensitivity ;
//	// Get soft iron correction estimate
//	mag_scale[0] = (mag_max[0] - mag_min[0])/2;
//	mag_scale[1] = (mag_max[1] - mag_min[1])/2;
//	mag_scale[2] = (mag_max[2] - mag_min[2])/2;

//	scale_x = 0,scale_y = 0,scale_z = 0;
//	float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
//	avg_rad /= 3.0;

//	scale_x = avg_rad/(float)mag_scale[0];    //1.14
//	scale_y = avg_rad/(float)mag_scale[1];  // 1.00
//	scale_z = avg_rad/(float)mag_scale[2]; // 0.89
}
void Quaternion_to_EulerAngle(float w,float x,float y,float z)
{
	  // roll (x-axis rotation)
		float sinr = 2*(w*x + y*z);
		float cosr = 1 - 2*(x*x + y*y);
		roll = atan2(sinr,cosr);

	 // pitch (y-axis rotation)
		float sinp = 2*(w*y - z*x);
		if (fabsf(sinp) >= 1)
			pitch = copysign(M_PI/2,sinp);
		else
			pitch =  asin(sinp);
	//// yaw (z-axis rotation)
		float siny = 2 *(w*z + x*y);
		float cosy = 1 - 2*(y*y + z*z);
		yaw = atan2(siny,cosy);

		roll = roll*180/M_PI;
		pitch = pitch*180/M_PI;
		yaw = yaw*180/M_PI;
}

