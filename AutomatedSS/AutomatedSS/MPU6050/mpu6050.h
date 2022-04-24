/*
	useful functions to manipulate mpu6050
*/

#ifndef MPU6050_H_
#define MPU6050_H_

// Include required built-in header files
#include <inttypes.h>
#include <stdint.h>

// user defined libraries
#include "../I2C/i2c.h"
#include "mpu6050_reg.h"

// variables
uint8_t ret; 
int16_t accel_buff[3], gyro_buff[3];
double accelX, accelY, accelZ;
double gyroX, gyroY, gyroZ;
double biasX, biasY;
double phi_accel, theta_accel;
double phi_innov, theta_innov;
double phi_est, theta_est;
double phi_prev, theta_prev;

//start mpu6050 over I2C
//return 0x68(device address with AD0 low), 
//return 0 if error
uint8_t mpu6050_start(void);


//configure important settings in mpu6050
//subject to change app(ilcation) by app
void mpu6050_init(void);


// read gyro/accel X, Y, Z all at once, high- & low-8-bits combined
// return int16_t (signed) in buff
// buff must have at least 3 available places
// data sequence: (buff)-->X, (buff+1)-->Y, (buff+2)-->Z
// no error handling for too small buff
void mpu6050_read_gyro_ALL(int16_t * buff);
void mpu6050_read_accel_ALL(int16_t * buff);


//read gyro/accel X, Y, Z, high- & low-8-bits separated, high first
//buff must have at least 2 available places
//no error handling for too small buff
void mpu6050_read_gyro_X(uint8_t * buff);
void mpu6050_read_gyro_Y(uint8_t * buff);
void mpu6050_read_gyro_Z(uint8_t * buff);
void mpu6050_read_accel_X(uint8_t * buff);
void mpu6050_read_accel_Y(uint8_t * buff);
void mpu6050_read_accel_Z(uint8_t * buff);


// newly added functions by bimalka98

#endif
