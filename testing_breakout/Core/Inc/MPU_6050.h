/*
 * MPU_6050.h
 *
 *  Created on: Jun 11, 2023
 *      Author: Chris
 */

#ifndef MPU_6050_H_
#define MPU_6050_H_

#include "stm32f7xx_hal.h"
#include <stdio.h>
#include <string.h>


/* unshifted address */
#define MPU_6050_ADDRESS 	(0x68U)

#define MASTER_R 			((MPU_6050_ADDRESS << 1) | 1U)
#define MASTER_W 			((MPU_6050_ADDRESS << 1) | 0U)

/* Self-Test for MPU-6050 sensors */
#define SELF_TEST_X 		(0x0DU)
#define SELF_TEST_Y			(0x0EU)
#define SELF_TEST_Z 		(0x0FU)
#define SELF_TEST_A 		(0x10U)

/* config registers */
#define CONFIG				(0x1AU)
#define SMPRT_DIV 			(0x19U)
#define USER_CTRL			(0x6AU)

/* Registers for the accelerometer */
#define ACCEL_CONFIG		(0x1CU)
#define ACCEL_XOUT_H 		(0x3BU)
#define ACCEL_XOUT_L		(0x3CU)
#define ACCEL_YOUT_H 		(0x3DU)
#define ACCEL_YOUT_L 		(0x3EU)
#define ACCEL_ZOUT_H 		(0x3FU)
#define ACCEL_ZOUT_L 		(0x40U)
#define AFS_SEL_2 			(0x00U)
#define AFS_SEL_4 			(1U << 3)
#define AFS_SEL_8 			(2U << 3)
#define AFS_SEL_16 			(3U << 3)



/* Registers for the gyro */
#define GYRO_CONFIG			(0x1BU)
#define GYRO_XOUT_H			(0x43U)
#define GYRO_XOUT_L			(0x44U)
#define GYRO_YOUT_H			(0x45U)
#define GYRO_YOUT_L			(0x46U)
#define GYRO_ZOUT_H			(0x47U)
#define GYRO_ZOUT_L			(0x48U)
#define FS_SEL_250			(0x00U)
#define FS_SEL_500			(1U << 3)
#define FS_SEL_1000			(2U << 3)
#define FS_SEL_2000			(3U << 3)


/* FIFO related registers */
#define FIFO_EN 			(0x23U)
#define FIFO_COUNT_H 		(0x72U)
#define FIFO_COUNT_L 		(0x73U)
#define FIFO_R_W 			(0x74U)
#define ENABLE_ALL			(0xFFU)
#define FIFO_ACCEL_EN		(1U << 3)

/* Power Management */
#define PWR_MGMT_1 			(0x6BU)

/* Interrupt */
#define INT_PIN_CFG			(0x37U)
#define INT_ENABLE 			(0x38U)
#define INT_STATUS 			(0x3AU)


#define WHO_AM_I 			(0x75U)

typedef struct
{
	/* i2c handle that contains all of the configuration for the i2c bus */
	I2C_HandleTypeDef *i2c_handle;

	/* pointer to a buffer that is responsible for transmitting data on the i2c bus */
	uint8_t* i2c_tx_buff;

	/* pointer to a buffer that is responsible for receiving data on the i2c bus */
	uint8_t* i2c_rx_buff;

	/* Contain the 16 bit acceleration value output from the MPU_6050 */
	int16_t accelX;
	int16_t accelY;
	int16_t accelZ;

	/* Two decimal precision acceleration values */
	float  aX;
	float  aY;
	float  aZ;

	/* Contain the 16 bit gyro value output from the MPU_6050 */
	int16_t gyroX;
	int16_t gyroY;
	int16_t gyroZ;

	/* Two decimal precision gyro values */
	float  gX;
	float  gY;
	float  gZ;

}mpu_6050_t;

/* Pre: Receive an mpu_6050_t struct with accelerometer data in it
 * Post return HAL_ERROR if Tx was unsuccessful, else returns HAL_OK
 */
uint8_t tx_Accel_Data(mpu_6050_t *my_mpu_6050, UART_HandleTypeDef *uartHandle);

//uint8_t INT_Config(mpu_6050_t *my_mpu_6050);

void init_MPU_6050(mpu_6050_t *my_mpu_6050, I2C_HandleTypeDef *i2c);

uint8_t selfTest(mpu_6050_t *my_mpu, uint8_t test_type);

uint8_t  accel_Gyro_Config(mpu_6050_t *my_mpu_6050);

/* This method identifies the slave address for the MPU_6050 */
uint8_t who_Am_I(mpu_6050_t *my_mpu_6050);

uint8_t fifo_Enable(mpu_6050_t *my_mpu_6050);

uint8_t get_Accel(mpu_6050_t *my_mpu_6050);

void formatAccel(mpu_6050_t *my_mpu_6050);

void tx_Accel(mpu_6050_t *my_mpu_6050, UART_HandleTypeDef *uartHandle);

uint8_t get_Gyro(mpu_6050_t *my_mpu_6050);

void formatGyro(mpu_6050_t *my_mpu_6050);

uint8_t wake(mpu_6050_t *my_mpu_6050);

uint8_t set_Sample_Rt(mpu_6050_t *my_mpu_6050);


#endif /* MPU_6050_H_ */
