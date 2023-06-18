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
#define ACCEL_X_OUT_H 		(0x3BU)
#define ACCEL_X_OUT_L		(0x3CU)
#define ACCEL_Y_OUT_H 		(0x3DU)
#define ACCEL_Y_OUT_L 		(0x3EU)
#define ACCEL_Z_OUT_H 		(0x3FU)
#define ACCEL_Z_OUT_L 		(0x40U)
/* Registers for the gyro */
#define GYRO_CONFIG			(0x1BU)


/* FIFO related registers */
#define FIFO_EN 			(0x23U)
#define FIFO_COUNT_H 		(0x72U)
#define FIFO_COUNT_L 		(0x73U)
#define FIFO_R_W 			(0x74U)
#define ENABLE_ALL			(0xFFU)
#define FIFO_ACCEL_EN		(1U << 3)

/* Power Management */
#define PWR_MGMT_1 			(0x6BU)


#define WHO_AM_I 			(0x75U)
#define GYRO_CONFIG 		(0x1BU)
#define ACCEL_CONFIG_ADD 	(0x1CU)
#define AFS_SEL_2 			(0x00U)
#define AFS_SEL_4 			(1U << 3)
#define AFS_SEL_8 			(2U << 3)
#define AFS_SEL_16 			(3U << 3)

typedef struct
{
	/* i2c handle that contains all of the configuration for the i2c bus */
	I2C_HandleTypeDef *i2c_handle;

	/* pointer to a buffer that is responsible for transmitting data on the i2c bus */
	uint8_t* i2c_tx_buff;

	/* pointer to a buffer that is responsible for receiving data on the i2c bus */
	uint8_t* i2c_rx_buff;

	int16_t accelX;
	int16_t accelY;
	int16_t accelZ;

	float  aX;
	float  aY;
	float  aZ;

}mpu_6050_t;

/* Tested */
void init_MPU_6050(mpu_6050_t *my_mpu_6050, I2C_HandleTypeDef *i2c);

/* Untested */
uint8_t selfTest(mpu_6050_t *my_mpu, uint8_t test_type);

uint8_t  mpu_Config(mpu_6050_t *my_mpu_6050);

/* This method identifies the slave address for the MPU_6050 */
uint8_t who_Am_I(mpu_6050_t *my_mpu_6050);

uint8_t fifo_Enable(mpu_6050_t *my_mpu_6050);

uint8_t get_Accel(mpu_6050_t *my_mpu_6050);

void print_Accel(mpu_6050_t *my_mpu_6050, UART_HandleTypeDef *uartHandle);

uint8_t wake(mpu_6050_t *my_mpu_6050);

uint8_t set_Sample_Rt(mpu_6050_t *my_mpu_6050);


#endif /* MPU_6050_H_ */
