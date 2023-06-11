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


#define WHO_AM_I 			(0x75U)
#define GYRO_CONFIG 		(0x1BU)
#define ACCEL_CONFIG_ADD 	(0x1CU)
#define AFS_SEL_2 			(0x00U)
#define AFS_SEL_4 			(0x01U)
#define AFS_SEL_8 			(0x10U)
#define AFS_SEL_16 			(0x18U)



typedef struct
{
	/* i2c handle that contains all of the configuration for the i2c bus */
	I2C_HandleTypeDef *i2c_handle;

	/* UART handle that contains all of the configuration for UART line */
	UART_HandleTypeDef *uart_handle;

	/* pointer to a buffer that is responsible for transmitting data on the i2c bus */
	uint8_t* i2c_trans_buff;

	/* Indicates the number of bytes that are supposed to be transferred on the i2c bus */
	uint8_t i2c_tx_size;

	/* pointer to a buffer that is responsible for receiving data on the i2c bus */
	uint8_t* i2c_rece_buff;

	/* Indicates the number of bytes that are supposed to be transferred on the i2c bus */
	uint8_t i2c_rx_size;


}mpu_6050_t;

void initMPU_6050(mpu_6050_t *my_mpu_6050, I2C_HandleTypeDef *i2c, UART_HandleTypeDef *uart);

uint8_t I2C_Tx(mpu_6050_t *my_mpu_6050, uint8_t mpu_reg, uint8_t num_bytes);

uint8_t I2C_Rx(mpu_6050_t *my_mpu_6050, uint8_t mpu_reg, uint8_t num_bytes);




#endif /* MPU_6050_H_ */
