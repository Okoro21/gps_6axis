/*
 * MPU_6050.c
 *
 *  Created on: Jun 11, 2023
 *      Author: chris
 */

#include "MPU_6050.h"

void InitMPU_6050(mpu_6050_t *my_mpu_6050, I2C_HandleTypeDef *i2c, UART_HandleTypeDef *uart)
{
	/* create a parameter that determines the size of each array */
	uint8_t i2cTxBuffer[30] = {0};
	uint8_t i2cRxBuffer[30] = {0};

	my_mpu_6050->i2c_handle = i2c;
	my_mpu_6050->uart_handle = uart;

	my_mpu_6050->i2c_tx_buff = i2cTxBuffer;
	my_mpu_6050->i2c_rece_buff = i2cRxBuffer;

	my_mpu_6050->i2c_tx_size = 0;
	my_mpu_6050->i2c_rx_size = 0;

}

/* Create another parameter that will allow user to
 * configure the full scale range of the accelerometer
 */
void  Mpu_Config(mpu_6050_t *my_mpu_6050)
{
	my_mpu_6050->i2c_tx_buff[0] = ACCEL_CONFIG;

	/* changing the value written to ACCEL_CONFIG */
	my_mpu_6050->i2c_tx_buff[1] = AFS_SEL_8;

	if (I2C_Tx(my_mpu_6050, my_mpu_6050->i2c_tx_buff, 2) == HAL_OK)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
	 else
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
}

uint8_t I2C_Tx(mpu_6050_t *my_mpu_6050, uint8_t mpu_reg, uint8_t num_bytes)
{
	uint8_t uart_buff[20];
	uint8_t uart_len = 0;

	uint8_t i2c_Tx_flag = HAL_ERROR;

	my_mpu_6050->i2c_tx_buff[0] = mpu_reg;

	i2c_Tx_flag = HAL_I2C_Master_Transmit(my_mpu_6050->i2c_handle, MASTER_W, mpu_reg, num_bytes, 100);

	if (i2c_Tx_flag != HAL_OK)
	{
	  uart_len = sprintf((char *)uart_buff, "I2C Tx failed\r\n");
	  HAL_UART_Transmit(my_mpu_6050->uart_handle, uart_buff, uart_len, 100);
	}

	return i2c_Tx_flag;
}

uint8_t I2C_Rx(mpu_6050_t *my_mpu_6050, uint8_t mpu_reg, uint8_t num_bytes)
{
	uint8_t uart_buff[20];
	uint8_t uart_len = 0;

	uint8_t i2c_Rx_flag = HAL_ERROR;
	uint8_t i2c_dummy;
	/* check the return value
	 * Change to I2C_Tx
	 */
	 I2C_Tx(my_mpu_6050, &mpu_reg, num_bytes);

	i2c_Rx_flag = HAL_I2C_Master_Receive(my_mpu_6050->i2c_handle, MASTER_R, my_mpu_6050->i2c_rece_buff, num_bytes, 100);

	//i2c_Rx_flag = HAL_I2C_Master_Receive(my_mpu_6050->i2c_handle, MASTER_R, &i2c_dummy, num_bytes, 100);



	if (i2c_Rx_flag != HAL_OK)
	{
	  uart_len = sprintf((char *)uart_buff, "I2C Rx failed\r\n");
	  HAL_UART_Transmit(my_mpu_6050->uart_handle, uart_buff, uart_len, 100);
	}

	return i2c_Rx_flag;
}

uint8_t selfTest(mpu_6050_t *my_mpu, uint8_t test_type)
{
	uint8_t isOk = HAL_ERROR;

	uint8_t testX = 0;

//	uint8_t test;
//
//	switch (test_type)
//	{
//		case TEST_X:
//			test = SELF_TEST_X;
//		break;
//
//		case TEST_Y:
//			test = SELF_TEST_Y;
//		break;
//
//		case TEST_Z:
//			test = SELF_TEST_Z;
//		break;
//
//	}


	isOk = I2C_Rx(my_mpu, SELF_TEST_X, 4);

	testX |= my_mpu->i2c_rece_buff[0];
	/* Mask off the 5 least significant bits */
	testX  &= ~(0x1FU);

	testX >>= 3; // Should be 0x08

	testX |= ((my_mpu->i2c_rece_buff[3] >> 4) & 0x3U); //Should be 0x0B


	return isOk;
}



