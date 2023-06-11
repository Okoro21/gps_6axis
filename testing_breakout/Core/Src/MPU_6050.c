/*
 * MPU_6050.c
 *
 *  Created on: Jun 11, 2023
 *      Author: chris
 */

#include "MPU_6050.h"

void initMPU_6050(mpu_6050_t *my_mpu_6050, I2C_HandleTypeDef *i2c, UART_HandleTypeDef *uart)
{
	/* create a paramter that determines the size of each array */
	uint8_t i2cTxBuffer[30];
	uint8_t i2cRxBuffer[30];

	my_mpu_6050->i2c_handle = i2c;
	my_mpu_6050->uart_handle = uart;

	my_mpu_6050->i2c_trans_buff = i2cTxBuffer;
	my_mpu_6050->i2c_rece_buff = i2cRxBuffer;

	my_mpu_6050->i2c_tx_size = 0;
	my_mpu_6050->i2c_rx_size = 0;

}

uint8_t I2C_Tx(mpu_6050_t *my_mpu_6050, uint8_t mpu_reg, uint8_t num_bytes)
{
	uint8_t uart_buff[20];
	uint8_t uart_len = 0;

	uint8_t i2c_Tx_flag = HAL_ERROR;


	i2c_Tx_flag = HAL_I2C_Master_Transmit(my_mpu_6050->i2c_handle, MASTER_W, &mpu_reg, 1, 100);

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

	/* check the return value */
	HAL_I2C_Master_Transmit(my_mpu_6050->i2c_handle, MASTER_W, &mpu_reg, 1, 100);

	i2c_Rx_flag = HAL_I2C_Master_Receive(my_mpu_6050->i2c_handle, MASTER_R, my_mpu_6050->i2c_rece_buff, 1, 100);

	if (i2c_Rx_flag != HAL_OK)
	{
	  uart_len = sprintf((char *)uart_buff, "I2C Rx failed\r\n");
	  HAL_UART_Transmit(my_mpu_6050->uart_handle, uart_buff, uart_len, 100);
	}

	return i2c_Rx_flag;
}



