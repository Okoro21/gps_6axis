/*
 * MPU_6050.c
 *
 *  Created on: Jun 11, 2023
 *      Author: chris
 */

#include "MPU_6050.h"


uint8_t who_Am_I(mpu_6050_t *my_mpu_6050)
{
	/* flags that check if communication between i2c master and slave was successful */
	uint8_t i2c_Tx_flag = HAL_ERROR;
	uint8_t i2c_Rx_flag = HAL_ERROR;
	uint8_t i2c_success = HAL_ERROR;

	my_mpu_6050->i2c_tx_buff[0] = WHO_AM_I;

	i2c_Tx_flag = HAL_I2C_Master_Transmit(my_mpu_6050->i2c_handle, MASTER_W, my_mpu_6050->i2c_tx_buff, 1, 100);

	i2c_Rx_flag = HAL_I2C_Master_Receive(my_mpu_6050->i2c_handle, MASTER_R, my_mpu_6050->i2c_rx_buff, 1, 100);

	if (i2c_Tx_flag == HAL_OK && i2c_Rx_flag == HAL_OK)
		i2c_success = HAL_OK;

	/* You are also supposed to check A0 Pin on MPU_6050 */

	return i2c_success;
}

void init_MPU_6050(mpu_6050_t *my_mpu_6050, I2C_HandleTypeDef *i2c)
{
	/* create a parameter that determines the size of each array */
	uint8_t i2cTxBuffer[6] = {0};
	uint8_t i2cRxBuffer[6] = {0};

	my_mpu_6050->i2c_handle = i2c;

	my_mpu_6050->i2c_tx_buff = i2cTxBuffer;
	my_mpu_6050->i2c_rx_buff = i2cRxBuffer;
}

/* Create another parameter that will allow user to
 * configure the full scale range of the accelerometer
 */
uint8_t accel_Gyro_Config(mpu_6050_t *my_mpu_6050)
{
	uint8_t configSuccess = HAL_ERROR;
	uint8_t dlpfSet = HAL_ERROR;

	/* Selecting the 8g full range scale for the accelerometer
	 * by writing AFS_SEL_8 to ACCEL_CONFIG register
	 */
	my_mpu_6050->i2c_tx_buff[0] = ACCEL_CONFIG;

	my_mpu_6050->i2c_tx_buff[1] = AFS_SEL_8;

	/* Selecting the 250 degree/seconds full range scale for the gyro
	 * by writing FS_SEL_250 to GYRO_CONFIG register
	 */

	my_mpu_6050->i2c_tx_buff[2] = GYRO_CONFIG;

	my_mpu_6050->i2c_tx_buff[3] = FS_SEL_250;


	configSuccess = HAL_I2C_Master_Transmit(my_mpu_6050->i2c_handle, MASTER_W, my_mpu_6050->i2c_tx_buff, 4, 100);

/* Enabling the digital low pass filter */

	my_mpu_6050->i2c_tx_buff[0] = CONFIG;
	my_mpu_6050->i2c_tx_buff[1] = (0x05U);

	dlpfSet = HAL_I2C_Master_Transmit(my_mpu_6050->i2c_handle, MASTER_W, my_mpu_6050->i2c_tx_buff, 2, 100);

return configSuccess;
}

uint8_t fifo_Enable(mpu_6050_t *my_mpu_6050)
{
	uint8_t enableSuccess = HAL_ERROR;

	my_mpu_6050->i2c_tx_buff[0] = FIFO_EN;

	my_mpu_6050->i2c_tx_buff[1] = FIFO_ACCEL_EN;

	enableSuccess = HAL_I2C_Master_Transmit(my_mpu_6050->i2c_handle, MASTER_W, my_mpu_6050->i2c_tx_buff, 2, 100);

	return enableSuccess;
}

uint8_t get_Accel(mpu_6050_t *my_mpu_6050)
{
	/* flags that check if communication between i2c master and slave was successful */
	uint8_t i2c_Tx_flag = HAL_ERROR;
	uint8_t i2c_Rx_flag = HAL_ERROR;
	uint8_t i2c_success = HAL_ERROR;

	my_mpu_6050->i2c_tx_buff[0] = ACCEL_XOUT_H;

	i2c_Tx_flag = HAL_I2C_Master_Transmit(my_mpu_6050->i2c_handle, MASTER_W, my_mpu_6050->i2c_tx_buff, 1, 1000);


	/* Retrieve the acceleration values from 6 registers
	 * ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, ACCEL_ZOUT_L respectively
	 */
	i2c_Rx_flag = HAL_I2C_Master_Receive(my_mpu_6050->i2c_handle, MASTER_R, my_mpu_6050->i2c_rx_buff, 6, 1000);

	if (i2c_Tx_flag == HAL_OK && i2c_Rx_flag == HAL_OK)
		i2c_success = HAL_OK;

	/* You are also supposed to check A0 Pin on MPU_6050 */

	return i2c_success;
}

void formatAccel(mpu_6050_t *my_mpu_6050)
{
	my_mpu_6050->accelX = (int16_t)((my_mpu_6050->i2c_rx_buff[0] << 8) | my_mpu_6050->i2c_rx_buff[1]);
	my_mpu_6050->aX =  ((float)(my_mpu_6050->accelX))/4096;

	/* Calibration value for acceleration in the x direction */
	//my_mpu_6050->aX-= 0.089;

	my_mpu_6050->accelY = (int16_t)((my_mpu_6050->i2c_rx_buff[2] << 8) | my_mpu_6050->i2c_rx_buff[3]);
	my_mpu_6050->aY =  ((float)my_mpu_6050->accelY)/4096;

	/* Calibration value for acceleration in the y direction */
	//my_mpu_6050->aY += 1.05;

	my_mpu_6050->accelZ = (int16_t)((my_mpu_6050->i2c_rx_buff[4] << 8) | my_mpu_6050->i2c_rx_buff[5]);
	my_mpu_6050->aZ =  ((float)my_mpu_6050->accelZ)/4096;

	/* Calibration value for acceleration in the z direction */
	my_mpu_6050->aZ += 0.1;
}

uint8_t tx_Accel_Data(mpu_6050_t *my_mpu_6050, UART_HandleTypeDef *uartHandle)
{
	/* Fetches the number of data bytes within i2c_rx_buff
	 * data_size should == 6 since we are fetching the MSB and LSB of each acceleration axes
	 */

	uint8_t data_size = strlen(my_mpu_6050->i2c_rx_buff);
	uint8_t uart_tx_success = HAL_ERROR;

	for (uint8_t index = 0; index < data_size; index++)
		my_mpu_6050->i2c_tx_buff[index] = my_mpu_6050->i2c_rx_buff[index];

	uart_tx_success = HAL_UART_Transmit(uartHandle, my_mpu_6050->i2c_tx_buff, data_size, 1000);

	return uart_tx_success;
}

void print_Accel(mpu_6050_t *my_mpu_6050, UART_HandleTypeDef *uartHandle)
{
	uint8_t uart_buff[1024];
	uint8_t uart_len = 0;

	uart_len = sprintf((char *)uart_buff, "AccelX: %.2f , AccelY: %.2f, AccelZ: %.2f\r\n", my_mpu_6050->aX, my_mpu_6050->aY, my_mpu_6050->aZ);
	HAL_UART_Transmit(uartHandle, uart_buff, uart_len, 100);
	HAL_Delay(500);
}

uint8_t get_Gyro(mpu_6050_t *my_mpu_6050)
{
	uint8_t i2c_Tx_flag = HAL_ERROR;
	uint8_t i2c_Rx_flag = HAL_ERROR;
	uint8_t i2c_success = HAL_ERROR;

	my_mpu_6050->i2c_tx_buff[0] = GYRO_XOUT_H;

	i2c_Tx_flag = HAL_I2C_Master_Transmit(my_mpu_6050->i2c_handle, MASTER_W, my_mpu_6050->i2c_tx_buff, 1, 1000);

	i2c_Rx_flag = HAL_I2C_Master_Receive(my_mpu_6050->i2c_handle, MASTER_R, my_mpu_6050->i2c_rx_buff, 6, 1000);

	if (i2c_Tx_flag == HAL_OK && i2c_Rx_flag == HAL_OK)
		i2c_success = HAL_OK;

	return i2c_success;
}

void formatGyro(mpu_6050_t *my_mpu_6050)
{
	my_mpu_6050->gyroX = (int16_t)((my_mpu_6050->i2c_rx_buff[0] << 8) | my_mpu_6050->i2c_rx_buff[1]);
	my_mpu_6050->gX = (my_mpu_6050->gyroX)/131.0;

	my_mpu_6050->gyroY = (int16_t)((my_mpu_6050->i2c_rx_buff[2] << 8) | my_mpu_6050->i2c_rx_buff[3]);
	my_mpu_6050->gY = (my_mpu_6050->gyroY)/131.0;

	my_mpu_6050->gyroZ = (int16_t)((my_mpu_6050->i2c_rx_buff[4] << 8) | my_mpu_6050->i2c_rx_buff[5]);
	my_mpu_6050->gZ = (my_mpu_6050->gyroZ)/131.0;
}

void print_Gyro(mpu_6050_t *my_mpu_6050, UART_HandleTypeDef *uartHandle)
{
	uint8_t uart_buff[1024];
	uint8_t uart_len = 0;

	//uart_len = sprintf((char *)uart_buff, "gyroX: %hd , gyroY: %hd, gyroZ: %hd\r\n", my_mpu_6050->gyroX, my_mpu_6050->gyroY, my_mpu_6050->gyroZ);
	uart_len = sprintf((char *)uart_buff, "gX: %.2f , gY: %.2f, gZ: %.2f\r\n", my_mpu_6050->gX, my_mpu_6050->gY, my_mpu_6050->gZ);
	HAL_UART_Transmit(uartHandle, uart_buff, uart_len, 100);
	HAL_Delay(500);
}


uint8_t set_Sample_Rt(mpu_6050_t *my_mpu_6050)
{
	uint8_t sampleSuccess = HAL_ERROR;

	my_mpu_6050->i2c_tx_buff[0] = SMPRT_DIV;

	/* Divider == 8 therefore sampleRate of accelerometer and gryo = 8kHz/8 == 1kHz */
	my_mpu_6050->i2c_tx_buff[1] = 0x08U;

	sampleSuccess = HAL_I2C_Master_Transmit(my_mpu_6050->i2c_handle, MASTER_W, my_mpu_6050->i2c_tx_buff, 2, 100);

	return sampleSuccess;
}

uint8_t wake(mpu_6050_t *my_mpu_6050)
{
	uint8_t wakeSuccess = HAL_ERROR;

	my_mpu_6050->i2c_tx_buff[0] = PWR_MGMT_1;

	my_mpu_6050->i2c_tx_buff[1] = 0x00U;

	wakeSuccess = HAL_I2C_Master_Transmit(my_mpu_6050->i2c_handle, MASTER_W, my_mpu_6050->i2c_tx_buff, 2, 100);

	return wakeSuccess;
}

//
//uint8_t selfTest(mpu_6050_t *my_mpu, uint8_t test_type)
//{
//	uint8_t isOk = HAL_ERROR;
//
//	uint8_t testX = 0;
//
////	uint8_t test;
////
////	switch (test_type)
////	{
////		case TEST_X:
////			test = SELF_TEST_X;
////		break;
////
////		case TEST_Y:
////			test = SELF_TEST_Y;
////		break;
////
////		case TEST_Z:
////			test = SELF_TEST_Z;
////		break;
////
////	}
//
//
//	isOk = I2C_Rx(my_mpu, SELF_TEST_X, 4);
//
//	testX |= my_mpu->i2c_rece_buff[0];
//	/* Mask off the 5 least significant bits */
//	testX  &= ~(0x1FU);
//
//	testX >>= 3; // Should be 0x08
//
//	testX |= ((my_mpu->i2c_rece_buff[3] >> 4) & 0x3U); //Should be 0x0B
//
//
//	return isOk;
//}



