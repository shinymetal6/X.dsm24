/*
 * RailCOM.c
 *
 *  Created on: Oct 28, 2020
 *      Author: fil
 */

#include "main.h"
#include "DCC.h"

uint8_t	railcom0_buf1[RAILCOM_BUF1_SIZE],railcom0_buf2[RAILCOM_BUF2_SIZE];
uint8_t	railcom1_buf1[RAILCOM_BUF1_SIZE],railcom1_buf2[RAILCOM_BUF2_SIZE];

uint8_t	railcom0_first, railcom1_first;
uint8_t	railcom0_done, railcom1_done;

void enable_railcom0(void)
{
	railcom0_first = railcom0_done = 0;
	HAL_UART_Receive_IT(&huart1, railcom0_buf1, RAILCOM_BUF1_SIZE);
}

void enable_railcom1(void)
{
	railcom1_first = railcom1_done = 0;
	HAL_UART_Receive_IT(&huart2, railcom1_buf1, RAILCOM_BUF1_SIZE);
}


