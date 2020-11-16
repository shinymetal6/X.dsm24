/*
 * CallBacks.c
 *
 *  Created on: Oct 28, 2020
 *      Author: fil
 */
#include "main.h"
#include "DCC.h"

uint16_t 	dcc_packet_main[PACKET_MAX_SIZE],dcc_packet_aux[PACKET_MAX_SIZE];
uint8_t		dcc_packet_main_size,dcc_packet_aux_size;
uint8_t		update_semaphore_main;	/* when 1 the fields can be updated */
uint8_t		update_semaphore_aux; 	/* when 1 the fields can be updated */
uint8_t		main_packet_sent=0,aux_packet_sent=0;
uint8_t		flag10ms=0;

void packetEND_main_callback(void)
{
	update_semaphore_main = 1;
	htim1.Instance->CR1 &= ~TIM_CR1_CEN;
	main_packet_sent++;
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_GPIO_WritePin(ENABLE0_GPIO_Port, ENABLE0_Pin,GPIO_PIN_RESET);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_ENABLE_RAILCOM);
}

void cutoutEND_main_callback(void)
{
	update_semaphore_main = 0;
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_BLANKS_RAILCOM);
	PWM_Start_DMA_Tim1( (uint32_t *)dcc_packet_main , dcc_packet_main_size);
	htim1.Instance->CR1|=TIM_CR1_CEN;
	if ( tracks.main_power == 1 )
		HAL_GPIO_WritePin(ENABLE0_GPIO_Port, ENABLE0_Pin,GPIO_PIN_SET);
}

void packetEND_aux_callback(void)
{
	update_semaphore_aux = 1;
	htim1.Instance->CR1 &= ~TIM_CR1_CEN;
	aux_packet_sent++;
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_GPIO_WritePin(ENABLE1_GPIO_Port, ENABLE1_Pin,GPIO_PIN_RESET);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, DAC_ENABLE_RAILCOM);
}

void cutoutEND_aux_callback(void)
{
	update_semaphore_aux = 0;
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, DAC_BLANKS_RAILCOM);
	PWM_Start_DMA_Tim16( (uint32_t *)dcc_packet_aux , dcc_packet_aux_size);
	htim16.Instance->CR1|=TIM_CR1_CEN;
	if ( tracks.aux_power == 1 )
		HAL_GPIO_WritePin(ENABLE1_GPIO_Port, ENABLE1_Pin,GPIO_PIN_SET);
}

void railcom0_callback(void)
{
	if ( railcom0_first == 0 )
	{
		HAL_UART_Receive_IT(&huart1, railcom0_buf2, RAILCOM_BUF2_SIZE);
		railcom0_first = 1;
	}
	else
	{
		railcom0_done=1;
		railcom0_first=0;
	}
}

void railcom1_callback(void)
{
	if ( railcom1_first == 0 )
	{
		HAL_UART_Receive_IT(&huart1, railcom1_buf2, RAILCOM_BUF2_SIZE);
		railcom1_first = 1;
	}
	else
	{
		railcom1_done=1;
		railcom1_first=0;
	}
}

void irq10ms(void)
{
	flag10ms = 1;
}


