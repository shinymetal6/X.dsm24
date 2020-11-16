/*
 * AnalogChannels.c
 *
 *  Created on: Oct 28, 2020
 *      Author: fil
 */
#include "main.h"
#include "DCC.h"

s_sense	Sense;

void get_adc(uint8_t channel)
{
uint8_t chn = channel &0x07;

	switch ( chn)
	{
	case	0	:	HAL_ADC_Start(&hadc1); HAL_ADC_Start(&hadc2); break;
	case	1	:	Sense.sense0 = HAL_ADC_GetValue(&hadc1);break;
	case	2	:	Sense.sense1 = HAL_ADC_GetValue(&hadc1);break;
	case	3	:	Sense.ext_sense1 = HAL_ADC_GetValue(&hadc1);break;
	case	4	:	Sense.commsense = HAL_ADC_GetValue(&hadc2);break;
	case	5	:	Sense.ext_sense0 = HAL_ADC_GetValue(&hadc2);break;
	case	6	:	HAL_ADC_Stop(&hadc1);HAL_ADC_Stop(&hadc2);break;
	case	7	:	HAL_ADC_Stop(&hadc1);HAL_ADC_Stop(&hadc2);break;
	}
}
