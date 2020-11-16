/*
 * UsbUart.c
 *
 *  Created on: Dec 9, 2019
 *      Author: fil
 */

#include "main.h"
#include "DCC.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
s_usb_rxbuf	usb_rxbuf;
uint8_t		usb_rx_buffer_ready;
uint8_t 	rbuffer[USB_BUFLEN];
uint8_t 	rbuffer_size;

extern		uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

void usb_tx_buffer(uint8_t *pData)
{
	CDC_Transmit_FS(pData,strlen((char *)pData));
}

uint32_t usb_check_rx_buffer(void)
{
	if ( usb_rx_buffer_ready == 1 )
	{
		usb_rx_buffer_ready = 0;
		if (rbuffer[0]== '<')
		{
			for(usb_rxbuf.byte_count=0;usb_rxbuf.byte_count<USB_BUFLEN;usb_rxbuf.byte_count++)
			{
				usb_rxbuf.packet[usb_rxbuf.byte_count] = rbuffer[usb_rxbuf.byte_count];
				if ( usb_rxbuf.packet[usb_rxbuf.byte_count] == '>')
				{
					usb_rxbuf.byte_count ++;
					return usb_rxbuf.byte_count;
				}
			}
		}
	}
	return 0;
}

