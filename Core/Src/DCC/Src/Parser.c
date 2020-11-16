/*
 * Parser.c
 *
 *  Created on: Dec 9, 2019
 *      Author: fil
 */


#include "main.h"
#include "DCC.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

char 			outbuf[64];

void one_byte_commands(char cmd)
{
	switch ( cmd)
	{
	case '0' 	: 	sprintf(outbuf,"P0 Main Off\n\r");
					tracks.main_power = 0;
					tracks.aux_power = 0;
					break;
	case '1' 	: 	sprintf(outbuf,"P1 Main On\n\r");
					tracks.main_power = 1;
					tracks.aux_power = 1;
					break;
	case 'R' 	: 	sprintf(outbuf,"Sent RESET\n\r");
					insert_reset_packet_main();
					break;
	case 'S' 	: 	sprintf(outbuf,"Status\n\r");
					break;
	default:	sprintf(outbuf,"Command error\n\r");
	}
}

void three_bytes_commands(char cmd,int p0,int p1)
{
	switch ( cmd)
	{
	case 'f' 	: 	sprintf(outbuf,"f %d %d\n\r",p0,p1);
					break;
	case 'T' 	: 	sprintf(outbuf,"T %d %d\n\r",p0,p1);
					break;
	default:	sprintf(outbuf,"Command error\n\r");
	}
}

void four_bytes_commands(char cmd,int p0,int p1,int p2)
{
	switch ( cmd)
	{
	case 'f' 	: 	sprintf(outbuf,"f %d %d %d\n\r",p0,p1,p2);
					break;
	case 'a' 	: 	sprintf(outbuf,"a %d %d %d\n\r",p0,p1,p2);
					break;
	case 'T' 	: 	sprintf(outbuf,"T : Address %d , Speed %d , Direction %d\n\r",p0,p1,p2);
					if ( encode_throttle_main(p0,p1,p2) == 0 )
						insert_len3_packet_main();
					else
						insert_len4_packet_main();
					break;
	default:	sprintf(outbuf,"Command error\n\r");
	}
}

void five_bytes_commands(char cmd,int p0,int p1,int p2,int p3)
{
	switch ( cmd)
	{
	case 'T' 	: 	sprintf(outbuf,"Throttle %d %d %d %d\n\r",p0,p1,p2,p3);
					break;
	default		:	sprintf(outbuf,"Command error\n\r");
	}
}

void parser( void )
{
int		p0,p1,p2,p3,p4,p5,p6,p7,pnum;
char 	cmd, lbuf[USB_BUFLEN];

	memcpy(lbuf,&usb_rxbuf.packet[1],usb_rxbuf.byte_count-1); // skip < and > ... lazy
	pnum = sscanf(lbuf,"%c %d %d %d %d %d %d %d %d",&cmd,&p0,&p1,&p2,&p3,&p4,&p5,&p6,&p7);
	switch (pnum)
	{
	case 1 :	one_byte_commands(cmd);
				break;
	case 3 :	three_bytes_commands(cmd,p0,p1);
				break;
	case 4 :	four_bytes_commands(cmd,p0,p1,p2);
				break;
	case 5 :	five_bytes_commands(cmd,p0,p1,p2,p3);
				break;
	default:	sprintf(outbuf,"Command error %d\n\r",pnum);
	}
	usb_tx_buffer((uint8_t *)(outbuf));
}
