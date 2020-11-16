/*
 * DCC_Packets.c
 *
 *  Created on: Dec 9, 2019
 *      Author: fil
 */


#include "main.h"
#include "DCC.h"
#include <string.h>

s_track 		tracks;

s_len3_packet 	len3_packet_main,len3_packet_aux;
s_len4_packet 	len4_packet_main,len4_packet_aux;
s_len5_packet 	len5_packet_main,len5_packet_aux;
s_len6_packet 	len6_packet_main,len6_packet_aux;
uint16_t		main_buffer[PACKET_MAX_SIZE] , aux_buffer[PACKET_MAX_SIZE];
uint16_t		main_buffer_len , aux_buffer_len;
s_len3_packet	idle_packet;
s_len3_packet	reset_packet;

void encode_byte(uint16_t *dest , uint8_t value)
{
	uint8_t i,mask=0x80;
	for ( i=0;i<8;i++)
	{
		dest[i] = BIT_0;
		if (( value & mask) == mask)
			dest[i] = BIT_1;
		mask = mask >> 1;
	}
}

uint32_t encode_throttle_main(int cab,int speed,int direction)
{
	// 01DCSSSS
uint16_t cab1,cab2,command,errcheck, retval=0;

	command = 0x40 | (direction << 5) | (speed & 0x1f);

	if ( cab > 127 )
	{
		retval=1;
		cab1 = (cab >> 8 ) | 0xc0;
		cab2 = cab & 0xff;
		errcheck = cab1 ^ cab2 ^ command;
		encode_byte(len4_packet_main.byte1,cab1);
		encode_byte(len4_packet_main.byte2,cab2);
		encode_byte(len4_packet_main.byte3,command);
		encode_byte(len4_packet_main.errcheck,errcheck);
	}
	else
	{
		retval=0;
		errcheck = cab ^ command;
		encode_byte(len3_packet_main.byte1,cab);
		encode_byte(len3_packet_main.byte2,command);
		encode_byte(len3_packet_main.errcheck,errcheck);
	}
	return retval;
}

void insert_reset_packet_main(void)
{
	if ( tracks.main_power == 0)
		return;
	while ( update_semaphore_main == 0);
	memcpy(dcc_packet_main,&reset_packet,sizeof(reset_packet));
	dcc_packet_main_size = sizeof(reset_packet)/2;
	update_semaphore_main = 0;
}

void insert_idle_packet_main(void)
{
	if ( tracks.main_power == 0)
		return;
	while ( update_semaphore_main == 0);
	memcpy(dcc_packet_main,&idle_packet,sizeof(idle_packet));
	dcc_packet_main_size = sizeof(idle_packet)/2;
	update_semaphore_main = 0;
}

void insert_len3_packet_main(void)
{
	if ( tracks.main_power == 0)
		return;
	while ( update_semaphore_main == 0);
	memcpy(dcc_packet_main,&len3_packet_main,sizeof(len3_packet_main));
	dcc_packet_main_size = sizeof(len3_packet_main)/2;
	update_semaphore_main = 0;
}

void insert_len4_packet_main(void)
{
	if ( tracks.main_power == 0)
		return;
	while ( update_semaphore_main == 0);
	memcpy(dcc_packet_main,&len4_packet_main,sizeof(len4_packet_main));
	dcc_packet_main_size = sizeof(len4_packet_main)/2;
	update_semaphore_main = 0;
}

void insert_reset_packet_aux(void)
{
	if ( tracks.aux_power == 0)
		return;
	while ( update_semaphore_aux == 0);
	memcpy(dcc_packet_aux,&reset_packet,sizeof(reset_packet));
	dcc_packet_aux_size = sizeof(reset_packet)/2;
	update_semaphore_aux = 0;
}

void insert_idle_packet_aux(void)
{
	if ( tracks.aux_power == 0)
		return;
	while ( update_semaphore_main == 0);
	memcpy(dcc_packet_aux,&idle_packet,sizeof(idle_packet));
	dcc_packet_aux_size = sizeof(idle_packet)/2;
	update_semaphore_aux = 0;
}

void insert_len3_packet_aux(void)
{
	if ( tracks.aux_power == 0)
		return;
	while ( update_semaphore_main == 0);
	memcpy(dcc_packet_aux,&len3_packet_main,sizeof(len3_packet_main));
	dcc_packet_aux_size = sizeof(len3_packet_main)/2;
	update_semaphore_aux = 0;
}

void insert_len4_packet_aux(void)
{
	if ( tracks.aux_power == 0)
		return;
	while ( update_semaphore_main == 0);
	memcpy(dcc_packet_aux,&len4_packet_main,sizeof(len4_packet_main));
	dcc_packet_aux_size = sizeof(len4_packet_main)/2;
	update_semaphore_aux = 0;
}

void init_bufs(void)
{
uint32_t	i;
	/* For main track */
	/* Prepares idle packets */
	for(i=0;i<PREAMBLE_LEN;i++)
		idle_packet.preamble[i] = BIT_1;
	encode_byte(idle_packet.byte1,0xff);
	encode_byte(idle_packet.byte2,0x00);
	encode_byte(idle_packet.errcheck,0xff);
	idle_packet.byte1_start=BIT_0;
	idle_packet.byte2_start=BIT_0;
	idle_packet.errdet_start=BIT_0;
	for(i=0;i<CLOSER_LEN;i++)
		idle_packet.closer[i] = BIT_1;

	/* Prepares reset packets */
	for(i=0;i<PREAMBLE_LEN;i++)
		reset_packet.preamble[i] = BIT_1;
	encode_byte(reset_packet.byte1,0x00);
	encode_byte(reset_packet.byte2,0x00);
	encode_byte(reset_packet.errcheck,0x00);
	reset_packet.byte1_start=BIT_0;
	reset_packet.byte2_start=BIT_0;
	reset_packet.errdet_start=BIT_0;
	for(i=0;i<CLOSER_LEN;i++)
		reset_packet.closer[i] = BIT_1;

	/* Prepare all the others */
	for(i=0;i<PREAMBLE_LEN;i++)
	{
		len3_packet_main.preamble[i] = BIT_1;
		len3_packet_main.preamble[i] = BIT_1;
		len3_packet_main.preamble[i] = BIT_1;
		len3_packet_main.preamble[i] = BIT_1;
	}
	for(i=0;i<CLOSER_LEN;i++)
	{
		len3_packet_main.closer[i] = BIT_1;
		len4_packet_main.closer[i] = BIT_1;
		len5_packet_main.closer[i] = BIT_1;
		len6_packet_main.closer[i] = BIT_1;
	}

	encode_byte(len3_packet_main.byte1,0xff);
	encode_byte(len3_packet_main.byte2,0x00);
	encode_byte(len3_packet_main.errcheck,0xff);
	len3_packet_main.byte1_start=BIT_0;
	len3_packet_main.byte2_start=BIT_0;
	len3_packet_main.errdet_start=BIT_0;

	encode_byte(len4_packet_main.byte1,0xff);
	encode_byte(len4_packet_main.byte2,0x00);
	encode_byte(len4_packet_main.byte3,0x00);
	encode_byte(len4_packet_main.errcheck,0xff);
	len4_packet_main.byte1_start=BIT_0;
	len4_packet_main.byte2_start=BIT_0;
	len4_packet_main.byte3_start=BIT_0;
	len4_packet_main.errdet_start=BIT_0;

	encode_byte(len5_packet_main.byte1,0xff);
	encode_byte(len5_packet_main.byte2,0x00);
	encode_byte(len5_packet_main.byte3,0x00);
	encode_byte(len5_packet_main.byte4,0x00);
	encode_byte(len5_packet_main.errcheck,0xff);
	len5_packet_main.byte1_start=BIT_0;
	len5_packet_main.byte2_start=BIT_0;
	len5_packet_main.byte3_start=BIT_0;
	len5_packet_main.byte4_start=BIT_0;
	len5_packet_main.errdet_start=BIT_0;

	encode_byte(len6_packet_main.byte1,0xff);
	encode_byte(len6_packet_main.byte2,0x00);
	encode_byte(len6_packet_main.byte3,0x00);
	encode_byte(len6_packet_main.byte4,0x00);
	encode_byte(len6_packet_main.byte5,0x00);
	encode_byte(len6_packet_main.errcheck,0xff);
	len6_packet_main.byte1_start=BIT_0;
	len6_packet_main.byte2_start=BIT_0;
	len6_packet_main.byte3_start=BIT_0;
	len6_packet_main.byte4_start=BIT_0;
	len6_packet_main.byte5_start=BIT_0;
	len6_packet_main.errdet_start=BIT_0;

	/* Define the current packet */
	insert_idle_packet_main();
}

