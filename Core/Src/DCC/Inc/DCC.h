/*
 * DCC.h
 *
 *  Created on: Oct 28, 2020
 *      Author: fil
 */

#ifndef SRC_DCC_INC_DCC_H_
#define SRC_DCC_INC_DCC_H_

//#include "main.h"

extern	TIM_HandleTypeDef htim1;
extern	TIM_HandleTypeDef htim16;
extern	TIM_HandleTypeDef htim6;
extern	TIM_HandleTypeDef htim7;
extern	DAC_HandleTypeDef hdac1;
extern	ADC_HandleTypeDef hadc1;
extern	ADC_HandleTypeDef hadc2;
extern	UART_HandleTypeDef hlpuart1;
extern	UART_HandleTypeDef huart1;
extern	UART_HandleTypeDef huart2;

/* DCC_Packets.c */
extern	void insert_reset_packet_main(void);
extern	void init_bufs(void);
extern	uint32_t encode_throttle_main(int cab,int speed,int direction);
extern	void insert_len3_packet_main(void);
extern	void insert_len4_packet_main(void);
extern	void insert_reset_packet_main(void);

/* CallBacks.c */
extern	void packetEND_main_callback(void);
extern	void cutoutEND_main_callback(void);
extern	void packetEND_aux_callback(void);
extern	void cutoutEND_aux_callback(void);

#define	PACKET_MAX_SIZE	80

extern	uint16_t 	dcc_packet_main[PACKET_MAX_SIZE],dcc_packet_aux[PACKET_MAX_SIZE];
extern	uint8_t		dcc_packet_main_size,dcc_packet_aux_size;
extern	uint8_t		update_semaphore_main,update_semaphore_aux;
extern	uint8_t		flag10ms;
extern	uint8_t		main_packet_sent,aux_packet_sent;


#define	DAC_BLANKS_RAILCOM	0x3ff
#define	DAC_ENABLE_RAILCOM	0x030

/* TimDMA.c */
extern	void PWM_Start_DMA_Tim1(uint32_t *pData, uint16_t Length);
extern	void PWM_Start_DMA_Tim16(uint32_t *pData, uint16_t Length);
extern	void tim6_start(void);
extern	void tim7_start(void);

/* UsbUart.c */
extern	void usb_tx_buffer(uint8_t *pData);
extern	uint32_t usb_check_rx_buffer(void);

/* Parser.c */
extern	void parser(void);

/* AnalogChannels.c */
extern	void get_adc(uint8_t channel);

/* RailCOM.c */
extern	void enable_railcom0(void);
extern	void enable_railcom1(void);
extern	void railcom0_callback(void);
extern	void railcom1_callback(void);

#define	RAILCOM_BUF1_SIZE	2
#define	RAILCOM_BUF2_SIZE	6
extern	uint8_t	railcom0_buf1[RAILCOM_BUF1_SIZE],railcom0_buf2[RAILCOM_BUF2_SIZE];
extern	uint8_t	railcom1_buf1[RAILCOM_BUF1_SIZE],railcom1_buf2[RAILCOM_BUF2_SIZE];
extern	uint8_t	railcom0_first, railcom1_first;
extern	uint8_t	railcom0_done, railcom1_done;


typedef struct {
	uint16_t 	sense0;
	uint16_t 	sense1;
	uint16_t 	commsense;
	uint16_t 	ext_sense0;
	uint16_t 	ext_sense1;
} s_sense;
extern	s_sense	Sense;


#define		BIT_0		340
#define		BIT_1		170
#define		PREAMBLE_LEN	15
#define		BYTE_LEN		8
#define		CLOSER_LEN		2

#define		PACKET_LEN	256
#define		PACKET_LEN	256

typedef struct {
	uint8_t 	main_power;
	uint8_t 	aux_power;
} s_track;
extern	s_track 		tracks;

typedef struct {
	uint16_t preamble[PREAMBLE_LEN];
	uint16_t byte1_start;
	uint16_t byte1[BYTE_LEN];
	uint16_t byte2_start;
	uint16_t byte2[BYTE_LEN];
	uint16_t errdet_start;
	uint16_t errcheck[BYTE_LEN];
	uint16_t closer[CLOSER_LEN];
} s_len3_packet;

typedef struct {
	uint16_t preamble[PREAMBLE_LEN];
	uint16_t byte1_start;
	uint16_t byte1[BYTE_LEN];
	uint16_t byte2_start;
	uint16_t byte2[BYTE_LEN];
	uint16_t byte3_start;
	uint16_t byte3[BYTE_LEN];
	uint16_t errdet_start;
	uint16_t errcheck[BYTE_LEN];
	uint16_t closer[CLOSER_LEN];
} s_len4_packet;

typedef struct {
	uint16_t preamble[PREAMBLE_LEN];
	uint16_t byte1_start;
	uint16_t byte1[BYTE_LEN];
	uint16_t byte2_start;
	uint16_t byte2[BYTE_LEN];
	uint16_t byte3_start;
	uint16_t byte3[BYTE_LEN];
	uint16_t byte4_start;
	uint16_t byte4[BYTE_LEN];
	uint16_t errdet_start;
	uint16_t errcheck[BYTE_LEN];
	uint16_t closer[CLOSER_LEN];
} s_len5_packet;

typedef struct {
	uint16_t preamble[PREAMBLE_LEN];
	uint16_t byte1_start;
	uint16_t byte1[BYTE_LEN];
	uint16_t byte2_start;
	uint16_t byte2[BYTE_LEN];
	uint16_t byte3_start;
	uint16_t byte3[BYTE_LEN];
	uint16_t byte4_start;
	uint16_t byte4[BYTE_LEN];
	uint16_t byte5_start;
	uint16_t byte5[BYTE_LEN];
	uint16_t errdet_start;
	uint16_t errcheck[BYTE_LEN];
	uint16_t closer[CLOSER_LEN];
} s_len6_packet;


/* USB UART */

#define	USB_BUFLEN	16
typedef struct {
	uint8_t byte_count;
	uint8_t packet[USB_BUFLEN-1];
} s_usb_rxbuf;

extern s_usb_rxbuf 	usb_rxbuf;
extern uint8_t 	rbuffer[USB_BUFLEN];
extern uint8_t 	rbuffer_size,usb_rx_buffer_ready;

extern	s_len3_packet 	len3_packet_main,len3_packet_aux;
extern	s_len4_packet 	len4_packet_main,len4_packet_aux;
extern	s_len5_packet 	len5_packet_main,len5_packet_aux;
extern	s_len6_packet 	len6_packet_main,len6_packet_aux;

extern	s_len3_packet	idle_packet;
extern	s_len3_packet	reset_packet;


#endif /* SRC_DCC_INC_DCC_H_ */
