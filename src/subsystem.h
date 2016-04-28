/*
 * subsystem.h
 *
 *  Created on: 2015/1/24
 *      Author: rusei
 */
#ifndef SUBSYSTEM_H_
#define SUBSYSTEM_H_

#include "parameter.h"

#define ground_Test_Mode	1

#define enable_stm_EPS		0
#define enable_stm_ADCS		0
#define enable_stm_IFB		0

#define antenna_FM			0
#define antenna_deploy		0
#define initila_Time		946684800		//1475280000 			//946684800 
/* !!!!!!!!!!!!!!!!!!!!!!!! UINT = LSB !!!!!!!!!!!!!! */
/* Script Length */
#define scriptNum 			7
/* Tick */
#define delay_time_based	configTICK_RATE_HZ

/* I2C parameters*/
#define i2c_max_size 		246
#define com_tx_max 			235  // include i2c command = 236 Max per I2C Send
#define com_rx_max 			200
#define max_telemetry_size 	214

/* COM Board rx */
#define com_rx_node 		0x60	//0x60
#define com_rx_get 			0x22
#define com_rx_check 		0x21
#define com_rx_delete 		0x24
#define com_rx_hk 			0x1A
#define com_rx_hk_len 		14
#define com_reset 			0xab
/* COM Board tx */
#define com_tx_node 		0x61	//0x61
#define com_tx_send 		0x10
#define com_tx_send_with_callsign 0x11
#define com_tx_beacon_send 	0x14
#define com_to_callsign 	0x22
#define com_from_callsign 	0x23
#define com_tx_hk 			0x41
#define com_tx_rate 		0x28 //1=1200,8=9600
/* EPS Board */

#define eps_node 			0x02
#if ground_Test_Mode && enable_stm_EPS
#define stm_eps_node 		0x72
#else
#define stm_eps_node 		0x02
#endif

#define eps_hk 				8
#define eps_output 			9   //output mask
#define eps_hk_len 			43+2

/* SEUV Board */
#define seuv_node 			0x6E	//110
/* Interface Board */
#if ground_Test_Mode && enable_stm_IFB
#define interface_node 		0x7D	
#else
#define interface_node 		0x6D	//109
#endif
/* ADCS Board*/
#define adcs_node 			0x57
#if ground_Test_Mode && enable_stm_ADCS
#define stm_node			0x77
#else
#define stm_node			0x57
#endif
/* Antenna Board*/
#define ant_node 			49
#define ant_arm 			173
#define ant_deploy 			165
#define ant_deploy_timeout 	10

/*--------------common parameters-----------------*/
#define E_NO_ERR			-1
#define eps_hk_length 		43
#define com_rx_hk_length 	14
#define com_tx_hk_length 	1
#define adcs_hk_length 		48
#define seuv_data_length 	4

#define com_delay 			10	//1000
#define eps_delay 			7
#define seuv_delay 			10
#define interface_delay		7
#define adcs_delay 			10

#define adcs_power 			0x09
#define gps_power 			0x10
#define seuv_power 			0x24
/*----------------FS parameters------------------*/
#define No_Error			0
#define Error          		1
#define inms_data_length 	196
#define wod_length 			232
#define eop_length 			32				/* TBD */
#define seuv_length 		37
#define hk_length 			100 + sizeof(hk_frame_t) + 8
#define schedule_length		20
/*----------------AX.25 2ed header------------------*/
#define AX25_2ed_size 		5
/*----------------CCSDS parameters------------------*/

#define TM_NONDATA_SIZE 					16
#define ERR_MISSING_PARAMETER 				2
#define ERR_CCSDS_TOO_MUCH_DATA 			3
#define ERR_CCSDS_BUFFER_TOO_SMALL 			4
#define TM_HEADERS_SIZE 					14
#define ERR_SUCCESS 						0
#define ERR_I2C_FAIL 						5
#define ERR_SIZE_ERROR 						7
#define ERR_F_READ 							8
#define TM_S1_SUCCESS_SIZE 					4
#define TM_S1_FAILURE_SIZE 					6
#define CCSDS_T1_TELECOMMAND_VERIFICATION 	0x01
#define CCSDS_T1_ACCEPTANCE_FAIL 			0x02
#define CCSDS_S3_ACCEPTANCE_SUCCESS 		0x01
#define CCSDS_S3_COMPLETE_SUCCESS 			0x07
#define CCSDS_S3_COMPLETE_FAIL 				0x08
#define CCSDS_ERR_ILLEGAL_TYPE 				0xFF
#define I2C_SEND_ERROR 						0xA0
#define I2C_READ_ERROR 						0xA1
#define CCSDS_PACKET_ERROR 					0xB0
#define FS_IO_ERR      						0x0B
#define PARA_ERR							0x0c
/*----------------APID----------------*/
// Each downlink type shall have a dedicated APID
#define inms_apid 		14
#define phoenix_hk_apid 2
#define seuv_apid 		3
#define eop_apid		4
#define wod_apid 		5

#define eps_apid 		8
#define com_apid 		9
#define obc_apid 		10
#define adcs_apid 		11

/*---------------SID---------------*/
#define phoenix_hk_sid	1
#define inms_sid 		2
#define seuv_sid 		3
#define eop_sid 		4
#define wod_sid 		255

#define eps_hk_sid 		6
#define com_hk_sid 		7
#define adcs_hk_sid 	8

#define adcs_tmp_sid 	0xF0   //-----------------

/*---------------Service Type---------------*/
#define T3_SYS_CONF 				3
#define T8_function_management 		8
#define T11_OnBoard_Schedule		11
#define T13_LargeData_Transfer		13
#define T15_dowlink_management 		15
#define T131_ADCS 					131
#define T132_SEUV 					132


/*------Subsystem Control Functions-----*/

/* device code:
	* ADCS            = 1
	* GPS             = 2
	* SEUV            = 3
	* INMS            = 4
	* Interface Board = 5
*/
#define ON 1
#define OFF 0

void power_control(int device, int stats);
void deploy_antenna(int timeout);
int antenna_status_check();
void power_OFF_ALL();
int parameter_init();
uint32_t get_time();
uint16_t Interface_3V3_current_get();
uint16_t Interface_5V_current_get();
uint16_t Interface_tmp_get();
uint16_t Interface_inms_thermistor_get();
int status_update();
void generate_Error_Report(int type, uint16_t cause_value);
int report_Collected_Data(char *args, uint16_t *buffer_length);
int report_Crippled_Data(char *args, uint16_t *buffer_length);
void ECEFtoECI(float * Time, int32_t * r_ECEF, int16_t * r_ECI);

#endif /* SUBSYSTEM_H_ */
