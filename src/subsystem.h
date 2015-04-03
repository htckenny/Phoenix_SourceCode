/*
 * subsystem.h
 *
 *  Created on: 2015/1/24
 *      Author: rusei
 */
#ifndef SUBSYSTEM_H_
#define SUBSYSTEM_H_

/* I2C parameters*/
#define i2c_max_size 246
#define com_tx_max 235  // include i2c command = 236 Max per I2C Send
#define com_rx_max 200
#define max_telemetry_size 214

/* COM Board rx */
#define com_rx_node 80
#define com_rx_get 0x22
#define com_rx_check 0x21
#define com_rx_delete 0x24
#define com_rx_hk 26
#define com_reset 0xab

/* COM Board tx */
#define com_tx_node 81
#define com_tx_send 16
#define com_tx_send_with_callsign 17
#define com_tx_mode 37
#define com_tx_beacon_send 20
#define com_to_callsign 34
#define com_from_callsign 35
#define com_tx_hk 0x41

/* EPS Board */
#define eps_node 2
#define eps_hk 8
#define eps_output 9   //output mask
/* SEUV Board */
#define seuv_node 110
/* Interface Board */
#define interface_node 109
/* ADCS Board*/
#define adcs_node 18
/* Antenna Board*/
#define ant_node 49
#define ant_arm 173
#define ant_deploy 165
#define ant_deploy_timeout 255

/*--------------common parameters-----------------*/
#define E_NO_ERR -1
#define eps_hk_length 43
#define com_rx_hk_length 16
#define com_tx_hk_length 1
#define adcs_hk_length 48
#define seuv_data_length 5

#define com_delay 7
#define eps_delay 7
#define seuv_delay 7
#define interface_delay 7
#define adcs_delay 7

#define adcs_power 0x09
#define gps_power 0x10
#define seuv_power 0x24
/*----------------FS parameters------------------*/
#define No_Error		0
#define Error          1
#define inms_data_length 196
#define wod_length 232
#define seuv_length 37
#define hk_length 139
/*----------------AX.25 2ed header------------------*/
#define AX25_2ed_size 5
/*----------------CCSDS parameters------------------*/
#define TM_NONDATA_SIZE 16
#define ERR_MISSING_PARAMETER 2
#define ERR_CCSDS_TOO_MUCH_DATA 3
#define ERR_CCSDS_BUFFER_TOO_SMALL 4
#define TM_HEADERS_SIZE 14
#define ERR_SUCCESS 0
#define ERR_I2C_FAIL 5
#define ERR_SIZE_ERROR 7
#define TM_S1_SUCCESS_SIZE 4
#define TM_S1_FAILURE_SIZE 6
#define CCSDS_T1_TELECOMMAND_VERIFICATION 0x10
#define CCSDS_T1_ACCEPTANCE_FAIL 0x1F
#define CCSDS_S3_ACCEPTANCE_SUCCESS 0x30
#define CCSDS_S3_COMPLETE_SUCCESS 0x35
#define CCSDS_S3_COMPLETE_FAIL 0x36
#define CCSDS_ERR_ILLEGAL_TYPE 0xFF
#define I2C_SEND_ERROR 0xA0
#define I2C_READ_ERROR 0xA1
#define CCSDS_PACKET_ERROR 0xB0
/*----------------APID----------------*/
// Each downlink type shall have a dedicated APID
#define inms_apid 1
#define phoenix_hk_apid 2
#define seuv_apid 3
#define wod_apid 5

#define eps_apid 8
#define com_apid 9
#define obc_apid 10
#define adcs_apid 11

/*---------------SID---------------*/
#define inms_sid 1
#define seuv_sid 2
#define phoenix_hk_sid 4
#define wod_sid 5

#define eps_hk_sid 6
#define com_hk_sid 7
#define adcs_hk_sid 8

#define adcs_tmp_sid 0xF0   //-----------------

/*---------------Service Type---------------*/
#define T129_ADCS 129
#define T3_SYS_CONF 3


/*------Subsystem Control Functions-----*/

	/* device code:
	 * ADCS = 1
	 * GPS = 2
	 * SEUV = 3
	 * INMS = 4
	 *           */
#define ON 1
#define OFF 0
void power_control(int device,int stats);
void deploy_antenna();
void power_OFF_ALL();

#endif /* SUBSYSTEM_H_ */
