/*
 * parameter.h
 *
 *  Created on: 2014/11/20
 *      Author: rusei
 */
#ifndef PARAMETER_H_
#define PARAMETER_H_
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

xTaskHandle mode_task;
xTaskHandle bat_check_task;
xTaskHandle com_task;
xTaskHandle wod_task;
xTaskHandle beacon_task;

xTaskHandle init_task;
xTaskHandle adcs_task;
xTaskHandle seuv_task;
xTaskHandle eop_task;
xTaskHandle hk_task;

xTaskHandle inms_error_handle;
xTaskHandle inms_current_moniter;
xTaskHandle inms_temp_moniter;
xTaskHandle inms_task;
xTaskHandle inms_task_receive;

xTaskHandle schedule_task;
xTaskHandle seuv_cm_task;
xTaskHandle Anom_mon_task;
xTaskHandle format_task;
xTaskHandle gps_task;

#define safe_mode 0
#define init_mode 1
#define adcs_mode 2
#define payload_mode 3

typedef struct __attribute__((packed)) {
	uint8_t mode_task;
	uint8_t bat_check_task;
	uint8_t com_task;
	uint8_t wod_task;
	uint8_t beacon_task;

	uint8_t init_task;
	uint8_t adcs_task;
	uint8_t seuv_task;
	uint8_t eop_task;
	uint8_t hk_task;

	uint8_t inms_error_handle;
	uint8_t inms_current_moniter;
	uint8_t inms_temp_moniter;
	uint8_t inms_task;
	uint8_t inms_task_receive;

	uint8_t schedule_task;
	uint8_t seuv_cm_task;
	uint8_t Anom_mon_task;
	uint8_t	gps_task;
} status_frame_t;
status_frame_t status_frame;

typedef struct __attribute__((packed)) {
	uint8_t mode_status_flag;						/* Flag indicates that which mode is the Cubesat in */
	uint8_t sun_light_flag;							/* Flag indicates that the Cubesat is under sun light */
	uint32_t reboot_count;							/* Counts of the time C&DH reboot */
	uint16_t interface_3V3_current;					/* Interface Board 3.3 V bus current for INMS */
	uint16_t interface_5V_current;					/* Interface Board 5.0 V bus current for INMS */
	uint16_t interface_temp;						/* Interface Board temperature */
	uint16_t interface_thermistor;					/* Interface Board thermistor temperature */
} hk_frame_t;
hk_frame_t HK_frame;


typedef struct __attribute__((packed)) {
	uint8_t mode;
	uint8_t batVoltage;
	uint8_t batCurrent;
	uint8_t bus3v3Current;
	uint8_t bus5v0Current;
	uint8_t tempComm;
	uint8_t tempEps;
	uint8_t tempBat;

} beacon_frame_t;
beacon_frame_t beacon_frame;

uint8_t schedule_new_command_flag;
uint8_t schedule_unlink_flag;
uint8_t idleunlocks;
uint8_t Test_Script;
uint8_t magnetometer_deploy;
uint8_t abort_transfer_flag;
uint8_t SD_lock_flag;
uint8_t i2c_lock_flag;
uint8_t use_GPS_header;
uint8_t inms_tm_status;
uint8_t seuv_with_INMS;
uint8_t INMS_power_status;

uint8_t decode_command;
uint32_t lastCommandTime;
uint8_t ADCS_DEBUG;
typedef struct __attribute__((packed)) {

	/* System Configuration */
	uint8_t first_flight;           	/* During early orbit, this flag should be 1,
	                                	   Finished early orbit, this flag should be 0*/
	uint8_t ant_deploy_flag;       		// default = 0, if deployed, = 1
	uint8_t shutdown_flag;				// if this flag = 1, COM board will not send any packet to GS
	uint8_t com_bit_rates; 				// (0x01)= 1200bps, (0x08)= 9600bps
	uint8_t use_IFB;

	/* FS related */
	uint8_t SD_partition_flag;
	uint8_t crippled_Mode;

	/*  seuv related  */
	uint8_t seuv_mode;					/*	(0x02) = Mode B always work
											(0x03) = Mode Off
											(0x04) = Mode Auto             */
	uint8_t seuv_period;				// period to collect a SEUV data, default = 60 (second)
	uint8_t seuv_sample_rate;			// how many samples to take when collect a SEUV data
	uint8_t seuv_ch1_G1_conf;
	uint8_t seuv_ch2_G1_conf;
	uint8_t seuv_ch3_G1_conf;
	uint8_t seuv_ch4_G1_conf;
	uint8_t seuv_ch1_G8_conf;
	uint8_t seuv_ch2_G8_conf;
	uint8_t seuv_ch3_G8_conf;
	uint8_t seuv_ch4_G8_conf;


	uint8_t hk_collect_period;      	// period to collect a house keeping data, default = 60 (second)
	uint8_t beacon_period;         	 	// beacon period while not in early orbit mode, default = 30(s)
	uint32_t reboot_count;        	  	// how many times C&DH has reboot
	uint32_t GS_threshold;

	/* Protocol sequence count */
	uint16_t obc_packet_sequence_count;	//used in packing CCSDS header apid
	uint8_t ax25_sequence_count;		//used in packing ax25  2nd header
	uint8_t tc_count;					//used in packing ax25  2nd header

	/* battery*/
	uint16_t vbat_recover_threshold;	// Leave safe mode threshold(mV), should be 7000 in default
	uint16_t vbat_safe_threshold;		// Enter safe Mode threshold(mV), should be 7500 in default

	/* Schedule related */
	uint8_t schedule_series_number;

	/* Error Packet related */
	uint16_t ErrSerialNumber;

	/* INMS related */
	uint8_t inms_status;
	uint16_t INMS_timeout;

	/* Image related */
	uint8_t image_lastPartNum;
	uint16_t image_lastPartLen;

} parameter_t;
parameter_t parameters;

typedef struct __attribute__((packed)) {

	uint32_t packettime;
	uint8_t samples;
	float ch1AVG;
	float ch1STD;
	float ch2AVG;
	float ch2STD;
	float ch3AVG;
	float ch3STD;
	float ch4AVG;
	float ch4STD;

} seuv_frame_t;
seuv_frame_t seuvFrame;


typedef struct __attribute__((packed)) {

	uint8_t strategy;
	uint8_t mag_deploy_status_flag;
} adcs_para_t;
adcs_para_t adcs_para;


typedef struct __attribute__((packed)) {
	uint32_t packet_number;
	int16_t T1; 		// EPS temp
	int16_t T2; 		// EPS temp
	int16_t T3; 		// EPS temp
	int16_t T4; 		// EPS temp
	uint16_t T5;  		// COM temp
	uint16_t T6; 		// Antenna temp
	int16_t T7; 		// ADCS temp
	int16_t T8; 		// ADCS Rate Sensor & Magnetometer temp
	uint16_t T9;  		// OBC temp
	uint16_t T10; 		// interface temp
	uint16_t T11; 		// interface INMS temp
	uint16_t Vbat;

} thermal_frame_t;
thermal_frame_t ThermalFrame;

/* Structure for GPS time */
typedef struct {
	int		week;		/* Week number */
	double	sec;		/* Elapsed time [s] */
}	wtime;

uint8_t Tdata[178];

#endif /* PARAMETER_H_ */
