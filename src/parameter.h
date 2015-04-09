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

xTaskHandle init_task;
xTaskHandle wod_task;
xTaskHandle adcs_task;
xTaskHandle seuv_task;
xTaskHandle hk_task;
xTaskHandle inms_task_receive;
xTaskHandle inms_task_current;
xTaskHandle inms_task_error;
xTaskHandle inms_task;

uint8_t mode_status_flag;
uint8_t beacon_period;
uint8_t sun_light_flag;
uint8_t adcs_done_flag;

typedef struct __attribute__((packed)) {

	/* System Configuration */
	uint8_t first_flight;
	uint8_t shutdown_flag;
	uint8_t adcs_function;  //H1-47+48
	uint8_t inms_function;
	uint8_t com_function;
	uint8_t seuv_function;  //H1-51+52
	uint8_t gps_function;   //h1-50

	/*  fs related  */

	uint32_t wod_store_count;
	uint32_t inms_store_count;
	uint32_t seuv_store_count;
	uint32_t hk_store_count;

	/* COM sequence count */
	uint16_t obc_packet_sequence_count;
	uint16_t inms_packet_sequence_count;
	uint16_t seuv_packet_sequence_count;
	uint16_t wod_packet_sequence_count;
	uint16_t phoenix_hk_packet_sequence_count;
	uint8_t ax25_sequence_count;
	uint8_t tc_count;





	 /* battery*/
	uint16_t vbat_recover_threshold;
	uint16_t vbat_safe_threshold;

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

} adcs_para_t;

adcs_para_t adcs_para;


#endif /* PARAMETER_H_ */
