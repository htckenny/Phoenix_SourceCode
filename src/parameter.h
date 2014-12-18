/*
 * parameter.h
 *
 *  Created on: 2014/11/20
 *      Author: rusei
 */
#ifndef PARAMETER_H_
#define PARAMETER_H_
#include <string.h>
#include <inttypes.h>
#include <stdint.h>

uint8_t mode_status_flag;
int virtual_clock;

	/*  fs related  */
	uint32_t wod_downlink_count;
	uint32_t inms_downlink_count;
	uint32_t seuv_downlink_count;
	uint32_t hk_downlink_count;
	uint32_t wod_store_count;
	uint32_t inms_store_count;
	uint32_t seuv_store_count;
	uint32_t hk_store_count;
	/* system */
	uint8_t first_flight;
	uint8_t shutdown_flag;
	 /* battery*/
	uint8_t vbat_threshold;
	uint8_t vbat_safe_threshold;


typedef struct __attribute__((packed)) {

	/*  fs related  */
	uint32_t wod_downlink_count;
	uint32_t inms_downlink_count;
	uint32_t seuv_downlink_count;
	uint32_t hk_downlink_count;
	uint32_t wod_store_count;
	uint32_t inms_store_count;
	uint32_t seuv_store_count;
	uint32_t hk_store_count;
	/* system */
	uint8_t first_flight;
	uint8_t shutdown_flag;
	 /* battery*/
	uint8_t vbat_threshold;
	uint8_t vbat_safe_threshold;

} parameter_t;

parameter_t parameters;

struct parameter_8
{
	/* system */
	uint8_t first_flight;
	uint8_t shutdown_flag;
	 /* battery*/
	uint8_t vbat_threshold;
	uint8_t vbat_safe_threshold;

};

struct parameter_32
{
	/*  fs related  */
	uint32_t wod_downlink_count;
	uint32_t inms_downlink_count;
	uint32_t seuv_downlink_count;
	uint32_t hk_downlink_count;
	uint32_t wod_store_count;
	uint32_t inms_store_count;
	uint32_t seuv_store_count;
	uint32_t hk_store_count;
};

#endif /* PARAMETER_H_ */
