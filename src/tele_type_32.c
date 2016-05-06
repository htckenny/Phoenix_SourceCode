/*
 * tele_type_32.c
 *
 *  Created on: 	2016/04/29
 *  Last updated: 	2016/05/02
 *  Author: 		Kenny Huang
 */

#include <dev/i2c.h>
#include <util/timestamp.h>
#include <util/hexdump.h>
#include <freertos/FreeRTOS.h>
#include <string.h>
#include <nanomind.h>
#include <csp/csp_endian.h>
#include <time.h>
#include <fat_sd/ff.h>
/* Self defined header file*/
#include "subsystem.h"
#include "parameter.h"
#include "tele_function.h"
#include "fs.h"

/* Definition of the subtype */
#define Request_Image_Status		1				/* Request Image's status */
#define Downlink_missing_frame		2				/* Downlink missing part number in LUT */
#define perform_merge				3				/* Merge the part into one file */


extern int image_check(uint8_t last_partNo, uint16_t last_part_length, uint16_t * Error_record, int * total_Errnumber);

void decodeService32(uint8_t subType, uint8_t*telecommand) {
	uint8_t completionError = I2C_SEND_ERROR;
	uint16_t para_length = (telecommand[4] << 8) + telecommand[5] - 4;
	uint8_t paras[180];
	uint16_t Miss_table[256 * 15];
	int total_number[1];

	if (para_length > 0)
		memcpy(&paras, telecommand + 9, para_length);
	switch (subType) {

	/*--------------- ID:1 Request Image Status ----------------*/
	case Request_Image_Status:
		if (para_length == 3)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		image_lastPartNum = paras[0];
		image_lastPartLen = (paras[1] << 8) + paras[2];
		// uint16_t last_length = (paras[1] << 8) + paras[2];
		if (image_check(image_lastPartNum, image_lastPartLen, Miss_table, total_number) == Error) {
			completionError = FS_IO_ERR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
		else {
			SendPacketWithCCSDS_AX25(&total_number[0], 2, obc_apid, 32, subType);
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		}
		break;
	/*--------------- ID:2 Downlink missing part number in LUT ----------------*/
	case Downlink_missing_frame:
		if (para_length == 0)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		int remaining_number = total_number[0] % 90 ;
		for (int i = 0; i < (total_number[0] / 90); i++) {
			SendPacketWithCCSDS_AX25(&Miss_table[i * 90], 2 * 90, obc_apid, 32, subType);
			printf("delay?\n");
		}
		SendPacketWithCCSDS_AX25(&Miss_table[90 * (total_number[0] / 90) ], 2 * remaining_number, obc_apid, 32, subType);

		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		break;
	/*--------------- ID:3 Downlink missing part number in LUT ----------------*/
	case perform_merge:
		if (para_length == 0)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}

		image_merge(image_lastPartNum, image_lastPartLen);
		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		break;
	/*---------------- Otherwise ----------------*/
	default:
		sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);

		break;
	}
}
