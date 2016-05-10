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
#define Configure_Image				1				/* Configure Image's related parameter */
#define Request_part_Status			2				/* Request Image part's status */
#define Request_Image_Status		3				/* Request Image's status */
#define perform_merge				4				/* Merge the part into one file */
#define move_to_flash				5				/* Move the image from SD card to FLASH memory */

void decodeService32(uint8_t subType, uint8_t*telecommand) {
	uint8_t completionError = I2C_SEND_ERROR;
	uint16_t para_length = (telecommand[4] << 8) + telecommand[5] - 4;
	uint8_t paras[180];
	uint8_t Miss_Part_table[255];
	uint8_t Miss_table[20];
	int total_number[1];

	if (para_length > 0)
		memcpy(&paras, telecommand + 9, para_length);
	switch (subType) {

	/*--------------- ID:1 Configure Image's related parameter ----------------*/
	case Configure_Image:
		if (para_length == 3)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		image_lastPartNum = paras[0];
		image_lastPartLen = (paras[1] << 8) + paras[2];
		printf("%d %d\n", image_lastPartNum, image_lastPartLen);
		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		break;
	/*--------------- ID:2 Request Image part's status ----------------*/
	case Request_part_Status:
		if (para_length == 1)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}

		if (image_part_check(paras[0], Miss_Part_table, total_number) == Error) {
			completionError = FS_IO_ERR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
		else {
			SendPacketWithCCSDS_AX25(&total_number[0], 2, obc_apid, 32, subType);
			if (total_number[0] != 0) {
				if (total_number[0] > 190) {
					SendPacketWithCCSDS_AX25(&Miss_Part_table[0], 190, obc_apid, 32, subType);
					SendPacketWithCCSDS_AX25(&Miss_Part_table[190], total_number[0] - 190, obc_apid, 32, subType);
				}
				else {
					SendPacketWithCCSDS_AX25(&Miss_Part_table[0], total_number[0], obc_apid, 32, subType);
				}
			}
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		}
		break;
	/*--------------- ID:3 Request Image Status ----------------*/
	case Request_Image_Status:
		if (para_length == 0)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		if (image_check(image_lastPartNum, image_lastPartLen, Miss_table, total_number) == Error) {
			completionError = FS_IO_ERR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
		else {
			SendPacketWithCCSDS_AX25(&total_number[0], 2, obc_apid, 32, subType);
			if (total_number[0] != 0)
				SendPacketWithCCSDS_AX25(&Miss_table[0], total_number[0], obc_apid, 32, subType);
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		}
		break;
	/*--------------- ID:4 Merge the part into one file ----------------*/
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
	/*--------------- ID:5 Move the image from SD card to FLASH memory ----------------*/
	case move_to_flash:
		if (para_length == 0)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		if (image_move() == Error) {
			completionError = FS_IO_ERR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
		else {
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		}
		break;
	/*---------------- Otherwise ----------------*/
	default:
		sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);

		break;
	}
}
