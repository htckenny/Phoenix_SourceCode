/*
 * tele_type_31.c
 *
 *  Created on: 	2016/05/20
 *  Last updated: 	2016/05/20
 *  Author: 		Kenny Huang
 */

#include <string.h>
#include <nanomind.h>
#include <dev/i2c.h>
#include <util/hexdump.h>
#include <freertos/FreeRTOS.h>

/* Self defined header file*/
#include "subsystem.h"
#include "parameter.h"
#include "tele_function.h"
#include "fs.h"

/* Definition of the subtype */
#define capture_save_photo		1				/* Capture and save image */
#define status_of_capture		2				/* Status of image capture and save operation */
#define file_list_process		3				/* File list operation */
#define file_download_process	4				/* File download operation */

void decodeService31(uint8_t subType, uint8_t*telecommand) {
	uint8_t completionError = I2C_SEND_ERROR;
	uint16_t para_length = (telecommand[4] << 8) + telecommand[5] - 4;
	uint8_t paras[180];
	uint8_t txBuffer[13];
	uint8_t rxBuffer[256];

	if (para_length > 0)
		memcpy(&paras, telecommand + 9, para_length);
	switch (subType) {

	/*---------------  ID:1 Capture and save image  ----------------*/
	case Capture_And_Save_Image:
		if (para_length == 10)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		txBuffer[0] = 110;
		memcpy(&txBuffer[1], paras, para_length);
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, para_length + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
		break;
	/*---------------  ID:2 Status of image capture and save operation  ----------------*/
	case status_of_capture:
		if (para_length == 0)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		txBuffer[0] = 230;
		memcpy(&txBuffer[1], paras, para_length);
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, para_length + 1, &rxBuffer, 2, adcs_delay) == E_NO_ERR) {
			hex_dump(&rxBuffer, 2);
			SendPacketWithCCSDS_AX25(&rxBuffer, 2, adcs_apid, 32, subType);
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
		break;
	/*---------------  ID:3 File list operation  ----------------*/
	case file_list_process:
		if (para_length == 0)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		txBuffer[0] = 114;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, 0, 0, adcs_delay) == E_NO_ERR) {
			printf("reset list\n");
		}
		else
			printf("failed to reset list\n");
		while (1) {
			txBuffer[0] = 240;
			if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, 22, adcs_delay) == E_NO_ERR) {
				printf("get file information\n");
				hex_dump(&rxBuffer, 22);
				// SendPacketWithCCSDS_AX25(&rxBuffer, 16, adcs_apid, 32, subType); // Send to ground
				if (rxBuffer[0] == 2)  // Check this value
					break;
				else {
					txBuffer[0] = 115;
					i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, 0, 0, adcs_delay);
				}
			}
			else
				printf("failed get file information\n");
			vTaskDelay(0.5 * delay_time_based);
		}
		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);

		break;
	/*---------------  ID:4 File download operation  ----------------*/
	case file_download_process:
		if (para_length == 8)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		char full_path[9];
		memcpy(full_path, &paras[0], 8);
		// photo_download("IMG00011");
		photo_download(full_path);
		break;

	/*---------------- Otherwise ----------------*/
	default:
		sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);

		break;
	}
}
