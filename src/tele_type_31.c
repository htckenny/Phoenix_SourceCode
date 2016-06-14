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
#define file_save				4				/* File save operation */
#define file_download			5				/* File download operation */
#define file_remove				6				/* File remove operation */

void decodeService31(uint8_t subType, uint8_t*telecommand) {
	uint8_t completionError = I2C_SEND_ERROR;
	uint16_t para_length = (telecommand[4] << 8) + telecommand[5] - 4;
	uint8_t paras[180];
	uint8_t txBuffer[13];
	uint8_t rxBuffer[256];
	char full_path[9] = {0};

	if (para_length > 0)
		memcpy(&paras, telecommand + 9, para_length);
	switch (subType) {

	/*---------------  ID:1 Capture and save image  ----------------*/
	case capture_save_photo:
		if (para_length == 10)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		txBuffer[0] = 134;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, 5, adcs_delay) == E_NO_ERR) {
			txBuffer[0] = 5;
			txBuffer[1] = rxBuffer[0];
			txBuffer[2] = rxBuffer[1];
			txBuffer[3] = 1 ;
			txBuffer[4] = rxBuffer[3];
			txBuffer[5] = rxBuffer[4];
			i2c_master_transaction_2(0, adcs_node, &txBuffer, 6, 0, 0, adcs_delay);
		}
		vTaskDelay(1 * delay_time_based);
		txBuffer[0] = 110;
		memcpy(&txBuffer[1], paras, para_length);
		hex_dump(&txBuffer[0], 11);
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
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, para_length + 1, &rxBuffer, 2, adcs_delay) == E_NO_ERR) {
			hex_dump(&rxBuffer, 2);
			SendPacketWithCCSDS_AX25(&rxBuffer, 2, adcs_apid, 31, subType);
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
				SendPacketWithCCSDS_AX25(&rxBuffer, 22, obc_apid, 31, subType); // Send to ground
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
	/*---------------  ID:4 Save the file from ADCS to OBC  ----------------*/
	case file_save:
		if (para_length == 8)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		memcpy(&full_path[0], &paras[0], 8);
		photoe_delete();
		if (photo_save(full_path) == No_Error) {
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		}
		else {
			completionError = FS_IO_ERR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
		break;
	/*---------------  ID:5 File download operation  ----------------*/
	case file_download:
		if (para_length == 0)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		uint8_t photo_buffer[190];
		uint16_t count ;
		uint8_t photo_last_size[1];
		count = photo_count(photo_last_size);
		for (int i = 0; i < count; i++) {
			if (photo_downlink(i, photo_buffer, 190) == No_Error) {
				printf("\t %d\n", i);
				SendPacketWithCCSDS_AX25(&photo_buffer, 190, obc_apid, 31, subType);
			}
			else {
				completionError = FS_IO_ERR;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
				break;
			}
			vTaskDelay(0.5 *delay_time_based);
		}
		if (photo_downlink(count, photo_buffer, photo_last_size[0]) == No_Error) {
			SendPacketWithCCSDS_AX25(&photo_buffer, photo_last_size[0], obc_apid, 31, subType);
		}
		else {
			completionError = FS_IO_ERR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			break;
		}
		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		break;
	/*---------------  ID:6 File remove operation   ----------------*/
	case file_remove:
		if (para_length == 8)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		memcpy(&full_path[0], &paras[0], 8);
		full_path[8] = 0;
		printf("%s\n", full_path);
		if (photo_remove(full_path) == No_Error) {
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		}
		else {
			completionError = FS_IO_ERR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
		break;
	/*---------------- Otherwise ----------------*/
	default:
		sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
		break;
	}
}
