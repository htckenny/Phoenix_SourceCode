#include <freertos/FreeRTOS.h>
#include "parameter.h"
#include "crc16.h"
#include <dev/i2c.h>
#include "Tele_function.h"
#include <util/hexdump.h>
#include "fs.h"
#include <string.h>
#include <io/nanomind.h>
#include <csp/csp_endian.h>
#include <time.h>
#include "SEUV_Task.h"
#include "Tele_function.h"
#include "subsystem.h"



void decodeService15(uint8_t subType, uint8_t*telecommand) {


	uint32_t T1;
	uint32_t T2;
	uint8_t completionError = ERR_F_READ;
// note: parameter start from telecommand[9]
	/*------------------------------------------Telecommand-----------------------------------*/


#define download_data 9
#define delete_data 10


	uint16_t packet_length = (256 * telecommand[4]) + telecommand[5];

	uint8_t paras[180];
	if ((packet_length - 4) > 0)
		memcpy(&paras, telecommand + 9, packet_length - 4); // !!!!!!!!!!!!!!!!!!!!!!!!
	switch (subType) {

	/*---------------ID:9 download_data----------------*/
	case download_data:
		printf("  download_data   \n");
		if (packet_length > 10) {


			memcpy(&T1, &paras[2], 4);
			memcpy(&T2, &paras[7], 4);

			if (paras[1] == 1) {
				sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
				if (downlink_data_between_t(paras[0], T1, T2) != No_Error) {
					sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError); //send complete fail
					break;
				}
			}
			else if (paras[1] == 2) {
				sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
				if (downlink_data_before_t(paras[0], T1) != No_Error) {
					sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError); //send complete fail
					break;
				}
			}
			else if (paras[1] == 3) {
				printf("  download_data  3 \n");
				sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
				if (downlink_data_after_t(paras[0], T1) != No_Error) {
					sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError); //send complete fail
					break;
				}
			}
			else
				sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE); //accept fail


			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); //send COMPLETE_success report
		} else
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE); //accept fail
		break;
	/*---------------ID:10 delete_data----------------*/
	case delete_data:
		if (packet_length > 10) {
			memcpy(&T1, &paras[2], 4);
			memcpy(&T2, &paras[7], 4);

			if (paras[1] == 1) {
				sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
				if (delete_data_between_t(paras[0], T1, T2) != No_Error) {
					sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError); //send complete fail
					break;
				}
			}
			else if (paras[1] == 2) {
				sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
				if (delete_data_before_t(paras[0], T1) != No_Error) {
					sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError); //send complete fail
					break;
				}
			}
			else if (paras[1] == 3) {
				sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
				if (delete_data_after_t(paras[0], T1) != No_Error) {
					sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError); //send complete fail
					break;
				}
			}
			else
				sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE); //accept fail


			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); //send COMPLETE_success report
		} else
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE); //accept fail
		break;

	default:
		sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
		break;

	}
}


