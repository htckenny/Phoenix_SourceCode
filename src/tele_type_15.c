#include <dev/i2c.h>
#include <util/timestamp.h>
#include <util/hexdump.h>
#include <freertos/FreeRTOS.h>
#include <string.h>
#include <nanomind.h>
#include <csp/csp_endian.h>
#include <time.h>
#include <inttypes.h>
/* Self defined header file*/
#include "subsystem.h"
#include "parameter.h"
#include "tele_function.h"
#include "fs.h"
#include "task_SEUV.h"


void decodeService15(uint8_t subType, uint8_t*telecommand) {

	uint32_t T1;
	uint32_t T2;
	uint8_t completionError = ERR_F_READ;
// note: parameter start from telecommand[9]
	/*------------------------------------------Telecommand-----------------------------------*/


#define download_data 9
#define delete_data 10


	uint16_t packet_length = (telecommand[4] << 8) + telecommand[5] -4;

	uint8_t paras[180];
	if ( packet_length > 0)
		memcpy(&paras, telecommand + 9, packet_length ); // !!!!!!!!!!!!!!!!!!!!!!!!
	switch (subType) {

	/*---------------ID:9 download_data----------------*/
	case download_data:
		printf("download_data  \n");
		printf("%d\n", packet_length);

		if (packet_length > 5) {

			memcpy(&T1, &paras[2], 4);
			// printf("be Time 1 = %"PRIu32" \n", T1);
			if (paras[1] == 1){								/* mode is 1 => need T1 and T2 */
				memcpy(&T2, &paras[7], 4);

			}
			else if (paras[1] == 2 || paras[1] == 3){		/* mode is 2 or 3 => only need T1 */
				T2 = 0;
			}
			T1 = csp_ntoh32(T1);
			T2 = csp_ntoh32(T2);
			printf("be Time 1 = %"PRIu32" \n", T1);
			printf("para 0 = %d\n", paras[0]);
			/* PHOENIX HK */
			if (paras[0] == 1) {
				sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
				if (scan_files_Downlink ("0:/HK_DATA", paras[1], T1, T2) != No_Error){
					sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError); //send complete fail
					break;
				}
			}
			/* INMS */
			else if (paras[0] == 2) {
				sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
				if (scan_files_Downlink ("0:/INMS_DATA", paras[1], T1, T2) != No_Error){
					sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError); //send complete fail
					break;
				}
			}
			/* SEUV */
			else if (paras[0] == 3) {
				sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
				if (scan_files_Downlink ("0:/SEUV_DATA", paras[1], T1, T2) != No_Error){
					sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError); //send complete fail
					break;
				}
			}
			/* EOP */
			else if (paras[0] == 4) {
				sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
				if (scan_files_Downlink ("0:/EOP_DATA", paras[1], T1, T2) != No_Error){
					sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError); //send complete fail
					break;
				}
			}
			/* WOD */
			else if (paras[0] == 5) {
				sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
				if (scan_files_Downlink ("0:/WOD_DATA", paras[1], T1, T2) != No_Error){
					sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError); //send complete fail
					break;
				}
			}
			else{
				sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE); //accept fail
			}
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); //send COMPLETE_success report
		} 
		else
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE); //accept fail
		break;
	/*---------------ID:10 delete_data----------------*/
	case delete_data:
		if (packet_length > 5) {

			memcpy(&T1, &paras[2], 4);
			if (paras[1] == 1){								/* mode is 1 => need T1 and T2 */
				memcpy(&T2, &paras[7], 4);
			}
			else if (paras[1] == 2 || paras[1] == 3){		/* mode is 2 or 3 => only need T1 */
				T2 = 0;
			}
			T1 = csp_ntoh32(T1);
			T2 = csp_ntoh32(T2);

			/* PHOENIX HK */
			if (paras[0] == 1) {
				sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
				if (scan_files_Delete ("0:/HK_DATA", paras[1], T1, T2) != No_Error){
					sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError); //send complete fail
					break;
				}
			}
			/* INMS */
			else if (paras[0] == 2) {
				sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
				if (scan_files_Delete ("0:/INMS_DATA", paras[1], T1, T2) != No_Error){
					sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError); //send complete fail
					break;
				}
			}
			/* SEUV */
			else if (paras[0] == 3) {
				sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
				if (scan_files_Delete ("0:/SEUV_DATA", paras[1], T1, T2) != No_Error){
					sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError); //send complete fail
					break;
				}
			}
			/* EOP */
			else if (paras[0] == 4) {
				sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
				if (scan_files_Delete ("0:/EOP_DATA", paras[1], T1, T2) != No_Error){
					sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError); //send complete fail
					break;
				}
			}
			/* WOD */
			else if (paras[0] == 5) {
				sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
				if (scan_files_Delete ("0:/WOD_DATA", paras[1], T1, T2) != No_Error){
					sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError); //send complete fail
					break;
				}
			}
			/* INMS Script */
			else if (paras[0] == 6) {
				sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
				
				for (int i = 0 ; i < 7 ; i++){
					inms_script_delete(i);
				}
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError); //send complete fail
				break;
			}
			else
				sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE); //accept fail

			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); //send COMPLETE_success report
		} 
		else
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE); //accept fail
		break;

	default:
		sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
		break;

	}
}

/**
 * Old Code
 */
			// if (paras[1] == 1) {
			// 	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
			// 	if (delete_data_between_t(paras[0], T1, T2) != No_Error) {
			// 		sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError); //send complete fail
			// 		break;
			// 	}
			// }
			// else if (paras[1] == 2) {
			// 	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
			// 	if (delete_data_before_t(paras[0], T1) != No_Error) {
			// 		sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError); //send complete fail
			// 		break;
			// 	}
			// }
			// else if (paras[1] == 3) {
			// 	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
			// 	if (delete_data_after_t(paras[0], T1) != No_Error) {
			// 		sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError); //send complete fail
			// 		break;
			// 	}
			// }
			