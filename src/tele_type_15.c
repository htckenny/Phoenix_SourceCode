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

#define download_data		9
#define delete_data			10
#define download_crippled	11
#define delete_crippled		12
#define abort_transfer		128

extern void beacon_Task(void * pvParameters);

void decodeService15(uint8_t subType, uint8_t*telecommand) {

	uint32_t T1;
	uint32_t T2;
	uint8_t completionError = ERR_F_READ;
	/*------------------------------------------Telecommand-----------------------------------*/

	uint16_t packet_length = (telecommand[4] << 8) + telecommand[5] - 4;
	uint8_t *rxdata = NULL;
	uint8_t paras[180];
	if ( packet_length > 0)
		memcpy(&paras, telecommand + 9, packet_length );
	switch (subType) {

	/*---------------ID:9 download_data----------------*/
	case download_data:
		printf("download_data  \n");

		if (packet_length > 5) {
			if (beacon_task != NULL) {
				vTaskDelete(beacon_task);
				beacon_task = NULL;
			}
			memcpy(&T1, &paras[2], 4);
			/* mode is 1 => need T1 and T2 */
			if (paras[1] == 1) {
				memcpy(&T2, &paras[7], 4);
			}
			/* mode is 2 or 3 => only need T1 */
			else if (paras[1] == 2 || paras[1] == 3) {
				T2 = 0;
			}
			T1 = csp_ntoh32(T1);
			T2 = csp_ntoh32(T2);
			printf("Date type = %d\n", paras[0]);

			if (paras[0] >= 1 && paras[0] <= 5) {
				sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
				/* PHOENIX HK */
				if (paras[0] == 1) {
					if (hk_task != NULL)
						vTaskSuspend(hk_task);
					if (scan_files_Count(fileName_HK[parameters.SD_partition_flag], paras[1], T1, T2) != No_Error) {};
					if (scan_files_Downlink(fileName_HK[parameters.SD_partition_flag], paras[1], T1, T2) != No_Error) {
						sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
						break;
					}
					if (hk_task != NULL)
						vTaskResume(hk_task);
				}
				/* INMS */
				else if (paras[0] == 2) {
					if (inms_task != NULL)
						vTaskSuspend(inms_task);
					if (scan_files_Count(fileName_INMS[parameters.SD_partition_flag], paras[1], T1, T2) != No_Error) {};
					if (scan_files_Downlink(fileName_INMS[parameters.SD_partition_flag], paras[1], T1, T2) != No_Error) {
						sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
						break;
					}
					if (inms_task != NULL)
						vTaskResume(inms_task);
				}
				/* SEUV */
				else if (paras[0] == 3) {
					if (seuv_task != NULL)
						vTaskSuspend(seuv_task);
					if (scan_files_Count(fileName_SEUV[parameters.SD_partition_flag], paras[1], T1, T2) != No_Error) {};
					if (scan_files_Downlink(fileName_SEUV[parameters.SD_partition_flag], paras[1], T1, T2) != No_Error) {
						sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
						break;
					}
					if (seuv_task != NULL)
						vTaskResume(seuv_task);
				}
				/* EOP */
				else if (paras[0] == 4) {
					if (eop_task != NULL)
						vTaskSuspend(eop_task);
					if (scan_files_Count(fileName_EOP[parameters.SD_partition_flag], paras[1], T1, T2) != No_Error) {};
					if (scan_files_Downlink(fileName_EOP[parameters.SD_partition_flag], paras[1], T1, T2) != No_Error) {
						sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
						break;
					}
					if (eop_task != NULL)
						vTaskResume(eop_task);
				}
				/* WOD */
				else if (paras[0] == 5) {
					if (wod_task != NULL)
						vTaskSuspend(wod_task);
					if (scan_files_Count(fileName_WOD[parameters.SD_partition_flag], paras[1], T1, T2) != No_Error) {};
					if (scan_files_Downlink(fileName_WOD[parameters.SD_partition_flag], paras[1], T1, T2) != No_Error) {
						sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
						break;
					}
					if (wod_task != NULL)
						vTaskResume(wod_task);
				}
				if (beacon_task == NULL)
					xTaskCreate(beacon_Task, (const signed char *) "beacon", 1024 * 4, NULL, 2, &beacon_task);
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			}
			else {
				sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			}
		}
		else
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
		break;
	/*---------------ID:10 delete_data----------------*/
	case delete_data:
		if (packet_length > 5) {

			memcpy(&T1, &paras[2], 4);
			/* mode is 1 => need T1 and T2 */
			if (paras[1] == 1) {
				memcpy(&T2, &paras[7], 4);
			}
			/* mode is 2 or 3 => only need T1 */
			else if (paras[1] == 2 || paras[1] == 3) {
				T2 = 0;
			}
			T1 = csp_ntoh32(T1);
			T2 = csp_ntoh32(T2);

			/* PHOENIX HK */
			if (paras[0] == 1) {
				sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
				if (scan_files_Delete (fileName_HK[parameters.SD_partition_flag], paras[1], T1, T2) != No_Error) {
					sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
					break;
				}
			}
			/* INMS */
			else if (paras[0] == 2) {
				sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
				if (scan_files_Delete (fileName_INMS[parameters.SD_partition_flag], paras[1], T1, T2) != No_Error) {
					sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
					break;
				}
			}
			/* SEUV */
			else if (paras[0] == 3) {
				sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
				if (scan_files_Delete (fileName_SEUV[parameters.SD_partition_flag], paras[1], T1, T2) != No_Error) {
					sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
					break;
				}
			}
			/* EOP */
			else if (paras[0] == 4) {
				sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
				if (scan_files_Delete (fileName_EOP[parameters.SD_partition_flag], paras[1], T1, T2) != No_Error) {
					sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
					break;
				}
			}
			/* WOD */
			else if (paras[0] == 5) {
				sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
				if (scan_files_Delete (fileName_WOD[parameters.SD_partition_flag], paras[1], T1, T2) != No_Error) {
					sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
					break;
				}
			}
			/* INMS Script */
			else if (paras[0] == 6) {
				sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);

				for (int i = 0 ; i < 7 ; i++) {
					inms_script_delete_flash(i);
				}
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
				break;
			}
			else
				sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);

			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		}
		else
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
		break;
	/*---------------ID:11 downlink crippled mode data----------------*/
	case download_crippled:
		if (packet_length == 3) {
			if (paras[0] >= 1 && paras[0] <= 5) {
				sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
			}
			if (paras[0] == 1) {
				rxdata = malloc(hk_length);
				for (int i = paras[1]; i < paras[1] + paras[2]; i++) {
					if (hk_read_crippled(i, rxdata) == No_Error) {
						SendDataWithCCSDS_AX25(1, &rxdata[0]);
						vTaskDelay(0.5 * delay_time_based);
					}
					else
						break;
				}
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			}
			else if (paras[0] == 2) {
				rxdata = malloc(inms_data_length);
				for (int i = paras[1]; i < paras[1] + paras[2]; i++) {
					if (inms_data_read_crippled(i, rxdata) == No_Error) {
						SendDataWithCCSDS_AX25(2, &rxdata[0]);
						vTaskDelay(0.5 * delay_time_based);
					}
					else
						break;
				}
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			}
			else if (paras[0] == 3) {
				rxdata = malloc(seuv_length);
				for (int i = paras[1]; i < paras[1] + paras[2]; i++) {
					if (seuv_read_crippled(i, rxdata) == No_Error) {
						SendDataWithCCSDS_AX25(3, &rxdata[0]);
						vTaskDelay(0.5 * delay_time_based);
					}
					else
						break;
				}
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			}
			else if (paras[0] == 4) {
				rxdata = malloc(eop_length);
				for (int i = paras[1]; i < paras[1] + paras[2]; i++) {
					if (eop_read_crippled(i, rxdata) == No_Error) {
						SendDataWithCCSDS_AX25(4, &rxdata[0]);
						vTaskDelay(0.5 * delay_time_based);
					}
					else
						break;
				}
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			}
			else if (paras[0] == 5) {
				rxdata = malloc(wod_length);
				for (int i = paras[1]; i < paras[1] + paras[2]; i++) {
					if (wod_read_crippled(i, rxdata) == No_Error) {
						SendDataWithCCSDS_AX25(5, &rxdata[0]);
						vTaskDelay(0.5 * delay_time_based);
					}
					else
						break;
				}
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			}
			else {
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
			if (rxdata != NULL)
				free(rxdata);
		}
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
		}

		break;
	/*---------------ID:12 delete crippled mode data----------------*/
	case delete_crippled:
		if (packet_length == 1) {
			if (paras[0] >= 1 && paras[0] <= 5) {
				sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
				crippled_data_delete(paras[0]);
			}
			else {
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
		}
		break;

	/*---------------ID:128 abort onging transfer----------------*/
	case abort_transfer:
		if (packet_length == 0)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		printf("Execute Type 15 Sybtype 128, Abort transmitting \r\n");
		abort_transfer_flag = 1;
		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		break;
	default:
		sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
		break;
	}
}