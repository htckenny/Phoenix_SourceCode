#include <dev/i2c.h>
#include <util/timestamp.h>
#include <util/hexdump.h>
#include <freertos/FreeRTOS.h>
#include <string.h>
#include <nanomind.h>
#include <csp/csp_endian.h>
#include <time.h>
/* Self defined header file*/
#include "subsystem.h"
#include "parameter.h"
#include "tele_function.h"
#include "fs.h"
#include "task_SEUV.h"

extern void vTaskSchedule(void * pvParameters);

void decodeService11(uint8_t subType, uint8_t *telecommand) {
	uint8_t completionError = ERR_SUCCESS;
	// note: parameter start from telecommand[9]
	/*------------------------------------------Telecommand-----------------------------------*/
#define Enable_Telecommand_Release		1
#define Disable_Telecommand_Release		2
#define Reset_Command_Schedule 			3
#define Insert_Telecommand				4
#define Delete_Telecommand				6
#define Time_Shifting					15
#define Dump_Command					17

	uint16_t packet_length = (telecommand[4] << 8 ) + telecommand[5];

	uint8_t paras[180] ={0};
	paras[0] = packet_length;
	printf("%d\n", paras[0]);
	if ((packet_length - 4) > 0)
		memcpy(&paras[1], telecommand + 9, packet_length - 4); // !!!!!!!!!!!!!!!!!!!!!!!!
	switch (subType) {

	//send acceptance report

	/*---------------ID:1 Enable_Telecommand_Release----------------*/
	case Enable_Telecommand_Release:
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  // acceptance success

		if (schedule_task_flag == 0) {
			xTaskCreate(vTaskSchedule, (const signed char*) "Sched", 1024 * 4, NULL, 3, &schedule_task);
			schedule_task_flag = 1 ;
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); //send COMPLETE_success report
		}
		else {
			printf("The task has already exist.\n");
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); //send COMPLETE_success report
		}
		break;

	/*---------------ID:2 Disable_Telecommand_Release----------------*/
	case Disable_Telecommand_Release:
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  // acceptance success
		if (schedule_task_flag == 1) {
			vTaskDelete(schedule_task);
			schedule_task_flag = 0;
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); //send COMPLETE_success report
		}
		else {
			printf("The task has not been activated yet!\n");
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); //send COMPLETE_success report
		}
		break;

	/*---------------ID:3 Reset_Command_Schedule----------------*/
	case Reset_Command_Schedule:
		// It shall clear all entries in the command schedule. The command schedule shall be disabled.

		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  // acceptance success
		if (schedule_reset() == Error) {
			printf("schedule reset failed\n");
			completionError = FS_IO_ERR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError); //send complete fail
			// It shall reset the scheduling event information
		}
		else {
			printf("schedule reset success!!\n");
			schedule_new_command_flag = 1;
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); //send COMPLETE_success report
		}

		break;

	/*---------------ID:4 Insert_Telecommand----------------*/
	case Insert_Telecommand:
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  // acceptance success

		// schedule_write need to check
		if (schedule_write(paras) == Error) {
			completionError = FS_IO_ERR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError); //send complete fail
		}
		else {
			schedule_new_command_flag = 1 ;
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); //send COMPLETE_success report
		}
		break;

	/*---------------ID:6 Delete_Telecommand----------------*/
	case Delete_Telecommand:
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  // acceptance success
		if (schedule_delete(telecommand[9], telecommand) == Error) {
			completionError = FS_IO_ERR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError); //send complete fail
		}
		else {
			schedule_new_command_flag = 1 ;
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); //send COMPLETE_success report
		}
		break;

	/*---------------ID:15 Time_Shifting----------------*/
	case Time_Shifting:
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  // acceptance success
		if (schedule_shift(telecommand + 9) == Error) {
			completionError = FS_IO_ERR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError); //send complete fail
		}
		else {
			schedule_new_command_flag = 1 ;
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); //send COMPLETE_success report
		}

		break;
	/*---------------ID:17 Dump_Command----------------*/
	case Dump_Command:
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  // acceptance success
		if (schedule_dump() == Error) {
			completionError = FS_IO_ERR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError); //send complete fail
		}
		else {
			schedule_new_command_flag = 1 ;
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); //send COMPLETE_success report
		}

		break;
	default:
		break;
	}
	/*send COMPLETE_success report*/
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);

}

