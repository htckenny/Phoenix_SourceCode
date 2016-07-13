#include <dev/i2c.h>
#include <util/timestamp.h>
#include <util/hexdump.h>
#include <freertos/FreeRTOS.h>
#include <string.h>
#include <nanomind.h>
#include <csp/csp_endian.h>
#include <time.h>
#include <fat_sd/ff.h>
#include <inttypes.h>
/* Self defined header file*/
#include "subsystem.h"
#include "parameter.h"
#include "tele_function.h"
#include "fs.h"

/* Definition of the subtype */
#define reboot						3				/* Reboot the whole system */
#define Sync_Time					4				/* Sync the OBC time */
#define Sync_Time_With_GPS			5				/* Sync the OBC time with GPS */
#define ShutdownTransmitter			6				/* Shut Down Trasmitter */
#define ResumeTransmitter			7				/* Resume Transmitter */
#define Enter_Safe_Threshold		8				/* Set Safe mode enter threshold (default = 7000) */
#define Leave_Safe_Threshold		9				/* Set Safe mode leave threshold (default = 7500) */
#define I2C_COMMAND					10				/* (!)very useful, but use it carefully */
#define para_to_default 			11				/* Set parameters to default */
#define set_tx_rates 				12				/* Set tx rate */
#define power_on_target 			14				/* Power ON specific system */
#define power_off_target 			15				/* Power OFF specific system */
#define enter_specific_mode			16				/* Enter specifc mode for certain purpose */
#define INMS_Script_State			17				/* Set INMS handler to enable/ disable */
#define SD_partition 				18				/* Set which partition would like to read from */
#define INMS_timeout_change			19				/* Set INMS timeout value */
#define INMS_restart				20				/* Restart INMS script handler, do this after upload new script*/
#define Enable_GPS_header			21				/* use GPS to get position data instead of ADCS */
#define Reset_Error_Report			22				/* Reset Error Report, should be conducted after successful downlink of Error Report */
#define Manual_Heater_Switch		23				/* Switch ON/OFF EPS heater in case of auto switch has problem */
#define switchInterfaceBoard		24				/* switch the use of interface board, in case the IFB failed in cold platue */
#define GS_timeout_change			25				/* Set GS anomaly's threshold  value */
#define Activate_GPS_process		26				/* Activate GPS process, and record the information */
#define SD_card_format				30				/* Format SD card, and create default folders */
#define enter_crippled_mode			32				/* Enter Crippled mode, change storage place to flash memory */
#define test_task_hang				33

extern void vTaskinms(void * pvParameters);
extern struct tm wtime_to_date(wtime wt);
extern void little2big_32(uint8_t * input_data);
extern void task_format(void * pvParameters);
extern void GPS_task(void* pvParameters);


void decodeService8(uint8_t subType, uint8_t*telecommand) {
	uint8_t txBuffer[200];
	timestamp_t t;
	time_t tt;
	uint8_t completionError = I2C_SEND_ERROR;
	uint16_t para_length = (telecommand[4] << 8) + telecommand[5] - 4;
	uint8_t paras[180];
	uint16_t threshold;
	uintptr_t label;

	if (para_length > 0)
		memcpy(&paras, telecommand + 9, para_length);
	switch (subType) {
	/*--------------- ID:3 reboot ----------------*/
	case reboot:
		if (para_length == 0)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		printf("Execute Type 8 Sybtype 3 , Rebooting \r\n");
		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		vTaskDelay(1 * delay_time_based);
		txBuffer[0] = 20;
		i2c_master_transaction_2(0, eps_node, &txBuffer, 1, 0, 0, eps_delay);

		break;
	/*---------------ID:4 Sync_Time----------------*/
	case Sync_Time:
		if (para_length == 5)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		printf("Execute Type 8 Sybtype 4 , Sync_Time \r\n");
		t.tv_sec = 0;
		t.tv_nsec = 0;
		memcpy(&t.tv_sec, &paras[0], 4);
		t.tv_sec = csp_ntoh32(t.tv_sec);
		t.tv_sec += 946684800 ;

		obc_timesync(&t, 1000);
		tt = t.tv_sec ;
		lastCommandTime = t.tv_sec;
		printf("OBC time Sync by telecommand to : %s\r\n", ctime(&tt));
		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		break;
	/*---------------ID:5 Sync the OBC time with GPS ----------------*/
	case Sync_Time_With_GPS:
		if (para_length == 0)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		uint8_t rxbuf[6];
		printf("Execute Type 8 Sybtype 5 ,Sync Time with GPS \r\n");
		txBuffer[0] = 169; /* Raw GPS Time */
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxbuf[0], 6, adcs_delay) == E_NO_ERR) {
			wtime wt;
			struct tm tmbuf;
			time_t t_of_day;
			uint16_t gps_week;
			uint32_t gps_second;
			/* Get week number and elapsed time */
			memcpy(&gps_week, &rxbuf[0], 2);
			memcpy(&gps_second, &rxbuf[2], 4);

			wt.week = gps_week;
			wt.sec = gps_second / 1000;
			/* Display week number and elapsed time */
			// printf("wn=%d, sec=%.3f\n", wt.week, wt.sec);

			/* Display time in the form of "YYYY/MM/DD HH:MM:SS" */
			tmbuf = wtime_to_date(wt);
			printf("%4.4d%2.2d%2.2d_%2.2d%2.2d%2.2d\n",
			       tmbuf.tm_year + 1900, tmbuf.tm_mon + 1, tmbuf.tm_mday,
			       tmbuf.tm_hour, tmbuf.tm_min, tmbuf.tm_sec);

			/* Construct struct time to epoch seconds */
			t_of_day = mktime(&tmbuf);
			ctime(&t_of_day);
			t.tv_nsec = 0;
			t.tv_sec = t_of_day ;
			obc_timesync(&t, 1000);
			tt = t.tv_sec;
			lastCommandTime = t.tv_sec;
			printf("OBC time Sync by GPS to : %s\r\n", ctime(&tt));
		}

		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		break;
	/*---------------ID:6 ShutdownTransmitter----------------*/

	case  ShutdownTransmitter:
		if (para_length == 0)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		printf("Execute Type 8 Sybtype 6 , ShutdownTransmitter \r\n");
		parameters.shutdown_flag = 1;
		para_w_flash();
		printf("Shutdown Command Detected!! \r\n");
		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		break;

	/*---------------ID:7 ResumeTransmitter----------------*/
	case  ResumeTransmitter:
		printf("Execute Type 8 Sybtype 7 , Resume Transmitter \r\n");
		parameters.shutdown_flag = 0;
		para_w_flash();
		printf("Shutdown Resume Command Detected!! \r\n");
		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		break;

	/*---------------ID:8 Enter_Safe_Threshold----------------*/
	case  Enter_Safe_Threshold:
		if (para_length == 2)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		printf("Execute Type 8 Sybtype 8 , Enter_Safe_Threshold \r\n");

		memcpy(&threshold, &paras, 2);
		threshold = csp_ntoh16(threshold);
		if (threshold >= 6500 && threshold <= 8300) {
			memcpy(&parameters.vbat_safe_threshold, &threshold, 2);
			para_w_flash();
			printf("Enter_Safe_Threshold = %d mV\n", parameters.vbat_safe_threshold);
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		}
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
		break;
	/*---------------ID:9 Leave_Safe_Threshold----------------*/
	case  Leave_Safe_Threshold:
		if (para_length == 2)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		printf("Execute Type 8 Sybtype 9 , Leave_Safe_Threshold \r\n");

		memcpy(&threshold, &paras, 2);
		threshold = csp_ntoh16(threshold);
		if (threshold >= 6500 && threshold <= 8300) {
			memcpy(&parameters.vbat_recover_threshold, &threshold, 2);
			para_w_flash();
			printf("Leave_Safe_Threshold = %d mV\n", parameters.vbat_recover_threshold);
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		}
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
		break;

	/*---------------ID:11 para_to_default----------------*/
	case  para_to_default:
		if (para_length == 0)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		printf("Execute Type 8 Sybtype 11 , para_to_default \r\n");
		para_d_flash();
		parameter_init();

		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		break;
	/*---------------ID:12 set_tx_rates      ----------------*/
	case set_tx_rates:
		if (para_length == 1)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		printf("Execute Type 8 Sybtype 12 ,set_tx_rate \r\n");

		if (set_tx_rate(paras[0]) != Error) {
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			parameters.com_bit_rates = paras[0];
			para_w_flash();
		}
		else
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		break;

	/*---------------ID:14 power_on_target     ----------------*/
	case power_on_target:
		if (para_length >= 1) {
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		}
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		printf("Execute Type 8 Sybtype 14 ,power_on_target [%d]\r\n", paras[0]);
		power_control(paras[0], ON);

		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);

		break;

	/*---------------ID:15 power_off_target     ----------------*/
	case power_off_target:
		if (para_length >= 1) {
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		}
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		printf("Execute Type 8 Sybtype 15 ,power_off_target [%d]\r\n", paras[0]);

		power_control(paras[0], OFF);

		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		break;
	/*---------------  ID:16 enter specific mode      ----------------*/
	case enter_specific_mode:
		if (para_length == 1)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		printf("Execute Type 8 Sybtype 16, enter specific mode  \r\n");
		if (paras[0] == 1) {
			if (HK_frame.mode_status_flag == 5) {
				HK_frame.mode_status_flag = paras[0];
			}
			else {
				printf("Wrong\n");
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
				break;
			}
		}
		else if (paras[0] == 2 || paras[0] == 4) {
			if (HK_frame.mode_status_flag == 1) {
				HK_frame.mode_status_flag = paras[0];
			}
			else {
				printf("Wrong\n");
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
				break;
			}
		}
		else if (paras[0] == 3) {
			if (HK_frame.mode_status_flag == 2 || HK_frame.mode_status_flag == 4) {
				HK_frame.mode_status_flag = paras[0];
			}
			else {
				printf("Wrong\n");
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
				break;
			}
		}
		else if (paras[0] == 5) {
			if (HK_frame.mode_status_flag == 2 || HK_frame.mode_status_flag == 3 || HK_frame.mode_status_flag == 4) {
				HK_frame.mode_status_flag = paras[0];
			}
			else {
				printf("Wrong\n");
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
				break;
			}
		}
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			break;
		}

		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		break;
	/*---------------  ID:17 Enable / Disable INMS script handler  ----------------*/
	case INMS_Script_State:
		if (para_length == 1)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		printf("Execute Type 8 Sybtype 17\r\n");

		if (paras[0] == 1) {
			parameters.inms_status = 1;
			printf("enable inms script handler\n");
		}
		else if (paras[0] == 0) {
			parameters.inms_status = 0;
			printf("disable inms script handler\n");
		}
		para_w_flash();
		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		break;
	/*---------------  ID:18 Set SD partition label number  ----------------*/
	case SD_partition:
		if (para_length == 1)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		printf("Execute Type 8 Sybtype 18\r\n");

		if (paras[0] == 0) {
			parameters.SD_partition_flag = 0;
			printf("Set SD Read to partition [0]\n");
		}
		else if (paras[0] == 1) {
			parameters.SD_partition_flag = 1;
			printf("Set SD Read to partition [1]\n");
		}
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			break;
		}

		para_w_flash();
		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		break;
	/*---------------  ID:19 Set INMS timeout value  ----------------*/
	case INMS_timeout_change:
		if (para_length == 2)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		printf("Execute Type 8 Sybtype 19, change INMS timeout \r\n");

		uint16_t timeout;
		memcpy(&timeout, &paras, 2);
		timeout = csp_ntoh16(timeout);

		memcpy(&parameters.INMS_timeout, &timeout, 2);
		para_w_flash();
		printf("change INMS timeout to %d s\n", parameters.INMS_timeout);
		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		break;
	/*---------------  ID:20 Restart INMS handler ----------------*/
	case INMS_restart:
		if (para_length == 0)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		printf("Execute Type 8 Sybtype 20, Restart INMS task \r\n");
		/* Payload mode */
		if (HK_frame.mode_status_flag == 3) {
			if (inms_task != NULL) {
				power_control(4, OFF);
				vTaskDelete(inms_task);
				printf("Restart INMS task\n");
				vTaskDelay(2 * delay_time_based);
				xTaskCreate(vTaskinms, (const signed char*) "INMS", 1024 * 4, NULL, 2, &inms_task);
			}
		}
		else {
			printf("Not in nominal mode\n");
		}
		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		break;
	/*---------------  ID:21 use GPS header ----------------*/
	case Enable_GPS_header:
		if (para_length == 1)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		printf("Execute Type 8 Sybtype 21\r\n");

		if (paras[0] == 1) {
			use_GPS_header = 1;
			printf("enable GPS header\n");
		}
		else if (paras[0] == 0) {
			use_GPS_header = 0;
			printf("disable GPS header\n");
		}
		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		break;
	/*---------------  ID:22 Reset Error Report ----------------*/
	case Reset_Error_Report:
		if (para_length == 0)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		printf("Execute Type 8 Sybtype 22, reset Error Report\r\n");
		if (errPacket_reset() == No_Error) {
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		}
		break;
	/*---------------  ID:23 Manual Control Heater's switch ----------------*/
	case Manual_Heater_Switch:
		if (para_length == 1)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		printf("Execute Type 8 Sybtype 23, manual control heater's switch\r\n");
		txBuffer[0] = 13;
		txBuffer[1] = 0;
		txBuffer[2] = 1;
		if (paras[0] == 1) {
			printf("Switch ON Heater\n");
			txBuffer[3] = 1;
			if (i2c_master_transaction_2(0, eps_node, &txBuffer, 4, 0, 0, eps_delay) == E_NO_ERR) {
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			}
		}
		else if (paras[0] == 0) {
			printf("Switch OFF Heater\n");
			txBuffer[3] = 0;
			if (i2c_master_transaction_2(0, eps_node, &txBuffer, 4, 0, 0, eps_delay) == E_NO_ERR) {
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			}
		}
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
		break;
	/*---------------  ID:24 swicth the use of interface board ----------------*/
	case switchInterfaceBoard:
		if (para_length == 1)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		printf("Execute Type 8 Sybtype 24, switch the use of interface board\r\n");

		if (paras[0] == 1) {
			parameters.use_IFB = 1;
			printf("enable IFB\n");
		}
		else if (paras[0] == 0) {
			parameters.use_IFB = 0;
			if (inms_temp_moniter != NULL) {
				vTaskDelete(inms_temp_moniter);
				inms_temp_moniter = NULL;
			}
			if (inms_current_moniter != NULL) {
				vTaskDelete(inms_current_moniter);
				inms_current_moniter = NULL;
			}
			printf("disable IFB\n");
		}
		para_w_flash();
		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		break;
	/*---------------  ID:25 Set GS anomaly's threshole value ----------------*/
	case GS_timeout_change:
		if (para_length == 4)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		printf("Execute Type 8 Sybtype 25, Set GS anomaly's threshold value\r\n");
		little2big_32(&paras[0]);

		memcpy(&parameters.GS_threshold, &paras[0], 4);
		if (parameters.GS_threshold > 0) {
			printf("%" PRIu32 "\n", parameters.GS_threshold);
			para_w_flash();
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		}
		else {
			completionError = FS_IO_ERR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}

		break;
	/*---------------  ID:26 Activate GPS process, and record the information ----------------*/
	case Activate_GPS_process:
		if (para_length == 0)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		printf("Execute Type 8 Sybtype 26, Activate GPS process\r\n");
		if (gps_task == NULL) {
			xTaskCreate(GPS_task, (const signed char*) "GPS", 1024 * 4, NULL, 3, &gps_task);
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		}
		break;
	/*---------------  ID:30 Format SD and Initialize ----------------*/
	case SD_card_format:
		if (para_length == 1) {
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		}
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		printf("Execute Type 8 Sybtype 30\r\n");
		label = paras[0];
		if (HK_frame.mode_status_flag == 5) {
			if (label == 0 || label == 1) {
				xTaskCreate(task_format, (const signed char*) "FORMAT", 1024 * 4, (void *)label, 2, &format_task);
				while (1) {
					if (format_task == NULL) {
						sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
						break;
					}
					vTaskDelay(1 * delay_time_based);
				}
			}
			else
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
		else
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);

		break;
	/*---------------  ID:32 Enter Crippled Mode ----------------*/
	case enter_crippled_mode:
		if (para_length == 1)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		printf("Execute Type 8 Sybtype 32 Enter Crippled Mode \r\n");
		if (paras[0] == 1) {
			parameters.crippled_Mode = 1;
			printf("Set storage place to flash\n");
		}
		else if (paras[0] == 0) {
			parameters.crippled_Mode = 0;
			printf("Set storage place to SD card\n");
		}
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			break;
		}
		para_w_flash();
		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		break;
	/*---------------  ID:33 Enter Crippled Mode ----------------*/
	case test_task_hang:
		if (para_length == 2)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		if (paras[0] == 0) {
			if (paras[1] == 0) {
				if (bat_check_task != NULL) {
					vTaskDelete(bat_check_task);
					bat_check_task = NULL;
				}
			}
			else {
				if (bat_check_task == NULL)
					xTaskCreate(BatteryCheck_Task, (const signed char *) "BatCk", 1024 * 4, NULL, 2, &bat_check_task);
			}
		}
		else if (paras[0] == 1) {
			if (paras[1] == 0) {
				if (beacon_task != NULL) {
					vTaskDelete(beacon_task);
					beacon_task = NULL;
				}
			}
			else {
				if (beacon_task == NULL)
					xTaskCreate(beacon_Task, (const signed char *) "beacon", 1024 * 4, NULL, 2, &beacon_task);
			}
		}
		else if (paras[0] == 2) {
			if (paras[1] == 0) {
				if (adcs_task != NULL) {
					vTaskDelete(adcs_task);
					adcs_task = NULL;
				}
			}
			else {
				if (adcs_task == NULL)
					xTaskCreate(ADCS_Task, (const signed char * ) "ADCS", 1024 * 4, NULL, 2, &adcs_task);
			}

		}
		else if (paras[0] == 3) {
			if (paras[1] == 0) {
				if (seuv_task != NULL) {
					vTaskDelete(seuv_task);
					seuv_task = NULL;
				}
			}
			else {
				if (seuv_task == NULL)
					xTaskCreate(SolarEUV_Task, (const signed char * ) "SEUV", 1024 * 4, NULL, 2, &seuv_task);
			}
		}
		else if (paras[0] == 4) {
			if (paras[1] == 0) {
				if (Anom_mon_task != NULL) {
					vTaskDelete(Anom_mon_task);
					Anom_mon_task = NULL;
				}
			}
			else {
				if (Anom_mon_task == NULL)
					xTaskCreate(Anomaly_Monitor_Task, (const signed char *) "Anom", 1024 * 4, NULL, 2, &Anom_mon_task);
			}
		}
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			break;
		}
		para_w_flash();
		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		break;

	/*---------------- Otherwise ----------------*/
	default:
		sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);

		break;
	}
}
