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
#define Enable_Subsystem			1				/* (?)This function is not used */
#define Disable_Subsystem			2				/* (?)This function is not used */
#define reboot						3				/* Reboot the whole system */
#define Sync_Time					4				/* Sync the OBC time*/
#define DTTest						5				/* Test Downlink 200 Bytes*/
#define ShutdownTransmitter			6				/* Shut Down Trasmitter */
#define ResumeTransmitter			7				/* Resume Transmitter */
#define Enter_Safe_Threshold		8				/* Set Safe mode enter threshold (default = 7000) */
#define Leave_Safe_Threshold		9				/* Set Safe mode leave threshold (default = 7500) */
#define I2C_COMMAND					10				/* (!)very useful, but use it carefully */
#define para_to_default 			11				/* Set parameters to default */
#define set_tx_rates 				12				/* Set tx rate */
#define set_call_sign 				13				/* Set Call sign "To” : NCKUGS & “From” : TW01TN*/
#define power_on_target 			14				/* Power ON specific system */
#define power_off_target 			15				/* Power OFF specific system */
#define enter_nominal_mode			16				/* Enter Nominal mode to start science related task */
#define INMS_Script_State			17				/* Set INMS handler to enable/ disable */
#define SD_partition 				18				/* Set which partition would like to read from */
#define INMS_timeout_change			19				/* Set INMS timeout value */
#define INMS_restart				20				/* Restart INMS script handler, do this after upload new script*/
#define SD_card_format				30				/* Format SD card, and create default folders */
#define SD_unlock					31				/* Unlock SD card */

extern void vTaskinms(void * pvParameters);

void decodeService8(uint8_t subType, uint8_t*telecommand) {
	uint8_t txBuffer[200];
	timestamp_t t;
	time_t tt;
	uint8_t completionError = I2C_SEND_ERROR;

	uint16_t para_length = (telecommand[4] << 8) + telecommand[5] - 4;
	uint8_t type = 8;
	uint8_t paras[180];
	uint16_t threshold;
	FATFS fs;
	FIL file;
	BYTE label;

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
		printf("OBC time Sync by telecommand to : %s\r\n", ctime(&tt));
		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); 
		break;


	/*---------------ID:5 DT 0~200----------------*/
	case DTTest:
		if (para_length == 0)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS); 
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE); 
			break;
		}
		printf("Execute Type 8 Sybtype 5 , Downlink 1~200!! \r\n");
		for (int a = 1; a < 201; a++)
			txBuffer[a - 1] = a;

		SendPacketWithCCSDS_AX25(&txBuffer, 200, obc_apid, type, subType);
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
		if (tx_mode(3) != 0) {  
			printf("tx_mode set fail \r\n");
		}

		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		break;

	/*---------------ID:7 ResumeTransmitter----------------*/
	case  ResumeTransmitter:
		printf("Execute Type 8 Sybtype 7 , Resume Transmitter \r\n");
		parameters.shutdown_flag = 0;
		para_w_flash();
		printf("Shutdown Resume Command Detected!! \r\n");
		if (tx_mode(1) != 0)   //set transceiver into standby mode
			printf("tx_mode set fail \r\n");
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

		memcpy(&parameters.vbat_safe_threshold, &threshold, 2);
		para_w_flash();
		printf("Enter_Safe_Threshold = %d mV\n", parameters.vbat_safe_threshold);
		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); 
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

		memcpy(&parameters.vbat_recover_threshold, &threshold, 2);
		para_w_flash();
		printf("Leave_Safe_Threshold = %d mV\n", parameters.vbat_recover_threshold);
		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); 
		break;
	/*---------------ID:10 I2C_COMMAND----------------*/
	case  I2C_COMMAND:
		if (para_length >= 2)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE); 
			break;
		}
		printf("Execute Type 8 Sybtype 10 , I2C_COMMAND node[%d] rx[%d] parameter \r\n", paras[0], para_length - 2);
		if (i2c_master_transaction(0, paras[0], &paras[2], para_length - 2, &txBuffer, paras[1], com_delay) == E_NO_ERR) {

			if (paras[1] != 0)
				i2c_master_transaction(0, com_tx_node, &txBuffer, paras[1], 0, 0, com_delay);
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); 
		} else
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError); 

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

		printf("First Flight %d\n", (int) parameters.first_flight);
		printf("shutdown_flag %d\n", (int) parameters.shutdown_flag);
		printf("wod_store_count %d\n", (int) parameters.wod_store_count);
		printf("inms_store_count %d\n", (int) parameters.inms_store_count);
		printf("seuv_store_count %d\n", (int) parameters.seuv_store_count);
		printf("hk_store_count %d\n", (int) parameters.hk_store_count);
		printf("obc_packet_sequence_count %d\n", (int) parameters.obc_packet_sequence_count);
		printf("vbat_recover_threshold %d\n", (int) parameters.vbat_recover_threshold);
		printf("vbat_safe_threshold %d\n", (int) parameters.vbat_safe_threshold);

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

		if (set_tx_rate(paras[0]) != Error){
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); 
			parameters.com_bit_rates = paras[0];
			para_w_flash();
		}
		else
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError); 
		break;

	/*---------------ID:13 set_call_sign      ----------------*/
	case set_call_sign:
		if (para_length == 0)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE); 
			break;
		}
		printf("Execute Type 8 Sybtype 13 ,set_call_sign \r\n");
		set_Call_Sign(0);

		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); 

		break;

	/*---------------ID:14 power_on_target     ----------------*/
	case power_on_target:
		printf("para_length = %d\n", para_length);
		printf("target = %d\n", paras[0]);
		if (para_length >= 1) {
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  
		}
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE); 
			printf("Wrong!!\n");
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
	/*---------------  ID:16 enter nominal mode      ----------------*/
	case enter_nominal_mode:
		if (para_length == 0)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE); 
			break;
		}
		printf("Execute Type 8 Sybtype 16 ,enter nominal mode  \r\n");

		HK_frame.mode_status_flag = 3;
		parameters.com_bit_rates = 8;
		para_w_flash();
		set_tx_rate(parameters.com_bit_rates);
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
		else
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);

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
	/*---------------  ID:30 Format SD and Initialize ----------------*/
	case SD_card_format:
		if (para_length == 1)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE); 
			break;
		}
		printf("Execute Type 8 Sybtype 30\r\n");

		label = paras[0];
		if (f_mount(label, NULL) == FR_OK)
			printf("unmount %d\n", label);

		if (f_mount(label, &fs) == FR_OK)
			printf("mount %d\n", label);

		if (paras[0] == 0) {
			f_mkfs(0, 0, 0);
			f_mkdir("0:/HK_DATA");
			f_mkdir("0:/INM_DATA");
			f_mkdir("0:/SEU_DATA");
			f_mkdir("0:/EOP_DATA");
			f_mkdir("0:/WOD_DATA");			
			f_open(&file, "0:/part0", FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
		}
		else if (paras[0] == 1) {
			f_mkfs(1, 0, 0);
			f_mkdir("1:/HK_DATA");
			f_mkdir("1:/INM_DATA");
			f_mkdir("1:/SEU_DATA");
			f_mkdir("1:/EOP_DATA");
			f_mkdir("1:/WOD_DATA");			
			f_open(&file, "1:/part1", FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
		}		
		else
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);

		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		break;
	/*---------------  ID:31 unlock SD (Test Stage) ----------------*/	
	case SD_unlock:
		if (para_length == 1)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE); 
			break;
		}
		printf("Execute Type 8 Sybtype 31\r\n");
		label = paras[0];
		if (f_mount(label, NULL) == FR_OK)
			printf("unmount %d\n", label);
		vTaskDelay(3 * delay_time_based);
		if (f_mount(label, &fs) == FR_OK)
			printf("mount %d\n", label);
		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		break;
	/*----------------------------Otherwise----------------*/
	default:
		sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);

		break;
	}
}
