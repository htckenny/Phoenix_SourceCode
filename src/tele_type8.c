#include <util/timestamp.h>
#include "subsystem.h"
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
#include "inttypes.h"

void decodeService8(uint8_t subType, uint8_t*telecommand) {
	uint8_t txBuffer[200];
	timestamp_t t;
	time_t tt;

	uint8_t completionError = I2C_SEND_ERROR;
// note: parameter start from telecommand[10]
	/*------------------------------------------Telecommand-----------------------------------*/
	// printf("Enter type 8\n");
	// printf("enter type 8\n");

#define reboot 						3
#define Sync_Time 					4
#define DTTest 						5
#define ShutdownTransmitter 		6
#define ResumeTransmitter			7
#define Enter_Safe_Threshold 		8
#define Leave_Safe_Threshold		9
#define I2C_COMMAND 				10 		//<---- very useful, but use it carefully
#define para_to_default 			11
#define set_tx_rates 				12
#define set_call_sign 				13
#define power_on_target 			14
#define power_off_target 			15
#define enter_nominal_mode			16
#define	INMS_Script_State			17

	printf("telecommand[4] = %u\n", telecommand[4]);
	printf("telecommand[5] = %u\n", telecommand[5]);
	uint16_t para_length = (telecommand[4] << 8) + telecommand[5] - 4;
	printf("para_len = %u\n", para_length);
	uint8_t type = 8;
	uint8_t paras[180];
	if (para_length > 0)
		memcpy(&paras, telecommand + 9, para_length);
	switch (subType) {
	/*---------------ID:3 reboot      ----------------*/
	case reboot:
		if (para_length == 0)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance success
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE); // send acceptance fail
			break;
		}
		printf("Execute Type 8 Sybtype 3 , Rebooting \r\n");

		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); //send COMPLETE_success report
		txBuffer[0] = 20;
		i2c_master_transaction(0, eps_node, &txBuffer, 1, 0, 0, eps_delay);

		break;
	/*---------------ID:4 Sync_Time----------------*/

	case Sync_Time:
		if (para_length == 5)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance success
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE); // send acceptance fail
			break;
		}
		printf("Execute Type 8 Sybtype 4 , Sync_Time \r\n");
		t.tv_sec = 0;
		t.tv_nsec = 0;
		// printf("%d\n", paras[0]);
		memcpy(&t.tv_sec, &paras[0], 4);
		t.tv_sec = csp_ntoh32(t.tv_sec);
		// printf("sec = %" PRIu32 "\n", t.tv_sec);

		t.tv_sec += 946684800 ;
		
		obc_timesync(&t, 1000);
		tt = t.tv_sec ;
		printf("OBC time Sync by telecommand to : %s\r\n", ctime(&tt));
		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); //send COMPLETE_success report
		break;


	/*---------------ID:5 DT 0~200----------------*/
	case DTTest:
		if (para_length == 0)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance success
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE); // send acceptance fail
			break;
		}
		printf("Execute Type 8 Sybtype 5 , Downlink 1~200!! \r\n");
		for (int a = 1; a < 201; a++)
			txBuffer[a - 1] = a;

		SendPacketWithCCSDS_AX25(&txBuffer, 200, obc_apid, type, subType);
		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); //send COMPLETE_success report
		break;
	/*---------------ID:6 ShutdownTransmitter----------------*/

	case  ShutdownTransmitter:
		if (para_length == 0)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance success
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE); // send acceptance fail
			break;
		}
		printf("Execute Type 8 Sybtype 6 , ShutdownTransmitter \r\n");
		parameters.shutdown_flag = 1;
		para_w();
		printf("Shutdown Command Detected!! \r\n");
		if (tx_mode(3) != 0) {  //set transceiver into standby mode
			printf("tx_mode set fail \r\n");
		}

		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); //send COMPLETE_success report
		break;

	/*---------------ID:7 ResumeTransmitter----------------*/
	case  ResumeTransmitter:
//	if(para_length==0)
//	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance success
//		else{
//			sendTelecommandReport_Failure(telecommand,CCSDS_T1_ACCEPTANCE_FAIL,CCSDS_ERR_ILLEGAL_TYPE); // send acceptance fail
//			break;
//		}
		printf("Execute Type 8 Sybtype 7 , Resume Transmitter \r\n");
		parameters.shutdown_flag = 0;
		para_w();
		printf("Shutdown Resume Command Detected!! \r\n");
		if (tx_mode(1) != 0)   //set transceiver into standby mode
			printf("tx_mode set fail \r\n");
		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); //send COMPLETE_success report
		break;

	/*---------------ID:8 Enter_Safe_Threshold----------------*/
	case  Enter_Safe_Threshold:
		if (para_length == 2)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance success
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE); // send acceptance fail
			break;
		}
		printf("Execute Type 8 Sybtype 8 , Enter_Safe_Threshold \r\n");
		memcpy(&parameters.vbat_safe_threshold, &paras[0], 2);
		para_w();
		printf("Enter_Safe_Threshold = %d\n", parameters.vbat_safe_threshold);
		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); //send COMPLETE_success report
		break;
	/*---------------ID:9 Leave_Safe_Threshold----------------*/
	case  Leave_Safe_Threshold:
		if (para_length == 2)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance success
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE); // send acceptance fail
			break;
		}
		printf("Execute Type 8 Sybtype 9 , Leave_Safe_Threshold \r\n");
		memcpy(&parameters.vbat_recover_threshold, &paras[0], 2);
		para_w();
		printf("Enter_Safe_Threshold = %d\n", parameters.vbat_recover_threshold);
		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); //send COMPLETE_success report
		break;
	/*---------------ID:10 I2C_COMMAND----------------*/
	case  I2C_COMMAND:
		if (para_length >= 2)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance success
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE); // send acceptance fail
			break;
		}
		printf("Execute Type 8 Sybtype 10 , I2C_COMMAND \r\n");
		if (i2c_master_transaction(0, paras[0], &paras[2], para_length - 2, &txBuffer, paras[1], eps_delay) == E_NO_ERR) {

			if (paras[1] != 0)
				i2c_master_transaction(0, com_tx_node, &txBuffer, paras[1], 0, 0, com_delay);
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); //send COMPLETE_success report
		} else
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError); //send complete fail

		break;


	/*---------------ID:11 para_to_default----------------*/
	case  para_to_default:
		if (para_length == 0)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance success
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE); // send acceptance fail
			break;
		}
		printf("Execute Type 8 Sybtype 11 , para_to_default \r\n");
		parameter_init();
		para_w();

		printf("First Flight %d\n", (int) parameters.first_flight);
		printf("shutdown_flag %d\n", (int) parameters.shutdown_flag);
		printf("wod_store_count %d\n", (int) parameters.wod_store_count);
		printf("inms_store_count %d\n", (int) parameters.inms_store_count);
		printf("seuv_store_count %d\n", (int) parameters.seuv_store_count);
		printf("hk_store_count %d\n", (int) parameters.hk_store_count);
		printf("obc_packet_sequence_count %d\n", (int) parameters.obc_packet_sequence_count);
		printf("vbat_recover_threshold %d\n", (int) parameters.vbat_recover_threshold);
		printf("vbat_safe_threshold %d\n", (int) parameters.vbat_safe_threshold);

		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); //send COMPLETE_success report
		break;
	/*---------------ID:12 set_tx_rates      ----------------*/
	case set_tx_rates:
		if (para_length == 1)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance success
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE); // send acceptance fail
			break;
		}
		printf("Execute Type 8 Sybtype 12 ,set_tx_rate \r\n");

		if (set_tx_rate(paras[0]) != Error)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); //send COMPLETE_success report
		else
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError); //send complete fail
		break;

	/*---------------ID:13 set_call_sign      ----------------*/
	case set_call_sign:
		if (para_length == 0)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance success
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE); // send acceptance fail
			break;
		}
		printf("Execute Type 8 Sybtype 13 ,set_call_sign \r\n");
		set_Call_Sign(0);

		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); //send COMPLETE_success report

		break;

	/*---------------ID:14 power_on_target     ----------------*/
	case power_on_target:
		printf("para_length = %d\n", para_length);
		printf("target = %d\n", paras[0]);
		if (para_length >= 1) {
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance success
		}
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE); // send acceptance fail
			printf("Wrong!!\n");
			break;
		}
		printf("Execute Type 8 Sybtype 14 ,power_on_target [%d]\r\n", paras[0]);
		power_control(paras[0], ON);

		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); //send COMPLETE_success report

		break;

	/*---------------ID:15 power_off_target     ----------------*/
	case power_off_target:
		if (para_length >= 1) {
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance success
		}
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE); // send acceptance fail
			break;
		}
		printf("Execute Type 8 Sybtype 15 ,power_off_target [%d]\r\n", paras[0]);

		power_control(paras[0], OFF);

		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); //send COMPLETE_success report
		break;
	/*---------------  ID:16 enter nominal mode      ----------------*/
	case enter_nominal_mode:
		if (para_length == 0)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance success
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE); // send acceptance fail
			break;
		}
		printf("Execute Type 8 Sybtype 16 ,enter nominal mode  \r\n");
		
		// parameters.first_flight = 0;
		HK_frame.mode_status_flag = 3;
		para_w();

		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); //send COMPLETE_success report


		break;
	case INMS_Script_State:
		if (para_length == 1)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance success
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE); // send acceptance fail
			break;
		}
		printf("Execute Type 8 Sybtype 17\r\n");
		
		// parameters.first_flight = 0;
		if (paras[0] == 1) {
			inms_status = 1;
			printf("enable inms script handler\n");			
		}	//enable
		else if (paras[0] == 0){
			inms_status = 0;
			printf("disable inms script handler\n");		
		}

		para_w();

		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); //send COMPLETE_success report


		break;

	/*----------------------------Otherwise----------------*/
	default:
		sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);

		break;
	}
}
