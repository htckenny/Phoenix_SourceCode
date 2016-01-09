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

/* Definition of the service sub type*/
#define Report_HK_State		1		/* Report House Keeping Data */
#define Report_COM_HK_1		2		/* Report Communication Board House Keeping Data */
#define Report_EPS_HK		3		/* Report EPS House Keeping Data */
#define Report_Parameter	4		/* Report All System Parameters */
#define Report_COM_HK_2		5		/* Report COM Board Baud Rate */
#define Report_Task_Status	6		/* Report all the task status */
#define Report_ADCS_HK		7		/* Report ADCS House Keeping Data */
#define Report_Script_Stat	8		/* Report INMS script's status */

#define scriptNum 7

extern uint16_t fletcher(uint8_t *script, size_t length);

void perform_fletcher(uint8_t * check_sum_final) {
	int maxlength = 0;
	int script_length[scriptNum];
	uint16_t xsum[scriptNum];
	int results;

	printf("length read\n");
	for (int i = 0; i < scriptNum; i++) {
		script_length[i] = inms_script_length_flash(i);
		if (script_length[i] >= maxlength) {
			maxlength = script_length[i];
		}
	}
	uint8_t script[scriptNum][maxlength];

	printf("data read\n");
	for (int i = 0 ; i < scriptNum ; i++){
		results = inms_script_read_flash(i, script_length[i], &script[i]);
		xsum[i] = fletcher(script[i], script_length[i]);

		// printf("%x\n", script_length[i]);
		// printf("%x\n", script[i][0] + (script[i][1] << 8));

		/* No error */
		if (xsum[i] == 0 && results == No_Error ) {
			check_sum_final[i] = 0;		
		}
		/* Error with no script */
		else if (results == Error){
			check_sum_final[i] = 1;
		}
		/* Error with checksum */
		else if (xsum[i] != 0){
			check_sum_final[i] = 2;
		}
		
	}
}

/* telecommand Service 3  */
void decodeService3(uint8_t subType, uint8_t*telecommand) {
	uint8_t txBuffer[254];
	uint8_t txlen;
	uint16_t buffs;
	uint8_t i2c_tx[10];
	uint8_t type = 3;
	uint8_t completionError;
	// note: parameter start from telecommand[10]
	/*------------------------------------------Telecommand-----------------------------------*/


	switch (subType) {

	/*---------------ID:1 Report_HK_State----------------*/
	case Report_HK_State:

		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report

		/* EOP mode */
		if (HK_frame.mode_status_flag == 2 && parameters.first_flight == 1)
			HK_frame.mode_status_flag = 5;
		
		memcpy(&txBuffer[0], &HK_frame.mode_status_flag, 1);
		memcpy(&txBuffer[1], &parameters.inms_store_count, 4);
		memcpy(&txBuffer[5], &parameters.seuv_store_count, 4);
		memcpy(&txBuffer[9], &parameters.hk_store_count, 4);
		memcpy(&txBuffer[13], &parameters.wod_store_count, 4);

		buffs = Interface_3V3_current_get();
		memcpy(&txBuffer[17], &buffs, 2);

		buffs = Interface_5V_current_get();
		memcpy(&txBuffer[19], &buffs, 2);

		buffs = Interface_tmp_get();
		memcpy(&txBuffer[21], &buffs, 2);

		buffs = Interface_inms_thermistor_get();
		memcpy(&txBuffer[23], &buffs, 2);

		txlen = 25;
		printf("[TM 3-1]current mode = %d\n", txBuffer[0]);
		SendPacketWithCCSDS_AX25(&txBuffer, txlen, obc_apid, type, subType);
		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); //send COMPLETE_success report

		break;

	/*---------------ID:2 Report_COM_HK_1    ----------------*/
	case Report_COM_HK_1:
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report

		i2c_tx[0] = com_rx_hk;
		if (i2c_master_transaction(0, com_rx_node, &i2c_tx, 1, &txBuffer, com_rx_hk_len, com_delay) == E_NO_ERR) {
			txlen = com_rx_hk_len;
			SendPacketWithCCSDS_AX25(&txBuffer, txlen, obc_apid, type, subType);
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		}
		else {
			completionError = I2C_READ_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError); //send complete fail
		}
		break;

	/*---------------ID:3 Report_EPS_HK   ----------------*/
	case Report_EPS_HK :
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report

		i2c_tx[0] = eps_hk;
		if (i2c_master_transaction(0, eps_node, &i2c_tx, 1, &txBuffer, eps_hk_len, eps_delay) == E_NO_ERR) {
			txlen = eps_hk_len - 2;
			SendPacketWithCCSDS_AX25(&txBuffer[2], txlen, obc_apid, type, subType);
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		}
		else {

			completionError = I2C_READ_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}


		break;
	/*---------------ID:4 Report_Parameter    ----------------*/
	case Report_Parameter:
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report

		memcpy(txBuffer, &parameters.first_flight, sizeof(parameter_t));
		txlen = sizeof(parameter_t);
		SendPacketWithCCSDS_AX25(&txBuffer, txlen, obc_apid, type, subType);

		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		break;
	/*---------------ID:5 Report_COM_HK_2    ----------------*/
	case Report_COM_HK_2:
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report

		i2c_tx[0] = com_tx_hk;
		if (i2c_master_transaction(0, com_tx_node, &i2c_tx, 1, &txBuffer, 1, com_delay) == E_NO_ERR) {
			txlen = 1;
			SendPacketWithCCSDS_AX25(&txBuffer, txlen, obc_apid, type, subType);
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		}
		else {
			completionError = I2C_READ_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}

		break;
	/*---------------ID:6 Report_Task_Status    ----------------*/
	case Report_Task_Status:
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);

		if (status_update() == E_NO_ERR) {
			printf("[TM 3-6]\n");
			printf("Task Name\t\tStatus\n");
			printf("---------------------\n");
			printf("Mode_task\t\t%d\n", status_frame.mode_task);
			printf("BatC_task\t\t%d\n", status_frame.bat_check_task);
			printf("COM_task\t\t%d\n", status_frame.com_task);
			printf("WOD_task\t\t%d\n", status_frame.wod_task);
			printf("WOD_task\t\t%d\n", status_frame.beacon_task);

			printf("init_task\t\t%d\n", status_frame.init_task);
			printf("adcs_task\t\t%d\n", status_frame.adcs_task);
			printf("seuv_task\t\t%d\n", status_frame.seuv_task);
			printf("EOP_task\t\t%d\n", status_frame.eop_task);
			printf("HK_task\t\t\t%d\n", status_frame.hk_task);

			printf("I_EH_task\t\t%d\n", status_frame.inms_error_handle);
			printf("I_CM_task\t\t%d\n", status_frame.inms_current_moniter);
			printf("I_SH_task\t\t%d\n", status_frame.inms_task);
			printf("I_RH_task\t\t%d\n", status_frame.inms_task_receive);	

			printf("Sche_task\t\t%d\n", status_frame.schedule_task);	

			memcpy(&txBuffer[0], &status_frame.mode_task, 15);

			txlen = 15;
			SendPacketWithCCSDS_AX25(&txBuffer, txlen, obc_apid, type, subType);
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		}
		else {
			completionError = FS_IO_ERR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
		break;
	/*--------------- ID:8 Report INMS script's status ----------------*/	
	case Report_Script_Stat:
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);

		perform_fletcher(txBuffer);
		hex_dump(&txBuffer, scriptNum);

		txlen = 7;
		SendPacketWithCCSDS_AX25(&txBuffer, txlen, obc_apid, type, subType);
		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		break;

	default:

		sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
		break;
	}
}



