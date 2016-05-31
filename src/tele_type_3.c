#include <string.h>
#include <nanomind.h>
#include <time.h>

#include <dev/i2c.h>
#include <util/timestamp.h>
#include <util/hexdump.h>
#include <freertos/FreeRTOS.h>
#include <csp/csp_endian.h>
#include <io/nanopower2.h>
#include <csp/csp.h>
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
#define Report_Task_Status	6		/* Report All the task status */
#define Report_ADCS_HK		7		/* Report ADCS House Keeping Data */
#define Report_Script_Stat	8		/* Report INMS script's status */
#define Report_WOD_Test		9
#define Report_GPS_Status	10		/* Report GPS's status */
#define Report_Error_Record	11		/* Report Error Record */
#define Report_Data_Number	12		/* Report Collected Data Number */
#define Report_GPS_Record	13		/* Report GPS's Record */

extern uint16_t fletcher(uint8_t *script, size_t length);
extern void little2big_32(uint8_t * input_data);
extern void little2big_16(uint8_t * input_data);

void perform_fletcher(uint8_t * check_sum_final) {
	int script_length[scriptNum];
	uint16_t xsum[scriptNum];
	int results;
	uint8_t *script = NULL;

	for (int i = 0; i < scriptNum; i++) {
		script_length[i] = inms_script_length_flash(i);
		if (!script)
			script = malloc(script_length[i]);
		else {
			free(script);
			script = malloc(script_length[i]);
		}

		results = inms_script_read_flash(i, script_length[i], script);
		xsum[i] = fletcher(script, script_length[i]);

		/* No error */
		if (xsum[i] == 0 && results == No_Error ) {
			check_sum_final[i] = 0;
		}
		/* Error with no script */
		else if (results == Error) {
			check_sum_final[i] = 1;
		}
		/* Error with checksum */
		else if (xsum[i] != 0) {
			check_sum_final[i] = 2;
		}
	}
	free(script);
}

/* telecommand Service 3  */
void decodeService3(uint8_t subType, uint8_t* telecommand) {
	uint8_t txBufferWithSID[254] = {0};
	uint8_t txBuffer[254];
	uint8_t txlen;
	uint16_t buffs;
	uint8_t i2c_tx[10];
	uint8_t type = 3;
	uint8_t completionError;

	uint16_t para_length = (telecommand[4] << 8) + telecommand[5] - 4;
	uint8_t paras[180];

	if (para_length > 0)
		memcpy(&paras, telecommand + 9, para_length);

	switch (subType) {

	/*---------------ID:1 Report_HK_State----------------*/
	case Report_HK_State:

		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);

		/* EOP mode */
		if (HK_frame.mode_status_flag == 2 && parameters.first_flight == 1)
			HK_frame.mode_status_flag = 4;

		memcpy(&txBuffer[0], &HK_frame.mode_status_flag, 1);
		memcpy(&txBuffer[1], &parameters.inms_store_count, 4);
		memcpy(&txBuffer[5], &parameters.seuv_store_count, 4);
		memcpy(&txBuffer[9], &parameters.hk_store_count, 4);
		memcpy(&txBuffer[13], &parameters.wod_store_count, 4);

		buffs = Interface_3V3_current_get();
		vTaskDelay(0.01 * delay_time_based);
		buffs = csp_ntoh16(buffs);
		memcpy(&txBuffer[17], &buffs, 2);

		buffs = Interface_5V_current_get();
		vTaskDelay(0.01 * delay_time_based);
		buffs = csp_ntoh16(buffs);
		memcpy(&txBuffer[19], &buffs, 2);

		buffs = Interface_inms_thermistor_get();
		vTaskDelay(0.01 * delay_time_based);
		buffs = csp_ntoh16(buffs);
		memcpy(&txBuffer[21], &buffs, 2);

		buffs = Interface_tmp_get();
		vTaskDelay(0.01 * delay_time_based);
		buffs = csp_ntoh16(buffs);
		memcpy(&txBuffer[23], &buffs, 2);

		txlen = 25;
		printf("[TM 3-1]current mode = %d\n", txBuffer[0]);
		txBufferWithSID[0] = 31;
		memcpy(&txBufferWithSID[1], &txBuffer[0], txlen);
		SendPacketWithCCSDS_AX25(&txBufferWithSID[0], txlen + 1, obc_apid, type, 25);
		hex_dump(&txBuffer, txlen);
		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);

		break;

	/*---------------ID:2 Report_COM_HK_1    ----------------*/
	case Report_COM_HK_1:
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);

		i2c_tx[0] = com_rx_hk;
		if (i2c_master_transaction_2(0, com_rx_node, &i2c_tx, 1, &txBuffer, com_rx_hk_len, com_delay) == E_NO_ERR) {
			for (int i = 0; i < 7; i++) {
				little2big_16(&txBuffer[i * 2]);
			}
			txlen = com_rx_hk_len;
			txBufferWithSID[0] = 32;
			memcpy(&txBufferWithSID[1], &txBuffer[0], txlen);
			SendPacketWithCCSDS_AX25(&txBufferWithSID[0], txlen + 1, obc_apid, type, 25);
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		}
		else {
			completionError = I2C_READ_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
		break;

	/*---------------ID:3 Report_EPS_HK   ----------------*/
	case Report_EPS_HK :
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		i2c_tx[0] = eps_hk;
		i2c_tx[1] = 0;
		txlen = sizeof(eps_hk_t);
		if (i2c_master_transaction_2(0, eps_node, &i2c_tx, 2, &txBuffer, txlen + 2, eps_delay) == E_NO_ERR) {
			txBufferWithSID[0] = 33;
			memcpy(&txBufferWithSID[1], &txBuffer[2], txlen);
			SendPacketWithCCSDS_AX25(&txBufferWithSID[0], txlen + 1, obc_apid, type, 25);
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		}
		else {
			completionError = I2C_READ_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
		break;
	/*---------------ID:4 Report_Parameter    ----------------*/
	case Report_Parameter:
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);

		memcpy(txBuffer, &parameters.first_flight, sizeof(parameter_t));

		little2big_32(&txBuffer[6]);
		little2big_32(&txBuffer[21]);
		little2big_32(&txBuffer[25]);
		little2big_32(&txBuffer[29]);
		little2big_32(&txBuffer[33]);
		little2big_32(&txBuffer[37]);
		little2big_32(&txBuffer[58]);

		little2big_16(&txBuffer[41]);
		little2big_16(&txBuffer[45]);
		little2big_16(&txBuffer[47]);
		little2big_16(&txBuffer[50]);
		little2big_16(&txBuffer[52]);

		txlen = sizeof(parameter_t);
		txBufferWithSID[0] = 34;
		memcpy(&txBufferWithSID[1], &txBuffer[0], txlen);
		hex_dump(&txBufferWithSID[1], txlen);
		SendPacketWithCCSDS_AX25(&txBufferWithSID[0], txlen + 1, obc_apid, type, 25);

		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		break;
	/*---------------ID:5 Report_COM_HK_2    ----------------*/
	case Report_COM_HK_2:
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txlen = 5;
		i2c_tx[0] = com_tx_hk;
		if (i2c_master_transaction_2(0, com_tx_node, &i2c_tx, 1, &txBuffer, 1, com_delay) == E_NO_ERR) {
			txBufferWithSID[0] = 35;
			memcpy(&txBufferWithSID[1], &txBuffer[0], 1);
		}
		i2c_tx[0] = 0xC3;
		if (i2c_master_transaction_2(0, ant_node, &i2c_tx, 1, &txBuffer[0], 2, com_delay) == E_NO_ERR) {
			memcpy(&txBufferWithSID[2], &txBuffer[0], 2);
		}
		i2c_tx[0] = 0xC0;
		if (i2c_master_transaction_2(0, ant_node, &i2c_tx, 1, &txBuffer[0], 2, com_delay) == E_NO_ERR) {
			memcpy(&txBufferWithSID[4], &txBuffer[0], 2);
			little2big_16(&txBufferWithSID[4]);
			SendPacketWithCCSDS_AX25(&txBufferWithSID[0], txlen + 1, obc_apid, type, 25);
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
			printf("beacon_task\t\t%d\n", status_frame.beacon_task);

			printf("init_task\t\t%d\n", status_frame.init_task);
			printf("adcs_task\t\t%d\n", status_frame.adcs_task);
			printf("seuv_task\t\t%d\n", status_frame.seuv_task);
			printf("EOP_task\t\t%d\n", status_frame.eop_task);
			printf("HK_task\t\t\t%d\n", status_frame.hk_task);

			printf("I_EH_task\t\t%d\n", status_frame.inms_error_handle);
			printf("I_CM_task\t\t%d\n", status_frame.inms_current_moniter);
			printf("I_TM_task\t\t%d\n", status_frame.inms_temp_moniter);
			printf("I_SH_task\t\t%d\n", status_frame.inms_task);
			printf("I_RH_task\t\t%d\n", status_frame.inms_task_receive);

			printf("Sche_task\t\t%d\n", status_frame.schedule_task);
			printf("SEUVCM_task\t\t%d\n", status_frame.seuv_cm_task);
			printf("Anom_Mon task\t\t%d\n", status_frame.Anom_mon_task);
			printf("GPS task\t\t%d\n", status_frame.gps_task);

			memcpy(&txBuffer[0], &status_frame.mode_task, sizeof(status_frame_t));

			txlen = sizeof(status_frame_t);
			txBufferWithSID[0] = 36;
			memcpy(&txBufferWithSID[1], &txBuffer[0], txlen);
			SendPacketWithCCSDS_AX25(&txBufferWithSID[0], txlen + 1, obc_apid, type, 25);
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		}
		else {
			completionError = FS_IO_ERR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
		break;

	/*---------------ID:7 ADCS House Keeping Data ----------------*/
	case Report_ADCS_HK:
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txlen = 1;
		txBuffer[0] = adcs_para.mag_deploy_status_flag;
		txBufferWithSID[0] = 37;
		memcpy(&txBufferWithSID[1], &txBuffer[0], txlen);
		SendPacketWithCCSDS_AX25(&txBufferWithSID[0], txlen + 1, obc_apid, type, 25);
		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		break;
	/*--------------- ID:8 Report INMS script's status ----------------*/
	case Report_Script_Stat:
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);

		perform_fletcher(txBuffer);
		hex_dump(&txBuffer, scriptNum);

		txlen = scriptNum;
		txBufferWithSID[0] = 38;
		memcpy(&txBufferWithSID[1], &txBuffer[0], txlen);
		SendPacketWithCCSDS_AX25(&txBufferWithSID[0], txlen + 1, obc_apid, type, 25);
		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		break;
	/*--------------- ID:9 Test SCS WOD interface ----------------*/
	case Report_WOD_Test:
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);

		uint8_t test[9];
		test[0] = 30;
		memcpy(&test[1], &beacon_frame.mode, 8);

		SendPacketWithCCSDS_AX25(&test, 9, obc_apid, 3, 25);

		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		break;
	/*--------------- ID:10 Report GPS status ----------------*/
	case Report_GPS_Status:
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);

		i2c_tx[0] = 139; /* check 168 ~ 172 */
		if (i2c_master_transaction_2(0, adcs_node, &i2c_tx, 1, &txBuffer, 60, adcs_delay) == E_NO_ERR) {
			txlen = 30;
			txBufferWithSID[0] = 40;
			little2big_16(&txBuffer[30]);
			little2big_32(&txBuffer[32]);
			little2big_32(&txBuffer[36]);
			little2big_16(&txBuffer[40]);
			little2big_32(&txBuffer[42]);
			little2big_16(&txBuffer[46]);
			little2big_32(&txBuffer[48]);
			little2big_16(&txBuffer[52]);

			memcpy(&txBufferWithSID[1], &txBuffer[24], txlen);
			SendPacketWithCCSDS_AX25(&txBufferWithSID[0], txlen + 1, obc_apid, type, 25);
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		}
		else {
			completionError = I2C_READ_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
		break;
	/*--------------- ID:11 Report Error Record ----------------*/
	case Report_Error_Record:
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		if (errPacket_dump() == Error) {
			completionError = FS_IO_ERR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
		else {
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		}
		break;
	/*--------------- ID:12 Report Collected Data Number ----------------*/
	case Report_Data_Number:
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		uint16_t data_number[5] = {0};
		char path[6] = {0};
		if (parameters.crippled_Mode == 1) {
			strcpy(path, "/boot");
			if (report_Crippled_Data(path, data_number) == No_Error) {
				memcpy(&txBuffer[0], &data_number[0], 10);
				for (int i = 0; i < 5 ; i++) {
					little2big_16(&txBuffer[i * 2]);
				}
				txlen = 10 ;
				txBufferWithSID[0] = 42;
				memcpy(&txBufferWithSID[1], &txBuffer[0], txlen);
				hex_dump(&txBuffer[0], txlen);
				SendPacketWithCCSDS_AX25(&txBufferWithSID[0], txlen + 1, obc_apid, type, 25);
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			}
			else {
				completionError = FS_IO_ERR;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			if (paras[0] == 0) {
				strcpy(path, "/sd0");
			}
			else if (paras[0] == 1) {
				strcpy(path, "/sd1");
			}
			else {
				completionError = FS_IO_ERR;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
				break;
			}
			if (report_Collected_Data(path, data_number) == No_Error) {
				memcpy(&txBuffer[0], &data_number[0], 10);
				for (int i = 0; i < 5 ; i++) {
					little2big_16(&txBuffer[i * 2]);
				}
				txlen = 10 ;
				txBufferWithSID[0] = 42;
				memcpy(&txBufferWithSID[1], &txBuffer[0], txlen);
				hex_dump(&txBuffer[0], txlen);
				SendPacketWithCCSDS_AX25(&txBufferWithSID[0], txlen + 1, obc_apid, type, 25);
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			}
			else {
				completionError = FS_IO_ERR;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		break;
	/*--------------- ID:13 Report GPS's Record ----------------*/
	case Report_GPS_Record:
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		uint8_t gps_data[180];
		txlen = 180 ;
		txBufferWithSID[0] = 43;
		for (int i = 0 ; i < 10 ; i++)
		{
			if (GPS_read(gps_data, i) == No_Error) {
				memcpy(&txBufferWithSID[1], &gps_data[0], txlen);
				SendPacketWithCCSDS_AX25(&txBufferWithSID[0], txlen + 1, obc_apid, type, 25);
			}
			vTaskDelay(0.5 * delay_time_based);
		}
		sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		break;
	default:
		sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
		break;
	}
}
