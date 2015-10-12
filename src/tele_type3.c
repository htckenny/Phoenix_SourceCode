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

#define Report_HK_State		1
#define Report_COM_HK_1		2
#define Report_EPS_HK		3
#define Report_Parameter	4
#define Report_COM_HK_2		5


/* Helper function */
uint16_t Interface_tmp_get() {
	uint8_t rx[5];

	uint8_t tx[2];
	tx[0] = 0xF0;	//0x221
	tx[1] = 0xF0;
	i2c_master_transaction(0, interface_node, &tx, 2, 0, 0, 0);

	if (i2c_master_transaction(0, interface_node, 0 , 0, &rx, 4, seuv_delay) == E_NO_ERR) {
		return ( rx[0] * 256 + rx[1]);

	} else
	return 0;
}

uint16_t Interface_inms_thermistor_get() {
	uint8_t rx[5];

	uint8_t tx[2];
	tx[0] = 0xD0;	//0x221
	tx[1] = 0xD0;
	i2c_master_transaction(0, interface_node, &tx, 2, 0, 0, 0);

	if (i2c_master_transaction(0, interface_node, 0 , 0, &rx, 4, seuv_delay) == E_NO_ERR) {
		return ( rx[0] * 256 + rx[1]);

	} else
	return 0;
}
uint16_t Interface_3V3_current_get() {
	uint8_t rx[5];

	uint8_t tx[2];
	tx[0] = 0x90;
	tx[1] = 0x90;
	i2c_master_transaction(0, interface_node, &tx, 2, 0, 0, 0);

	if (i2c_master_transaction(0, interface_node, 0 , 0, &rx, 4, seuv_delay) == E_NO_ERR) {
		return ( rx[0] * 256 + rx[1]);

	} else
	return 0;

}

uint16_t Interface_5V_current_get() {
	uint8_t rx[5];

	uint8_t tx[2];
	tx[0] = 0xB0;	//0x189
	tx[1] = 0xB0;
	i2c_master_transaction(0, interface_node, &tx, 2, 0, 0, 0);

	if (i2c_master_transaction(0, interface_node, 0 , 0, &rx, 4, seuv_delay) == E_NO_ERR) {
			return ( rx[0] * 256 + rx[1]);

		} else
		return 0;


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

		memcpy(&txBuffer[0], &HK_frame.mode_status_flag, 1);
		memcpy(&txBuffer[1], &parameters.inms_store_count, 4);
		memcpy(&txBuffer[5], &parameters.seuv_store_count, 4);
		memcpy(&txBuffer[9], &parameters.hk_store_count, 4);
		memcpy(&txBuffer[13], &parameters.wod_store_count, 4);

		buffs=Interface_3V3_current_get();
		memcpy(&txBuffer[17], &buffs, 2);

		buffs=Interface_5V_current_get();
		memcpy(&txBuffer[19], &buffs, 2);

		buffs=Interface_tmp_get();
		memcpy(&txBuffer[21], &buffs, 2);

		buffs=Interface_inms_thermistor_get();
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

		// memcpy(&txBuffer[0], &parameters.vbat_safe_threshold, 2);
		// memcpy(&txBuffer[2], &parameters.vbat_recover_threshold, 2);
		// txBuffer[4] = parameters.hk_collect_period;
		// txBuffer[5] = parameters.seuv_period;
		// txBuffer[6] = parameters.beacon_period;
		// printf("%d\n", sizeof(parameter_t));
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

	default:

		sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
		break;
	}
}
