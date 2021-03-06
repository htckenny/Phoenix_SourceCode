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

extern void little2big_64(uint8_t * input_data);
extern void little2big_32(uint8_t * input_data);
extern void little2big_16(uint8_t * input_data);

void decodeService131(uint8_t subType, uint8_t * telecommand) {
	uint8_t txBuffer[254];
	uint8_t rxBuffer[254];
	uint8_t rxBufferWithSID[254];
	uint8_t completionError = ERR_SUCCESS;
	uint8_t err;
	uint8_t rxBufferLength;
	uint8_t types = 131;
	uint16_t parameterLength;
	/*------------------------------------------Telecommand-----------------------------------*/

	/*---------------ID:1 Reset----------------*/
	if (subType == Reset ) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 1;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR) //send I2C
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:2 Set_Unix_Time-----------------*/
	else if (subType == Set_Unix_Time ) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 6;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:3 ADCS_Run_Mode----------------*/
	else if (subType == ADCS_Run_Mode) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 1;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:4 Select_Logged_Data----------------*/
	else if (subType == Selected_Logged_Data) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 13;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:5 Power_Control----------------*/
	else if (subType == Power_Control) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 5;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:6 Deploy_Magnetometer_Boom----------------*/

	else if (subType == Deploy_Magnetometer_Boom) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		if (HK_frame.mode_status_flag != 0)
		{
			magnetometer_deploy = 1;
#if mag_meter_deploy
			// txBuffer[0] = subType;
			// parameterLength = 1;
			// memcpy(&txBuffer[1], telecommand + 9, parameterLength);
			// if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			// 	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			// else {
			// 	completionError = I2C_SEND_ERROR;
			// 	sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			// }
#else
			printf("deploy magnetometer !! (simulation)\n");
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
#endif
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}
	/*--------------ID:7 Trigger_ADCS_Loop----------------*/
	else if (subType == Trigger_ADCS_Loop) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 0;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:17 Set_Attitude_Estimation_Mode----------------*/
	else if (subType == Set_Attitude_Estimation_Mode) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 1;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:18 Set_Attitude_Control_Mode----------------*/
	else if (subType == Set_Attitude_Control_Mode) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 4;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);

		little2big_16(&txBuffer[1 + 2]);

		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:19 Set_Commanded_Attitude_Angles----------------*/
	else if (subType == Set_Commanded_Attitude_Angles) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 6;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		for (int i = 0; i < 3; i++) {
			little2big_16(&txBuffer[1 + 2 * i]);
		}

		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:32 Set_Wheel_Speed----------------*/
	else if (subType == Set_Wheel_Speed) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 6;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		for (int i = 0; i < 3; i++) {
			little2big_16(&txBuffer[1 + 2 * i]);
		}
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:33 Set_Magnetorquer_Output----------------*/
	else if (subType == Set_Magnetorquer_Output) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 6;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		for (int i = 0; i < 3; i++) {
			little2big_16(&txBuffer[1 + 2 * i]);
		}
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:102 Set_Startup_Mode----------------*/
	else if (subType == Set_Startup_Mode) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 1;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:64 Set_SGP4_Orbit_Parameters----------------*/
	else if (subType == Set_SGP4_Orbit_Parameters) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 64;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		for (int i = 0; i < 8 ; i++)
			little2big_64(&txBuffer[1 + 8 * i]);
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:80 Set_Configuration----------------*/
	else if (subType == Set_Configuration) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 236;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:81 Set_Magnetorquer_Configuration----------------*/
	else if (subType == Set_Magnetorquer_Configuration) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 13;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);

		little2big_16(&txBuffer[1 + 3]);
		little2big_16(&txBuffer[1 + 5]);
		little2big_16(&txBuffer[1 + 7]);
		little2big_16(&txBuffer[1 + 10]);

		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:82 Set_Wheel_Configuration----------------*/
	else if (subType == Set_Wheel_Configuration) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 13;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		little2big_16(&txBuffer[1 + 0]);
		little2big_16(&txBuffer[1 + 2]);
		little2big_16(&txBuffer[1 + 5]);
		little2big_16(&txBuffer[1 + 7]);
		little2big_16(&txBuffer[1 + 9]);
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:83 Set_CSS_Configuration----------------*/
	else if (subType == Set_CSS_Configuration) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 14;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:84 Set_Sun_Sensor_Configuration----------------*/
	else if (subType == Set_Sun_Sensor_Configuration) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 17;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		little2big_16(&txBuffer[1 + 0]);
		little2big_16(&txBuffer[1 + 2]);
		little2big_16(&txBuffer[1 + 4]);
		little2big_16(&txBuffer[1 + 12]);
		little2big_16(&txBuffer[1 + 14]);
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:85 Set_Nadir_Sensor_Configuration----------------*/
	else if (subType == Set_Nadir_Sensor_Configuration) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 57;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		for (int i = 0; i < 3; i++)
			little2big_16(&txBuffer[1 + 2 * i]);
		for (int i = 6; i < 8; i++)
			little2big_16(&txBuffer[1 + 2 * i]);
		for (int i = 8; i < 28; i++)
			little2big_16(&txBuffer[1 + 1 + 2 * i]);


		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:86 Set_Magnetometer_Configuration----------------*/
	else if (subType == Set_Magnetometer_Configuration) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 30;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		for (int i = 0; i < 15; i++)
			little2big_16(&txBuffer[1 + 2 * i]);
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:87 Set_Rate_Sensor_Configuration----------------*/
	else if (subType == Set_Rate_Sensor_Configuration) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 6;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		for (int i = 0; i < 3; i++)
			little2big_16(&txBuffer[1 + 2 * i]);
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:88 Set_Detumbling_Control_Parameters----------------*/
	else if (subType == Set_Detumbling_Control_Parameters) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 10;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		for (int i = 0; i < 2; i++)
			little2big_32(&txBuffer[1 + 4 * i]);
		little2big_16(&txBuffer[1 + 8]);
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:89 Set_Ymomentum_Control_Parameters----------------*/
	else if (subType == Set_Ymomentum_Control_Parameters) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 26;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		for (int i = 0; i < 7; i++)
			little2big_32(&txBuffer[1 + 4 * i]);
		little2big_16(&txBuffer[1 + 28]);
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:90 Set_Moment_Of_Inertia----------------*/
	else if (subType == Set_Moment_Of_Inertia) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 24;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		for (int i = 0; i < 6; i++)
			little2big_32(&txBuffer[1 + 4 * i]);
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:91 Set_Estimation_Parameters----------------*/
	else if (subType == Set_Estimation_Parameters) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 26;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		for (int i = 0; i < 6; i++)
			little2big_32(&txBuffer[1 + 4 * i]);
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:100 Save_Configuration----------------*/
	else if (subType == Save_Configuration) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 0;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:101 Save_Orbit_Parameters----------------*/
	else if (subType == Save_Orbit_Parameters) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 0;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:110 Capture_And_Save_Image----------------*/
	else if (subType == Capture_And_Save_Image) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 10;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:114 Reset_File_List----------------*/
	else if (subType == Reset_File_List) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 0;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:115 Advance_File_List_Index----------------*/
	else if (subType == Advance_File_List_Index) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 0;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:116 Initialize_File_Download----------------*/
	else if (subType == Initialize_File_Download) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 13;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:117 Advance_File_Read_Pointer----------------*/
	else if (subType == Advance_File_Read_Pointer) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 2;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:118 Erase_File----------------*/
	else if (subType == Erase_File) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 13;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:119 Erase_All_Files----------------*/
	else if (subType == Erase_All_Files) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 0;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:120 Set_Boot_Index----------------*/
	else if (subType == Set_Boot_Index) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 1;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:121 Erase_Program----------------*/
	else if (subType == Erase_Program) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 1;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}
	/*--------------ID:122 Upload a 256 bytes block of program code to flash----------------*/
	else if (subType == 122) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 259;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		little2big_16(&txBuffer[1 + 1]);
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}
	/*--------------ID:123 Finalize_Program_Upload----------------*/
	else if (subType == Finalize_Program_Upload) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 71;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		little2big_32(&txBuffer[1 + 1]);
		little2big_16(&txBuffer[1 + 5]);
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}
	/*--------------ID:124 Reset CubeComputer ----------------*/
	else if (subType == 124) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}
	/*--------------ID:125 Run Selected Program ----------------*/
	else if (subType == 125) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}
	/*--------------ID:126 Copy Program to internal  ----------------*/
	else if (subType == 126) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		parameterLength = 2;
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}
	/*--------------ID:127 Reset CubeComputer ----------------*/
	else if (subType == 127) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}
	/*------------------------------------------Telemetry-----------------------------------*/
	/*--------------ID:128 Identification---------------*/
	else if (subType == Identification) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 8;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:130 Communication_Status--------------*/
	else if (subType == Communication_Status) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:131 Telecommand_Acknowledge--------------*/
	else if (subType == Telecommand_Acknowledge) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 4;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:133 Reset_Cause--------------*/
	else if (subType == Reset_Cause) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 1;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:138 Actuator_Commands--------------*/
	else if (subType == Actuator_Commands) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 12;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:189 ACP_Execution_State--------------*/
	else if (subType == ACP_Execution_State) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 3;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:190 ACP_Execution_Times--------------*/
	else if (subType == ACP_Execution_Times) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 8;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:193 EDAC_And_Latchup_Counters--------------*/
	else if (subType == EDAC_And_Latchup_Counters) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 10;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:194 Startup_Mode--------------*/
	else if (subType == Startup_Mode) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 1;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:240 File_Information--------------*/
	else if (subType == File_Information) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 22;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:241 File_Block_CRC--------------*/
	else if (subType == File_Block_CRC) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}


	/*---------------ID:134 Power_Control_Selection--------------*/
	else if (subType == Power_Control_Selection) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 5;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			rxBufferWithSID[0] = subType;
			memcpy(&rxBufferWithSID[1], &rxBuffer[0], rxBufferLength);
			err = SendPacketWithCCSDS_AX25(&rxBufferWithSID[0], rxBufferLength + 1, adcs_apid, 3, 25);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:135 Power_And_Temperature_Measurements--------------*/
	else if (subType == Power_And_Temperature_Measurements) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 18;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			little2big_16(&rxBuffer[0]);
			for (int i = 0 ; i < 6; i++) {
				little2big_16(&rxBuffer[4 + 2 * i]);
			}
			rxBufferWithSID[0] = subType;
			memcpy(&rxBufferWithSID[1], &rxBuffer[0], rxBufferLength);
			err = SendPacketWithCCSDS_AX25(&rxBufferWithSID[0], rxBufferLength + 1, adcs_apid, 3, 25);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:136 ADCS_State--------------*/
	else if (subType == ADCS_State) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 48;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			little2big_32(&rxBuffer[0]);
			little2big_16(&rxBuffer[4]);
			for (int i = 0 ; i < 18; i++) {
				little2big_16(&rxBuffer[12 + 2 * i]);
			}

			rxBufferWithSID[0] = subType;
			memcpy(&rxBufferWithSID[1], &rxBuffer[0], rxBufferLength);
			err = SendPacketWithCCSDS_AX25(&rxBufferWithSID[0], rxBufferLength + 1, adcs_apid, 3, 25);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:137 ADCS_Measurements--------------*/
	else if (subType == ADCS_Measurements) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 36;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			for (int i = 0 ; i < 18; i++) {
				little2big_16(&rxBuffer[0 + 2 * i]);
			}
			rxBufferWithSID[0] = subType;
			memcpy(&rxBufferWithSID[1], &rxBuffer[0], rxBufferLength);
			err = SendPacketWithCCSDS_AX25(&rxBufferWithSID[0], rxBufferLength + 1, adcs_apid, 3, 25);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:143 Current_Time--------------*/
	else if (subType == Current_Time) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:144 Current_State--------------*/
	else if (subType == Current_State) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;
		uint8_t txBufferWithSID[7];
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			txBufferWithSID[0] = 144;
			memcpy(&txBufferWithSID[1], &rxBuffer[0], rxBufferLength);
			err = SendPacketWithCCSDS_AX25(&txBufferWithSID[0], rxBufferLength + 1, adcs_apid, 3, 25);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:145 Estimated_Attitude_Angles--------------*/
	else if (subType == Estimated_Attitude_Angles) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:146 Estimated_Angular_Rates--------------*/
	else if (subType == Estimated_Angular_Rates) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:147 Satellite_Position_LLH--------------*/
	else if (subType == Satellite_Position_LLH) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:148 Satellite_Velocity_ECI--------------*/
	else if (subType == Satellite_Velocity_ECI) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:149 Magnetic_Field_Vector--------------*/
	else if (subType == Magnetic_Field_Vector) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:150 Coarse_Sun_Vector--------------*/
	else if (subType == Coarse_Sun_Vector) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:151 Fine_Sun_Vector--------------*/
	else if (subType == Fine_Sun_Vector) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:152 Nadir_Vector--------------*/
	else if (subType == Nadir_Vector) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:153 Rate_Sensor_Rates--------------*/
	else if (subType == Rate_Sensor_Rates) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:154 Wheel_Speed--------------*/
	else if (subType == Wheel_Speed) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:173 CubeSense_Current_Measurements--------------*/
	else if (subType == CubeSense_Current_Measurements) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:174 CubeControl_Current_Measurements--------------*/
	else if (subType == CubeControl_Current_Measurements) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:175 Peripheral_Current_And_Temperature_Measurements--------------*/
	else if (subType == Peripheral_Current_And_Temperature_Measurements) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:139 Raw_Sensor_Measurements--------------*/
	else if (subType == Raw_Sensor_Measurements) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 60;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {

			little2big_16(&rxBuffer[0]);
			little2big_16(&rxBuffer[2]);
			little2big_16(&rxBuffer[6]);
			little2big_16(&rxBuffer[8]);
			little2big_16(&rxBuffer[18]);
			little2big_16(&rxBuffer[20]);
			little2big_16(&rxBuffer[22]);
			little2big_16(&rxBuffer[30]);
			little2big_32(&rxBuffer[32]);

			little2big_32(&rxBuffer[36]);
			little2big_16(&rxBuffer[40]);
			little2big_32(&rxBuffer[42]);
			little2big_16(&rxBuffer[46]);
			little2big_32(&rxBuffer[48]);
			little2big_16(&rxBuffer[52]);


			rxBufferWithSID[0] = subType;
			memcpy(&rxBufferWithSID[1], &rxBuffer[0], rxBufferLength);
			err = SendPacketWithCCSDS_AX25(&rxBufferWithSID[0], rxBufferLength + 1, adcs_apid, 3, 25);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:163 Angular_Rate_Covariance--------------*/
	else if (subType == Angular_Rate_Covariance) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:164 Raw_Nadir_Sensor--------------*/
	else if (subType == Raw_Nadir_Sensor) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:165 Raw_Sun_Sensor--------------*/
	else if (subType == Raw_Sun_Sensor) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:166 Raw_CSS--------------*/
	else if (subType == Raw_CSS) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:167 Raw_Magnetometer--------------*/
	else if (subType == Raw_Magnetometer) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:168 Raw_GPS_Status--------------*/
	else if (subType == Raw_GPS_Status) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:169 Raw_GPS_Time--------------*/
	else if (subType == Raw_GPS_Time) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:170 Raw_GPS_X--------------*/
	else if (subType == Raw_GPS_X) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:171 Raw_GPS_Y--------------*/
	else if (subType == Raw_GPS_Y) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:172 Raw_GPS_Z--------------*/
	else if (subType == Raw_GPS_Z) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:140 Estimation_Data--------------*/
	else if (subType == Estimation_Data) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 42;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			for (int i = 0 ; i < 21; i++) {
				little2big_16(&rxBuffer[0 + 2 * i]);
			}
			rxBufferWithSID[0] = subType;
			memcpy(&rxBufferWithSID[1], &rxBuffer[0], rxBufferLength);
			err = SendPacketWithCCSDS_AX25(&rxBufferWithSID[0], rxBufferLength + 1, adcs_apid, 3, 25);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:157 IGRF_Modelled_Magnetic_Field_Vector--------------*/
	else if (subType == IGRF_Modelled_Magnetic_Field_Vector) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:158 Modelled_Sun_Vector--------------*/
	else if (subType == Modelled_Sun_Vector) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:159 Estimated_Gyro_Bias--------------*/
	else if (subType == Estimated_Gyro_Bias) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:160 Estimated_Innovation_Vector--------------*/
	else if (subType == Estimated_Innovation_Vector) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:161 Quaternion_Error_Vector--------------*/
	else if (subType == Quaternion_Error_Vector) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:162 Quaternion_Covariance--------------*/
	else if (subType == Quaternion_Covariance) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:155 Magnetorquer_Command--------------*/
	else if (subType == Magnetorquer_Command) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			for (int i = 0; i < 3 ; i++)
				little2big_16(&rxBuffer[2 * i]);

			rxBufferWithSID[0] = subType;
			memcpy(&rxBufferWithSID[1], &rxBuffer[0], rxBufferLength);
			err = SendPacketWithCCSDS_AX25(&rxBufferWithSID[0], rxBufferLength + 1, adcs_apid, 3, 25);

			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:156 Wheel_Speed_Commands--------------*/
	else if (subType == Wheel_Speed_Commands) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			for (int i = 0; i < 3 ; i++)
				little2big_16(&rxBuffer[2 * i]);

			rxBufferWithSID[0] = subType;
			memcpy(&rxBufferWithSID[1], &rxBuffer[0], rxBufferLength);
			err = SendPacketWithCCSDS_AX25(&rxBufferWithSID[0], rxBufferLength + 1, adcs_apid, 3, 25);

			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:191 SGP4_Orbit_Parameters--------------*/
	else if (subType == SGP4_Orbit_Parameters) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 64;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			for (int i = 0; i < 8 ; i++)
				little2big_64(&rxBuffer[8 * i]);
			rxBufferWithSID[0] = subType;
			memcpy(&rxBufferWithSID[1], &rxBuffer[0], rxBufferLength);
			err = SendPacketWithCCSDS_AX25(&rxBufferWithSID[0], rxBufferLength + 1, adcs_apid, 3, 25);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:192 Configuration--------------*/
	else if (subType == Configuration) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 240;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {

			little2big_16(&rxBuffer[3]);
			little2big_16(&rxBuffer[5]);
			little2big_16(&rxBuffer[7]);
			little2big_16(&rxBuffer[10]);
			little2big_16(&rxBuffer[13]);
			little2big_16(&rxBuffer[15]);
			little2big_16(&rxBuffer[18]);
			little2big_16(&rxBuffer[20]);
			little2big_16(&rxBuffer[22]);
			little2big_16(&rxBuffer[40]);
			little2big_16(&rxBuffer[42]);
			little2big_16(&rxBuffer[44]);
			little2big_16(&rxBuffer[52]);
			little2big_16(&rxBuffer[54]);
			little2big_16(&rxBuffer[57]);
			little2big_16(&rxBuffer[59]);
			little2big_16(&rxBuffer[61]);
			little2big_16(&rxBuffer[69]);
			little2big_16(&rxBuffer[71]);
			for (int i = 0; i < 38; ++i) {
				little2big_16(&rxBuffer[74 + 2 * i]);
			}

			little2big_32(&rxBuffer[150]);
			little2big_32(&rxBuffer[154]);
			little2big_16(&rxBuffer[158]);
			for (int i = 0; i < 7; ++i) {
				little2big_32(&rxBuffer[160 + 4 * i]);
			}
			little2big_16(&rxBuffer[188]);
			for (int i = 0; i < 12; ++i) {
				little2big_32(&rxBuffer[190 + 4 * i]);
			}

			rxBufferWithSID[0] = 192;
			memcpy(&rxBufferWithSID[1], &rxBuffer[0], 150);
			err = SendPacketWithCCSDS_AX25(&rxBufferWithSID[0], 150 + 1, adcs_apid, 3, 25);
			vTaskDelay(1 * delay_time_based);
			rxBufferWithSID[0] = 195;
			memcpy(&rxBufferWithSID[1], &rxBuffer[150], 90);
			err = SendPacketWithCCSDS_AX25(&rxBufferWithSID[0], 90 + 1, adcs_apid, 3, 25);

			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:230 Status_Of_Image_Capture_And_Save_Operation--------------*/
	else if (subType == Status_Of_Image_Capture_And_Save_Operation) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 16;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:250 Uploaded_Program_Status--------------*/
	else if (subType == Uploaded_Program_Status) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 3;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			little2big_16(&rxBuffer[1]);
			rxBufferWithSID[0] = subType;
			memcpy(&rxBufferWithSID[1], &rxBuffer[0], rxBufferLength);
			err = SendPacketWithCCSDS_AX25(&rxBufferWithSID[0], rxBufferLength + 1, adcs_apid, 3, 25);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:251 Get_Flash_Program_List--------------*/
	else if (subType == Get_Flash_Program_List) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 255;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}
	/*---------------ID:252 Get_Flash_Program_List--------------*/
	else if (subType == 252) {
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 2;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			rxBufferWithSID[0] = subType;
			memcpy(&rxBufferWithSID[1], &rxBuffer[0], rxBufferLength);
			err = SendPacketWithCCSDS_AX25(&rxBufferWithSID[0], rxBufferLength + 1, adcs_apid, 3, 25);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}
}