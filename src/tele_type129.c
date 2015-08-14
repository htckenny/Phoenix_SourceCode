#include <util/timestamp.h>

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
#include "subsystem.h"

void decodeService129(uint8_t subType, uint8_t * telecommand) {
	uint8_t txBuffer[254];
	uint8_t rxBuffer[254];
	uint8_t completionError = ERR_SUCCESS;
	uint8_t err;
	uint8_t rxBufferLength;
	uint8_t types = 3;
	uint8_t parameterLength;
// note: parameter start from telecommand[9]
	/*------------------------------------------Telecommand-----------------------------------*/

	/*---------------ID:1 Reset----------------*/
	if (subType == Reset ) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 1;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength); // copy command & parameter to I2C tx buffer
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR) //send I2C
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report if send fail
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:2 Set_Unix_Time-----------------*/
	else if (subType == Set_Unix_Time ) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 6;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:3 ADCS_Run_Mode----------------*/
	else if (subType == ADCS_Run_Mode) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 1;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:4 Select_Logged_Data----------------*/
	else if (subType == Selected_Logged_Data) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 13;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:5 Power_Control----------------*/
	else if (subType == Power_Control) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 5;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:6 Deploy_Magnetometer_Boom----------------*/
	else if (subType == Deploy_Magnetometer_Boom) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 1;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:7 Trigger_ADCS_Loop----------------*/
	else if (subType == Trigger_ADCS_Loop) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 0;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:17 Set_Attitude_Estimation_Mode----------------*/
	else if (subType == Set_Attitude_Estimation_Mode) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 1;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:18 Set_Attitude_Control_Mode----------------*/
	else if (subType == Set_Attitude_Control_Mode) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 4;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:19 Set_Commanded_Attitude_Angles----------------*/
	else if (subType == Set_Commanded_Attitude_Angles) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 6;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:32 Set_Wheel_Speed----------------*/
	else if (subType == Set_Wheel_Speed) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 6;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:33 Set_Magnetorquer_Output----------------*/
	else if (subType == Set_Magnetorquer_Output) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 6;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:102 Set_Startup_Mode----------------*/
	else if (subType == Set_Startup_Mode) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 1;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:64 Set_SGP4_Orbit_Parameters----------------*/
	else if (subType == Set_SGP4_Orbit_Parameters) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 64;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:80 Set_Configuration----------------*/
	else if (subType == Set_Configuration) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 236;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:81 Set_Magnetorquer_Configuration----------------*/
	else if (subType == Set_Magnetorquer_Configuration) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 13;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:82 Set_Wheel_Configuration----------------*/
	else if (subType == Set_Wheel_Configuration) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 13;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:83 Set_CSS_Configuration----------------*/
	else if (subType == Set_CSS_Configuration) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 14;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:84 Set_Sun_Sensor_Configuration----------------*/
	else if (subType == Set_Sun_Sensor_Configuration) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 17;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:85 Set_Nadir_Sensor_Configuration----------------*/
	else if (subType == Set_Nadir_Sensor_Configuration) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 57;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:86 Set_Magnetometer_Configuration----------------*/
	else if (subType == Set_Magnetometer_Configuration) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 30;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:87 Set_Rate_Sensor_Configuration----------------*/
	else if (subType == Set_Rate_Sensor_Configuration) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 6;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:88 Set_Detumbling_Control_Parameters----------------*/
	else if (subType == Set_Detumbling_Control_Parameters) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 10;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:89 Set_Ymomentum_Control_Parameters----------------*/
	else if (subType == Set_Ymomentum_Control_Parameters) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 26;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:90 Set_Moment_Of_Inertia----------------*/
	else if (subType == Set_Moment_Of_Inertia) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 24;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:91 Set_Estimation_Parameters----------------*/
	else if (subType == Set_Estimation_Parameters) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 26;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:100 Save_Configuration----------------*/
	else if (subType == Save_Configuration) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 0;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:101 Save_Orbit_Parameters----------------*/
	else if (subType == Save_Orbit_Parameters) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 0;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:110 Capture_And_Save_Image----------------*/
	else if (subType == Capture_And_Save_Image) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 10;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:114 Reset_File_List----------------*/
	else if (subType == Reset_File_List) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 0;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:115 Advance_File_List_Index----------------*/
	else if (subType == Advance_File_List_Index) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 0;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:116 Initialize_File_Download----------------*/
	else if (subType == Initialize_File_Download) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 13;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:117 Advance_File_Read_Pointer----------------*/
	else if (subType == Advance_File_Read_Pointer) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 2;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:118 Erase_File----------------*/
	else if (subType == Erase_File) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 13;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:119 Erase_All_Files----------------*/
	else if (subType == Erase_All_Files) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 0;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:120 Set_Boot_Index----------------*/
	else if (subType == Set_Boot_Index) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 1;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*--------------ID:121 Erase_Program----------------*/
	else if (subType == Erase_Program) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 1;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}



	/*--------------ID:123 Finalize_Program_Upload----------------*/
	else if (subType == Finalize_Program_Upload) { //you should modify this ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		txBuffer[0] = subType;
		parameterLength = 71;  // you should modify the size of parameter part
		memcpy(&txBuffer[1], telecommand + 9, parameterLength);
		if (i2c_master_transaction(0, adcs_node, &txBuffer, parameterLength + 1, 0, 0, adcs_delay) == E_NO_ERR)
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}


	/*------------------------------------------Telemetry-----------------------------------*/
	/*--------------ID:128 Identification---------------*/
	else if (subType == Identification) {              //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 8;                           // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:130 Communication_Status--------------*/
	else if (subType == Communication_Status) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:131 Telecommand_Acknowledge--------------*/
	else if (subType == Telecommand_Acknowledge) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 4;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:133 Reset_Cause--------------*/
	else if (subType == Reset_Cause) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 1;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:138 Actuator_Commands--------------*/
	else if (subType == Actuator_Commands) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 12;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:189 ACP_Execution_State--------------*/
	else if (subType == ACP_Execution_State) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 3;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:190 ACP_Execution_Times--------------*/
	else if (subType == ACP_Execution_Times) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 8;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:193 EDAC_And_Latchup_Counters--------------*/
	else if (subType == EDAC_And_Latchup_Counters) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 10;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:194 Startup_Mode--------------*/
	else if (subType == Startup_Mode) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 1;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:240 File_Information--------------*/
	else if (subType == File_Information) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 22;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:241 File_Block_CRC--------------*/
	else if (subType == File_Block_CRC) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}


	/*---------------ID:134 Power_Control_Selection--------------*/
	else if (subType == Power_Control_Selection) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 5;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:135 Power_And_Temperature_Measurements--------------*/
	else if (subType == Power_And_Temperature_Measurements) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 18;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:136 ADCS_State--------------*/
	else if (subType == ADCS_State) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 48;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:137 ADCS_Measurements--------------*/
	else if (subType == ADCS_Measurements) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 36;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:143 Current_Time--------------*/
	else if (subType == Current_Time) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:144 Current_State--------------*/
	else if (subType == Current_State) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:145 Estimated_Attitude_Angles--------------*/
	else if (subType == Estimated_Attitude_Angles) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:146 Estimated_Angular_Rates--------------*/
	else if (subType == Estimated_Angular_Rates) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:147 Satellite_Position_LLH--------------*/
	else if (subType == Satellite_Position_LLH) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:148 Satellite_Velocity_ECI--------------*/
	else if (subType == Satellite_Velocity_ECI) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:149 Magnetic_Field_Vector--------------*/
	else if (subType == Magnetic_Field_Vector) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:150 Coarse_Sun_Vector--------------*/
	else if (subType == Coarse_Sun_Vector) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:151 Fine_Sun_Vector--------------*/
	else if (subType == Fine_Sun_Vector) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:152 Nadir_Vector--------------*/
	else if (subType == Nadir_Vector) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:153 Rate_Sensor_Rates--------------*/
	else if (subType == Rate_Sensor_Rates) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:154 Wheel_Speed--------------*/
	else if (subType == Wheel_Speed) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:173 CubeSense_Current_Measurements--------------*/
	else if (subType == CubeSense_Current_Measurements) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:174 CubeControl_Current_Measurements--------------*/
	else if (subType == CubeControl_Current_Measurements) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:175 Peripheral_Current_And_Temperature_Measurements--------------*/
	else if (subType == Peripheral_Current_And_Temperature_Measurements) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:139 Raw_Sensor_Measurements--------------*/
	else if (subType == Raw_Sensor_Measurements) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 60;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:163 Angular_Rate_Covariance--------------*/
	else if (subType == Angular_Rate_Covariance) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:164 Raw_Nadir_Sensor--------------*/
	else if (subType == Raw_Nadir_Sensor) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:165 Raw_Sun_Sensor--------------*/
	else if (subType == Raw_Sun_Sensor) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:166 Raw_CSS--------------*/
	else if (subType == Raw_CSS) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:167 Raw_Magnetometer--------------*/
	else if (subType == Raw_Magnetometer) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:168 Raw_GPS_Status--------------*/
	else if (subType == Raw_GPS_Status) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:169 Raw_GPS_Time--------------*/
	else if (subType == Raw_GPS_Time) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:170 Raw_GPS_X--------------*/
	else if (subType == Raw_GPS_X) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:171 Raw_GPS_Y--------------*/
	else if (subType == Raw_GPS_Y) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:172 Raw_GPS_Z--------------*/
	else if (subType == Raw_GPS_Z) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:140 Estimation_Data--------------*/
	else if (subType == Estimation_Data) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 42;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:157 IGRF_Modelled_Magnetic_Field_Vector--------------*/
	else if (subType == IGRF_Modelled_Magnetic_Field_Vector) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:158 Modelled_Sun_Vector--------------*/
	else if (subType == Modelled_Sun_Vector) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:159 Estimated_Gyro_Bias--------------*/
	else if (subType == Estimated_Gyro_Bias) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:160 Estimated_Innovation_Vector--------------*/
	else if (subType == Estimated_Innovation_Vector) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:161 Quaternion_Error_Vector--------------*/
	else if (subType == Quaternion_Error_Vector) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:162 Quaternion_Covariance--------------*/
	else if (subType == Quaternion_Covariance) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:155 Magnetorquer_Command--------------*/
	else if (subType == Magnetorquer_Command) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:156 Wheel_Speed_Commands--------------*/
	else if (subType == Wheel_Speed_Commands) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 6;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:191 SGP4_Orbit_Parameters--------------*/
	else if (subType == SGP4_Orbit_Parameters) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 64;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:192 Configuration--------------*/
	else if (subType == Configuration) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 236;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:230 Status_Of_Image_Capture_And_Save_Operation--------------*/
	else if (subType == Status_Of_Image_Capture_And_Save_Operation) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 16;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:250 Uploaded_Program_Status--------------*/
	else if (subType == Uploaded_Program_Status) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 3;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}

	/*---------------ID:251 Get_Flash_Program_List--------------*/
	else if (subType == Get_Flash_Program_List) {                 //you should modify this subType ID
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		txBuffer[0] = subType;
		rxBufferLength = 255;                                // you should modify rxBufferLength
		if (i2c_master_transaction(0, adcs_node, &txBuffer, 1, &rxBuffer, rxBufferLength, adcs_delay) == E_NO_ERR) {
			err = SendPacketWithCCSDS_AX25(&rxBuffer, rxBufferLength , adcs_apid, types, subType);
			if (err == ERR_SUCCESS)
				sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else {
				completionError = err;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
		}
		else {                                                                 //send COMPLETE_FAIL report
			completionError = I2C_SEND_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
	}


}  /* end of decodeService129*/

