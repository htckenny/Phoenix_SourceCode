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
#include <inttypes.h>

#define configure 						1
#define change_mode						2
#define Execute_sampling_downlink		3



void decodeService132(uint8_t subType, uint8_t*telecommand) {

	uint8_t completionError = 0;
	uint8_t SEUV_data[37];

	/*------------------------------------------Telecommand-----------------------------------*/
	/* note: parameter start from telecommand[9]*/

	uint16_t packet_length = (telecommand[4] << 8) + telecommand[5] - 4;
	uint8_t paras[180];
	printf("packet_length = %d\n", packet_length);
	if (packet_length > 0)
		memcpy(&paras, telecommand + 9, packet_length); // !!!!!!!!!!!!!!!!!!!!!!!!
	switch (subType) {

	/*--------------- ID:1 configure ----------------*/
	case configure:
		if (packet_length == 2) {
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report

			parameters.seuv_period = paras[0];
			parameters.seuv_sample_rate = paras[1];
			para_w();

			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); //send COMPLETE_success report
		} else
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE); //accept fail
		break;
	/*--------------- ID:2 change_mode ----------------*/
	case change_mode:
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		if (packet_length == 1) {			
			parameters.seuv_mode = paras[0];	//set the seuv mode 
			para_w();
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); //send COMPLETE_success report
		} 
		else {
			completionError = I2C_READ_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError); //send complete fail
		}

		break;
	/*--------------- ID:3 Execute a sampling and downlink ----------------*/
	case Execute_sampling_downlink:
		if (packet_length == 0) {
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
			/* Read parameter from the FS, should delete this line if parameter_init is already called */
			parameter_init();
			/* sample SEUV once with gain = 1 */
			get_a_packet(1);
			seuvFrame.samples += 1; 
			memcpy(&SEUV_data, &seuvFrame, 37);
			seuvFrame.samples -= 1; 
			hex_dump(SEUV_data, 37);
			SendDataWithCCSDS_AX25(3, SEUV_data);
			/* sample SEUV once with gain = 8 */
			get_a_packet(8);
			seuvFrame.samples += 8; 
			memcpy(&SEUV_data, &seuvFrame, 37);
			seuvFrame.samples -= 8; 
			hex_dump(SEUV_data, 37);
			SendDataWithCCSDS_AX25(3, SEUV_data);

			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); //send COMPLETE_success report
		}
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError); //send complete fail
		}		
		break;
			
	default:
		sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
		break;

	}
}


