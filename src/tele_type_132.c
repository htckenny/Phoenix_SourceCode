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

#define configure 						1
#define change_mode						2
#define Execute_sampling_downlink		3
#define Change_channel_parameter		4

void decodeService132(uint8_t subType, uint8_t*telecommand) {

	uint8_t completionError = 0;
	uint8_t SEUV_data[seuv_length];

	/*------------------------------------------Telecommand-----------------------------------*/
	/* note: parameter start from telecommand[9]*/

	uint16_t packet_length = (telecommand[4] << 8) + telecommand[5] - 4;
	uint8_t paras[8];
	if (packet_length > 0)
		memcpy(&paras, telecommand + 9, packet_length);
	switch (subType) {

	/*--------------- ID:1 configure ----------------*/
	case configure:
		if (packet_length == 2) {
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);

			parameters.seuv_period = paras[0];
			parameters.seuv_sample_rate = paras[1];
			para_w_flash();

			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		}
		else
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
		break;
	/*--------------- ID:2 change_mode ----------------*/
	case change_mode:
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
		if (packet_length == 1) {

			if (parameters.seuv_mode == 2 || parameters.seuv_mode == 4) {
				if (paras[0] == 3) {
					power_control(3, OFF);
				}
			}
			else if (parameters.seuv_mode == 3 || parameters.seuv_mode == 4) {
				if (paras[0] == 2) {
					power_control(3, ON);
				}
			}
			else {
				completionError = PARA_ERR;
				sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
			}
			parameters.seuv_mode = paras[0];	//set the seuv mode
			para_w_flash();
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		}
		else
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);

		break;
	/*--------------- ID:3 Execute a sampling and downlink ----------------*/
	case Execute_sampling_downlink:
		if (packet_length == 0) {
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
			if (parameters.seuv_mode == 2) {
				vTaskSuspend(seuv_task);
			}
			else if (parameters.seuv_mode == 3) {
				power_control(3, ON);
			}
			/* sample SEUV once with gain = 1 */
			get_a_packet(1);
			memcpy(&SEUV_data, &seuvFrame, seuv_length);
			SendDataWithCCSDS_AX25(3, SEUV_data);
			/* sample SEUV once with gain = 8 */
			get_a_packet(8);
			seuvFrame.samples ++;
			memcpy(&SEUV_data, &seuvFrame, seuv_length);
			seuvFrame.samples --;
			SendDataWithCCSDS_AX25(3, SEUV_data);
			if (parameters.seuv_mode == 2) {
				vTaskResume(seuv_task);
			}
			else if (parameters.seuv_mode == 3)
				power_control(3, OFF);

			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		}
		else {
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError);
		}
		break;
	/*--------------- ID:4 change channel's parameter ----------------*/
	case Change_channel_parameter:
		if (packet_length == 8) {
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
			/* Gain 1 configuration */
			parameters.seuv_ch1_G1_conf = paras[0];
			parameters.seuv_ch2_G1_conf = paras[1];
			parameters.seuv_ch3_G1_conf = paras[2];
			parameters.seuv_ch4_G1_conf = paras[3];
			/* Gain 8 configuration */
			parameters.seuv_ch1_G8_conf = paras[4];
			parameters.seuv_ch2_G8_conf = paras[5];
			parameters.seuv_ch3_G8_conf = paras[6];
			parameters.seuv_ch4_G8_conf = paras[7];
			para_w_flash();
			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
		}
		else
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
		break;
	default:
		sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
		break;

	}
}
