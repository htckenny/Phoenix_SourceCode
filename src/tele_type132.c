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

void decodeService132(uint8_t subType, uint8_t*telecommand) {
	uint8_t txlen;
	timestamp_t t;

	uint8_t ch;
	uint8_t count;   // error count , 0 = no error
	uint8_t completionError;
// note: parameter start from telecommand[9]
	/*------------------------------------------Telecommand-----------------------------------*/


#define configure 1
#define get_and_download 2


	uint16_t packet_length = (telecommand[4] << 8) + telecommand[5] - 4;
	uint8_t type = 132;
	uint8_t paras[180];
	if (packet_length > 0)
		memcpy(&paras, telecommand + 9, packet_length); // !!!!!!!!!!!!!!!!!!!!!!!!
	switch (subType) {

	/*---------------ID:1 configure----------------*/
	case configure:
		if (packet_length == 6) {
			sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report

			parameters.seuv_period = paras[0];
			parameters.seuv_sample_rate = paras[1];
			para_w();

			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); //send COMPLETE_success report
		} else
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE); //accept fail
		break;
	/*---------------ID:2 get_and_download----------------*/
	case get_and_download:
		sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report

		power_control(3, ON);
		vTaskDelay(2000);

		count = 0;
		for (ch = 1; ch < 5; ch++)
			count += seuv_take_data(ch);
		power_control(3, OFF);
		if (count == 0) {
			t.tv_sec = 0;
			t.tv_nsec = 0;
			obc_timesync(&t, 6000);  //get time
			seuvFrame.packettime = csp_hton32(t.tv_sec);
			txlen = (uint8_t)sizeof(seuv_frame_t);
			SendPacketWithCCSDS_AX25(&seuvFrame.packettime, txlen, seuv_apid, type, subType);


			sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS); //send COMPLETE_success report

		} else {
			completionError = I2C_READ_ERROR;
			sendTelecommandReport_Failure(telecommand, CCSDS_S3_COMPLETE_FAIL, completionError); //send complete fail
		}

		break;
	default:
		sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
		break;

	}
}


