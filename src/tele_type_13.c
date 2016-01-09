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

/*------------------------------------------Telecommand-----------------------------------*/
#define script_first_part		9
#define script_middle_part		10
#define script_last_part		11
#define Ack_to_a_part			14

int script_length = 0;

void  decodeService13(uint8_t subType, uint8_t *telecommand) {

	uint16_t packet_length = (telecommand[4] << 8) + telecommand[5] - 4;
	uint8_t paras[180];
	int script_ID = 0;
	if (packet_length > 0)
		memcpy(&paras, telecommand + 9, packet_length); 
	printf("packet = %d\n", packet_length);
	script_ID = paras[0];
	printf("Scrip ID : %d\n", script_ID);
	
	switch (subType) {

	/*---------------ID:9 script_first_part	----------------*/
	case script_first_part:
		sendTelecommandReport_Success_INMS(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		printf("first\n");
		if (paras[1] == 1) {
			if(inms_script_write_flash(paras[0], &paras[2], 1, packet_length-2) == No_Error) {
				script_length = (paras[2] + (paras[3] << 8));
				sendTelecommandReport_Success_INMS(telecommand, CCSDS_S3_COMPLETE_SUCCESS); //send COMPLETE_success report
			} 
			else
				sendTelecommandReport_Failure_INMS(telecommand, CCSDS_S3_COMPLETE_FAIL, ERR_F_READ);
				
		}		
		else
			sendTelecommandReport_Failure_INMS(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE); //accept fail

		break;
	/*---------------ID:10  script_middle_part----------------*/
	case script_middle_part:
		sendTelecommandReport_Success_INMS(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
		printf("second\n");
		if (paras[1] != 1) {
			if (inms_script_write_flash(paras[0], &paras[2], 0, packet_length -2) == No_Error )
				sendTelecommandReport_Success_INMS(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else
				sendTelecommandReport_Failure_INMS(telecommand, CCSDS_S3_COMPLETE_FAIL, ERR_F_READ);
		}		
		else
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE); //accept fail

		break;
	/* --------------ID: 11  script_last_part--------------*/
	case script_last_part:
		sendTelecommandReport_Success_INMS(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS); 
		printf("final\n");
		if (paras[1] == 1){
			if (inms_script_write_flash(paras[0], &paras[2], 1, (paras[2] + (paras[3] << 8))) == No_Error) 	
				sendTelecommandReport_Success_INMS(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
			else
				sendTelecommandReport_Failure_INMS(telecommand, CCSDS_S3_COMPLETE_FAIL, ERR_F_READ);
		}
		else if (paras[1] > 1){
			if (inms_script_write_flash(paras[0], &paras[2], 0, script_length - (paras[1] - 1) * 150) == No_Error) 	
				sendTelecommandReport_Success_INMS(telecommand, CCSDS_S3_COMPLETE_SUCCESS); 
			else
				sendTelecommandReport_Failure_INMS(telecommand, CCSDS_S3_COMPLETE_FAIL, ERR_F_READ);
		}
		else
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE); 

		break;
	case Ack_to_a_part:
		/* Not used anymore */
		break;		
	default:
		sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
		break;

	}
}


