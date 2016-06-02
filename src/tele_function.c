#include <util/timestamp.h>
#include <util/hexdump.h>
#include <freertos/FreeRTOS.h>
#include <dev/i2c.h>
#include <csp/csp_endian.h>
#include <string.h>
#include <nanomind.h>
#include <time.h>

#include "subsystem.h"
#include "parameter.h"
#include "crc16.h"
#include "tele_function.h"
#include "fs.h"

int set_tx_rate(uint8_t mode) {
	/**
	 * mode = 1, Baud rate = 1200 bps
	 * mode = 8, Baud rate = 9600 bps
	 */
	uint8_t txbuf[2];
	txbuf[0] = com_tx_rate;
	txbuf[1] = mode;
	if (i2c_master_transaction_2(0, com_tx_node, &txbuf, 2, 0, 0, com_delay) == E_NO_ERR)
		return No_Error;
	else
		return Error;

}
void set_Call_Sign(int SSID) {

	uint8_t txdata[8];
	txdata[0] = 0x22; 	// set TO Call-Sign //
	txdata[1] = 78; 	// NCKUGS
	txdata[2] = 67;
	txdata[3] = 75;
	txdata[4] = 85;
	txdata[5] = 71;
	txdata[6] = 83;
	txdata[7] = 0x60 + 2 * (uint8_t)SSID;

	i2c_master_transaction_2(0, com_tx_node, &txdata, 8, 0, 0, com_delay);

	txdata[0] = 0x23; 	// set FROM Call-Sign //
	txdata[1] = 84; 	// TW01TN
	txdata[2] = 87;
	txdata[3] = 48;
	txdata[4] = 49;
	txdata[5] = 84;
	txdata[6] = 78;
	txdata[7] = 0x60 + 2 * (uint8_t)SSID;

	i2c_master_transaction_2(0, com_tx_node, &txdata, 8, 0, 0, com_delay);
}

// Generates sequenceCount , reset if overflow
uint8_t TC_Count() {

	if (parameters.tc_count == 4)
		parameters.tc_count = 0;

	parameters.tc_count = parameters.tc_count + 1;
	return parameters.tc_count - 1;
}

// Generates sequenceCount , reset if overflow
uint8_t AX25_Sequence_Count() {
	if (parameters.ax25_sequence_count == 255) {
		parameters.ax25_sequence_count = 0;
		return parameters.ax25_sequence_count;
	}
	parameters.ax25_sequence_count =
	    parameters.ax25_sequence_count + 1;
	return parameters.ax25_sequence_count;
}
/* Generates sequenceCount , reset if overflow */
uint16_t CCSDS_GetSequenceCount(uint16_t apid) {
	/* OBC related command */
	if (apid == obc_apid) {
		if (parameters.obc_packet_sequence_count == 16383) {
			parameters.obc_packet_sequence_count = 0;
		}
		else
			parameters.obc_packet_sequence_count ++ ;
		return parameters.obc_packet_sequence_count;
	}
	else
		return 255;
}

// Generates a complete telemetry packet with the values and data specified
uint8_t CCSDS_GenerateTelemetryPacketWithTime(uint8_t* telemetryBuffer,
        uint8_t* telemetryBufferSize, uint16_t apid, uint8_t serviceType,
        uint8_t serviceSubtype, uint8_t* sourceData, uint8_t sourceDataLength,
        uint32_t time) {

	uint16_t chk;   					// CRC syndrome
	uint16_t sequenceCount;   			// Sequence Count of packet
	uint8_t packetLength;   			// Total length of packet

	uint16_t packetLengthFieldValue;  	// Value of Packet Length header field

	uint8_t *dataPtr;  					// For source data copy
	uint8_t *dataEndPtr;

	if (sourceData == NULL)
		return ERR_MISSING_PARAMETER; 	// No data
	if (telemetryBuffer == NULL)
		return ERR_MISSING_PARAMETER; 	// No telemetry buffer
	if (telemetryBufferSize == NULL)
		return ERR_MISSING_PARAMETER; 	// No telemetry buffer size

	// Total packet length must be <= than 235 octets (otherwise it cannot be transmitted in COM board)
	if (sourceDataLength > (235 - TM_NONDATA_SIZE - AX25_2ed_size - 1))
		return ERR_CCSDS_TOO_MUCH_DATA; // Data too big

	// Total packet length (not value of packet length field !)
	packetLength = TM_NONDATA_SIZE + sourceDataLength;

	// Test if telemetry buffer big enough
	if (*telemetryBufferSize < packetLength)
		return ERR_CCSDS_BUFFER_TOO_SMALL; // Buffer too small

	// Get sequence count
	sequenceCount = CCSDS_GetSequenceCount(apid);

	// Packet Header (Packet ID, Packet Sequence Control and Packet Length) (48 bits)
	// Packet ID (16 bits)
	telemetryBuffer[0] = 0x08 | ((uint8_t)(apid >> 8) & 0x07);
	// Version number (3 bits) = 0, Type (1bit) = 0, Data Field Header Flag (1 bit) = 1, APID (11 bits) -> 3 MSB
	telemetryBuffer[1] = (uint8_t)(apid);   // APID (11 bits) -> 8 LSB
	// Packet Sequence Control (16 bits)
	telemetryBuffer[2] = 0xC0 | ((uint8_t)(sequenceCount >> 8) & 0x3F);
	// Grouping Flag (2 bits), Source Sequence Count (14 bits) -> 6 MSB

	telemetryBuffer[3] = (uint8_t)(sequenceCount);
	// Source Sequence  Count (14 bits) -> 8 LSB

	// Packet Length (16 bits)
	packetLengthFieldValue = packetLength - 7;
	telemetryBuffer[4] = (uint8_t)(packetLengthFieldValue >> 8); 	// Packet Length (16 bits) 	-> 8 MSB
	telemetryBuffer[5] = (uint8_t)(packetLengthFieldValue); 		// 							-> 8 LSB

	// Packet Data Field (Telemetry Data Field Header, Source Data and Packet Error Control) (Variable)
	// Telemetry Data Field Header (56 bits)
	telemetryBuffer[6] = 0x10; 				// Spare (1 bit) = 0, PUS Version Number (3 bits) = 1, Spare (4 bits) = 0
	telemetryBuffer[7] = serviceType; 		// Service Type (8 bits)
	telemetryBuffer[8] = serviceSubtype; 	// Service Subtype (8 bits)

	// Absolute Time (40 bits, 5 octets)
	// CUC with 4 octets of coarse time and 1 octet of fine time
	telemetryBuffer[9] = (uint8_t)(time >> 24); 		// C1
	telemetryBuffer[10] = (uint8_t)(time >> 16); 		// C2
	telemetryBuffer[11] = (uint8_t)(time >> 8); 		// C3
	telemetryBuffer[12] = (uint8_t)(time); 				// C4
	telemetryBuffer[13] = 0; 							// F1 // TODO: Seems to be always 0

	// Copy Source Data (variable length)
	dataPtr = (telemetryBuffer + TM_HEADERS_SIZE);
	dataEndPtr = dataPtr + sourceDataLength;
	while (dataPtr < dataEndPtr)
		*(dataPtr++) = *(sourceData++);

	// Packet Error Control (16 bits)
	chk = 0xFFFF; 	// Init syndrome

	unsigned int LTbl[256];
	InitLtbl(LTbl);

	// Compute CRC (all packet data except FCS field)
	for (int i = 0; i < packetLength - 2; i++)
		chk = crc_opt(telemetryBuffer[i], chk, LTbl);

	// Fill CRC field
	telemetryBuffer[packetLength - 2] = (uint8_t)(chk >> 8);
	telemetryBuffer[packetLength - 1] = (uint8_t)(chk);

	// Return number of bytes written
	*telemetryBufferSize = packetLength;

	return ERR_SUCCESS;
}

// Generates a complete telemetry packet with the values and data specified
uint8_t CCSDS_GenerateTelemetryPacket(uint8_t* telemetryBuffer,
                                      uint8_t* telemetryBufferSize, uint16_t apid, uint8_t serviceType,
                                      uint8_t serviceSubtype, uint8_t* sourceData, uint8_t sourceDataLength) {

	uint32_t frame_time;
	timestamp_t t;

	if (serviceType == 3 && serviceSubtype == 25 && sourceData[0] == 0xFF) {
		memcpy(&frame_time, &sourceData[1], 4);
	}
	else {
		t.tv_sec = 0;
		t.tv_nsec = 0;
		obc_timesync(&t, 6000);
		frame_time = t.tv_sec - 946684800;
	}
	return CCSDS_GenerateTelemetryPacketWithTime(telemetryBuffer,
	        telemetryBufferSize, apid, serviceType, serviceSubtype, sourceData,
	        sourceDataLength, frame_time);
}

// packet with AX.25 Secondary header and Send to COM
uint8_t AX25_GenerateTelemetryPacket_Send(uint8_t* data , uint8_t data_len) {

	uint8_t txBuffer[255];
	uint8_t txlen;
	uint8_t rx;
	if (data_len > i2c_max_size - 6)
		return ERR_SIZE_ERROR;

	memcpy(&txBuffer[5], data, data_len);
	txBuffer[0] = com_tx_send;
	txBuffer[1] = 0; 						// Frame Identification(8bits)
	txBuffer[2] = AX25_Sequence_Count(); 	// Master Frame Count
	txBuffer[3] = txBuffer[2];				// Virtual Channel Frame Count
	txBuffer[4] = 0x00; 					// First Header Point : 0xFE = no packet fragment inside

	txBuffer[data_len + AX25_2ed_size] = TC_Count();
	txlen = data_len + AX25_2ed_size + 1;
	if (parameters.shutdown_flag != 1) {
		if (i2c_master_transaction_2(0, com_tx_node, &txBuffer, txlen, &rx, 1, com_delay) != E_NO_ERR)
			return Error;
	}
	else
		printf("transmitter shutdown!!");

	return ERR_SUCCESS;
}


uint8_t AX25_Send(uint8_t* data , uint8_t data_len) {

	uint8_t txBuffer[255];
	uint8_t txlen;
	uint8_t rx;
	if (data_len > i2c_max_size - 1)
		return ERR_SIZE_ERROR;

	memcpy(&txBuffer[1], data, data_len);
	txBuffer[0] = com_tx_send;
	txlen = data_len + 1;
	if (parameters.shutdown_flag != 1) {
		if (i2c_master_transaction_2(0, com_tx_node, &txBuffer, txlen, &rx, 1, com_delay) != E_NO_ERR)
			return Error;
	}

	return ERR_SUCCESS;
}

/*-----------------------------------------------
 * sendTelecommandReport_Success()
 * -----------------------------------------------
 * Input :
 * 	*telecommand : buffer containing the concerned telecommand packet
 *  reportType : contains the success type (received, completed, progress, etc)
 *
 * Output :
 *
 *	error : No_Error when no error has occured, and error ID otherwise
 *
 * Description:
 * 	This function is called by decodeService8() every time a success message
 *  needs to be downlinked to the ground station in a telemetry packet, after
 *  a successful step in the execution of a service 8 function.
 *
 * -----------------------------------------------*/
uint8_t sendTelecommandReport_Success(uint8_t * telecommand, uint8_t reportType) {

	uint8_t err;
	// CCSDS Source Data
	uint8_t success[TM_S1_SUCCESS_SIZE];
	// CCSDS Packet length
	uint8_t packetLength = TM_NONDATA_SIZE + TM_S1_SUCCESS_SIZE;

	uint8_t temporaryBuffer[packetLength];

	// Packet Sequence Control (direct copy from telecommand)
	success[0] = telecommand[0];
	success[1] = telecommand[1];

	// Telecommand Packet ID (direct copy from telecommand)
	success[2] = telecommand[2];
	success[3] = telecommand[3];

	// Generate CCSDS telemetry packet
	err = CCSDS_GenerateTelemetryPacket(&temporaryBuffer[0], &packetLength,
	                                    obc_apid, CCSDS_T1_TELECOMMAND_VERIFICATION, reportType, success,
	                                    TM_S1_SUCCESS_SIZE);

	if (err == ERR_SUCCESS) {
		return AX25_GenerateTelemetryPacket_Send(&temporaryBuffer[0] , packetLength);
	}
	return err;
}
uint8_t sendTelecommandReport_Success_INMS(uint8_t * telecommand, uint8_t reportType) {

	uint8_t err;
	// CCSDS Source Data
	uint8_t success[TM_S1_SUCCESS_SIZE];
	// CCSDS Packet length
	uint8_t packetLength = TM_NONDATA_SIZE + TM_S1_SUCCESS_SIZE;

	uint8_t temporaryBuffer[packetLength];

	// Packet Sequence Control (direct copy from telecommand)
	success[0] = telecommand[0];
	success[1] = telecommand[1];

	// Telecommand Packet ID (direct copy from telecommand)
	success[2] = telecommand[2];
	success[3] = telecommand[3];

	// Generate CCSDS telemetry packet
	err = CCSDS_GenerateTelemetryPacket(&temporaryBuffer[0], &packetLength,
	                                    inms_apid, CCSDS_T1_TELECOMMAND_VERIFICATION, reportType, success,
	                                    TM_S1_SUCCESS_SIZE);

	if (err == ERR_SUCCESS) {
		return AX25_GenerateTelemetryPacket_Send(&temporaryBuffer[0] , packetLength);
	}
	return err;
}
/*-----------------------------------------------
 * sendTelecommandReport_Failure()
 * -----------------------------------------------
 * Input :
 * 	*telecommand : buffer containing the concerned telecommand packet
 * 	reportType : contains the success type (received, completed, progress, etc)
 *  err : describes the nature of the failure
 *
 * Output :
 *	none
 *
 * Description:
 * 	This function is called by decodeService8() every time a failure message
 *  needs to be downlinked to the ground station in a telemetry packet, after
 *  an unsuccesful step in the execution of a service 8 function.
 *
 * -----------------------------------------------*/

// Sends a telecommand failure report
// Report type can be any failure subtype of service type 1
// Returns encountered errors
uint8_t sendTelecommandReport_Failure(uint8_t* telecommand, uint8_t reportType, uint8_t err)
{
	// CCSDS Source Data
	uint8_t failure[TM_S1_FAILURE_SIZE];
	// CCSDS Packet length
	uint8_t packetLength = TM_NONDATA_SIZE + TM_S1_FAILURE_SIZE;

	uint8_t temporaryBuffer[packetLength];

	// Packet Sequence Control (direct copy from telecommand)
	failure[0] = telecommand[0];
	failure[1] = telecommand[1];

	// Telecommand Packet ID (direct copy from telecommand)
	failure[2] = telecommand[2];
	failure[3] = telecommand[3];

	// Error Code
	failure[4] = 0x00;
	failure[5] = err;

	// Generate CCSDS telemetry packet
	err = CCSDS_GenerateTelemetryPacket(&temporaryBuffer[0], &packetLength,
	                                    obc_apid, CCSDS_T1_TELECOMMAND_VERIFICATION, reportType, failure,
	                                    TM_S1_FAILURE_SIZE);

	if (err == ERR_SUCCESS)
		return AX25_GenerateTelemetryPacket_Send(&temporaryBuffer[0], packetLength);


	return err;
}
uint8_t sendTelecommandReport_Failure_INMS(uint8_t* telecommand, uint8_t reportType, uint8_t err)
{
	// CCSDS Source Data
	uint8_t failure[TM_S1_FAILURE_SIZE];
	// CCSDS Packet length
	uint8_t packetLength = TM_NONDATA_SIZE + TM_S1_FAILURE_SIZE;

	uint8_t temporaryBuffer[packetLength];

	// Packet Sequence Control (direct copy from telecommand)
	failure[0] = telecommand[0];
	failure[1] = telecommand[1];

	// Telecommand Packet ID (direct copy from telecommand)
	failure[2] = telecommand[2];
	failure[3] = telecommand[3];

	// Error Code
	failure[4] = 0x00;
	failure[5] = err;

	// Generate CCSDS telemetry packet
	err = CCSDS_GenerateTelemetryPacket(&temporaryBuffer[0], &packetLength,
	                                    inms_apid, CCSDS_T1_TELECOMMAND_VERIFICATION, reportType, failure,
	                                    TM_S1_FAILURE_SIZE);

	if (err == ERR_SUCCESS)
		return AX25_GenerateTelemetryPacket_Send(&temporaryBuffer[0], packetLength);


	return err;
}


uint8_t SendPacketWithCCSDS_AX25(void * hkBuffer, uint8_t hkBufferLength, uint8_t apid, uint8_t type, uint8_t subTypes) {
	uint8_t err;
	uint8_t packetLength = 255;

	uint8_t messageBuffer[255];


	err = CCSDS_GenerateTelemetryPacket(&messageBuffer[0], &packetLength, apid, type, subTypes, hkBuffer, hkBufferLength);
	if (err == ERR_SUCCESS) {
		return AX25_GenerateTelemetryPacket_Send(&messageBuffer[0], packetLength);
	}
	return CCSDS_PACKET_ERROR;
}  /* end of SendPacketWithCCSDS_AX25 */


uint8_t SendDataWithCCSDS_AX25(uint8_t datatype, uint8_t* data) { //add sid then packet it with ccsds
	/*   data type:
	 *   1 = HK
	 *   2 = inms
	 *   3 = seuv
	 *   4 = eop
	 *   5 = wod          */
	uint8_t datalength;
	uint8_t databuffer[250];
	uint8_t txframe[246];
	uint8_t txframe_time[246];
	uint8_t tx_length = 245;
	uint8_t err;
	uint16_t chk;
	unsigned int LTbl[256];

	if (datatype == 1) {
		datalength = hk_length;
		memcpy(&databuffer[0], &data[0], datalength);
		err = CCSDS_GenerateTelemetryPacket(&txframe[0], &tx_length, obc_apid, 125, 1, &databuffer[0], datalength);
		if (err == ERR_SUCCESS) {
			AX25_GenerateTelemetryPacket_Send(&txframe[0], tx_length);
			return No_Error;
		}
		else
			return Error;
	}
	else if (datatype == 2) {
		datalength = inms_data_length;
		memcpy(&databuffer[0], &data[0], datalength);
		err = CCSDS_GenerateTelemetryPacket(&txframe[0], &tx_length, obc_apid, 128, 1, &databuffer[0], datalength);
		if (err == ERR_SUCCESS) {
			AX25_GenerateTelemetryPacket_Send(&txframe[0], tx_length);
			return No_Error;
		}
		else
			return Error;
	}
	else if (datatype == 3) {
		datalength = seuv_length;
		memcpy(&databuffer[0], &data[0], datalength);
		err = CCSDS_GenerateTelemetryPacket(&txframe[0], &tx_length, obc_apid, 127, 1, &databuffer[0], datalength);
		if (err == ERR_SUCCESS) {
			AX25_GenerateTelemetryPacket_Send(&txframe[0], tx_length);
			return No_Error;
		}
		else
			return Error;
	}
	else if (datatype == 4) {
		datalength = eop_length;
		memcpy(&databuffer[0], &data[0], datalength);
		err = CCSDS_GenerateTelemetryPacket(&txframe[0], &tx_length, obc_apid, 126, 1, &databuffer[0], datalength);
		if (err == ERR_SUCCESS) {
			AX25_GenerateTelemetryPacket_Send(&txframe[0], tx_length);
			return No_Error;
		}
		else
			return Error;
	}
	else if (datatype == 5) {

		databuffer[0] = wod_sid;

		memcpy(&databuffer[1], &data[0], 4 + 8 * 24);
		err = CCSDS_GenerateTelemetryPacket(&txframe_time[0], &tx_length, obc_apid, 3, 25, &databuffer[0], 4 + 8 * 24 + 1);

		memcpy(&txframe[0], &txframe_time[0], 15);
		memcpy(&txframe[15], &txframe_time[19], tx_length - 15 - 4);

		txframe[5] -= 4;
		// Packet Error Control (16 bits)
		chk = 0xFFFF;
		InitLtbl(LTbl);

		// Compute CRC (all packet data except FCS field)
		for (int i = 0; i < tx_length - 6; i++)
			chk = crc_opt(txframe[i], chk, LTbl);

		// Fill CRC field
		txframe[tx_length - 6] = (uint8_t)(chk >> 8);
		txframe[tx_length - 5] = (uint8_t)(chk);

		if (err == ERR_SUCCESS) {
			AX25_GenerateTelemetryPacket_Send(&txframe[0], tx_length - 4);
			return No_Error;
		}
		else
			return Error;
	}
	else
		return Error;

	memcpy(&databuffer[1], data, datalength - 1);

	err = CCSDS_GenerateTelemetryPacket(&txframe[0], &tx_length, obc_apid, 15, 9, &databuffer[0], datalength);

	if (err == ERR_SUCCESS) {
		return AX25_GenerateTelemetryPacket_Send(&txframe[0], tx_length);
	}
	else
		printf("have error on generate CCSDS packet, maybe overflow  , tx_length = %d \n", tx_length);

	return Error;
}

void decodeCCSDS_Command(uint8_t * telecommand, uint8_t packet_length) {
	decode_command = 1;
	uint8_t serviceType =  telecommand[7];
	uint8_t serviceSubType = telecommand[8];
	uint16_t chk;
	unsigned int LTbl[256];
	timestamp_t t;

	// Compute CRC (all packet data except FCS field)
	chk = 0xFFFF;

	InitLtbl(LTbl);
	for (int i = 0; i < packet_length - 2; i++)
		chk = crc_opt(telecommand[i], chk, LTbl);

	/* Check CRC field */
	if (telecommand[packet_length - 2] == (uint8_t)(chk >> 8) && telecommand[packet_length - 1] == (uint8_t)(chk) ) {
		printf("Telecommand Pass CRC Check\n");

		switch (serviceType) {

		case T3_SYS_CONF:
			decodeService3(serviceSubType, telecommand);
			break;
		case T8_function_management:
			decodeService8(serviceSubType, telecommand);
			break;
		case T11_OnBoard_Schedule:
			decodeService11(serviceSubType, telecommand);
			break;
		case T13_LargeData_Transfer:
			decodeService13(serviceSubType, telecommand);
			break;
		case T15_dowlink_management:
			decodeService15(serviceSubType, telecommand);
			break;
		case T31_photo_processing:
			decodeService31(serviceSubType, telecommand);
			break;
		case T32_image_upload:
			decodeService32(serviceSubType, telecommand);
			break;
		case T131_ADCS:
			decodeService131(serviceSubType, telecommand);
			break;
		case T132_SEUV:
			decodeService132(serviceSubType, telecommand);
			break;
		default:
			sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
			break;
		}
		t.tv_sec = 0;
		t.tv_nsec = 0;
		obc_timesync(&t, 6000);
		lastCommandTime = t.tv_sec;
	}
	else {
		printf("CRC Check not pass!!!\n");
	}
	decode_command = 0;
}  /* end of decodeCCSDS_Command */
