#include <util/timestamp.h>
#include "subsystem.h"
#include <freertos/FreeRTOS.h>
#include "parameter.h"
#include "crc16.h"
#include <dev/i2c.h>
#include "Tele_function.h"
#include <util/hexdump.h>
#include "fs.h"
#include <io/nanomind.h>
#include <string.h>


void set_Call_Sign(int SSID){


    uint8_t txdata[8];
    txdata[0] = 0x22; //set TO Call-Sign
    txdata[1] = 78; //NCKUGS
    txdata[2] = 67;
    txdata[3] = 75;
    txdata[4] = 85;
    txdata[5] = 71;
    txdata[6] = 83;
    txdata[7] = (uint8_t)SSID;

i2c_master_transaction(0,com_tx_node,&txdata,8,0,0,0);

            txdata[0] = 0x23; //set FROM Call-Sign
            txdata[1] = 84; //TW01TN
            txdata[2] = 87;
            txdata[3] = 48;
            txdata[4] = 49;
            txdata[5] = 84;
            txdata[6] = 78;
            txdata[7] = (uint8_t)SSID;

 i2c_master_transaction(0,com_tx_node,&txdata,8,0,0,0);


}

// Generates sequenceCount , reset if overflow
uint8_t TC_Count(){

	if (parameters.tc_count == 4)
	parameters.tc_count = 0;

	parameters.tc_count = parameters.tc_count + 1;
			return parameters.tc_count-1;
}


// Generates sequenceCount , reset if overflow
uint8_t AX25_Sequence_Count(){
	if (parameters.ax25_sequence_count == 255) {
				parameters.ax25_sequence_count = 0;
				return parameters.ax25_sequence_count;
			}
			parameters.ax25_sequence_count =
					parameters.ax25_sequence_count + 1;
			return parameters.ax25_sequence_count;
}



// Generates sequenceCount , reset if overflow
uint16_t CCSDS_GetSequenceCount(uint16_t apid) {

	if (apid == obc_apid) {   //obc related command
		if (parameters.obc_packet_sequence_count == 16383) {
			parameters.obc_packet_sequence_count = 0;
			return parameters.obc_packet_sequence_count;
		}
		parameters.obc_packet_sequence_count =
				parameters.obc_packet_sequence_count + 1;
		return parameters.obc_packet_sequence_count;
	}

	else if (apid == inms_apid) {   //obc related command
		if (parameters.inms_packet_sequence_count == 16383) {
			parameters.inms_packet_sequence_count = 0;
			return parameters.inms_packet_sequence_count;
		}
		parameters.inms_packet_sequence_count =
				parameters.inms_packet_sequence_count + 1;
		return parameters.inms_packet_sequence_count;
	}

	else if (apid == seuv_apid) {   //obc related command
		if (parameters.seuv_packet_sequence_count == 16383) {
			parameters.seuv_packet_sequence_count = 0;
			return parameters.seuv_packet_sequence_count;
		}
		parameters.seuv_packet_sequence_count =
				parameters.seuv_packet_sequence_count + 1;
		return parameters.seuv_packet_sequence_count;
	}

	else if (apid == wod_apid) {   //obc related command
		if (parameters.wod_packet_sequence_count == 16383) {
			parameters.wod_packet_sequence_count = 0;
			return parameters.wod_packet_sequence_count;
		}
		parameters.wod_packet_sequence_count =
				parameters.wod_packet_sequence_count + 1;
		return parameters.wod_packet_sequence_count;
	}

	else if (apid == phoenix_hk_apid) {   //obc related command
		if (parameters.phoenix_hk_packet_sequence_count == 16383) {
			parameters.phoenix_hk_packet_sequence_count = 0;
			return parameters.phoenix_hk_packet_sequence_count;
		}
		parameters.phoenix_hk_packet_sequence_count =
				parameters.phoenix_hk_packet_sequence_count + 1;
		return parameters.phoenix_hk_packet_sequence_count;
	}
	else
		return 255;
}



// Generates a complete telemetry packet with the values and data specified
uint8_t CCSDS_GenerateTelemetryPacketWithTime(uint8_t* telemetryBuffer,
		uint8_t* telemetryBufferSize, uint16_t apid, uint8_t serviceType,
		uint8_t serviceSubtype, uint8_t* sourceData, uint8_t sourceDataLength,
		uint32_t time) {
	uint16_t chk;   // CRC syndrome
	uint16_t sequenceCount;   // Sequence Count of packet
	uint8_t packetLength;   // Total length of packet
	uint16_t packetLengthFieldValue;  // Value of Packet Length header field
	uint8_t*dataPtr;  // For source data copy
	uint8_t*dataEndPtr;

	if (sourceData == NULL)
		return ERR_MISSING_PARAMETER; // No data
	if (telemetryBuffer == NULL)
		return ERR_MISSING_PARAMETER; // No telemetry buffer
	if (telemetryBufferSize == NULL)
		return ERR_MISSING_PARAMETER; // No telemetry buffer size

// Total packet length must be <= than 235 octets (otherwise it cannot be transmitted in COM board)
	if (sourceDataLength > (235 - TM_NONDATA_SIZE - AX25_2ed_size -1))
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
	telemetryBuffer[4] = (uint8_t)(packetLengthFieldValue >> 8); // Packet Length (16 bits) -> 8 MSB
	telemetryBuffer[5] = (uint8_t)(packetLengthFieldValue); // -> 8 LSB

// Packet Data Field (Telemetry Data Field Header, Source Data and Packet Error Control) (Variable)
// Telemetry Data Field Header (56 bits)
	telemetryBuffer[6] = 0x10; // Spare (1 bit) = 0, PUS Version Number (3 bits) = 1, Spare (4 bits) = 0
	telemetryBuffer[7] = serviceType; // Service Type (8 bits)
	telemetryBuffer[8] = serviceSubtype; // Service Subtype (8 bits)

// Absolute Time (40 bits, 5 octets)
// CUC with 4 octets of coarse time and 1 octet of fine time
	telemetryBuffer[9] = (uint8_t)(time >> 24); // C1
	telemetryBuffer[10] = (uint8_t)(time >> 16); // C2
	telemetryBuffer[11] = (uint8_t)(time >> 8); // C3
	telemetryBuffer[12] = (uint8_t)(time); // C4
	telemetryBuffer[13] = 0; // F1 // TODO: Seems to be always 0

// Copy Source Data (variable length)
	dataPtr = (telemetryBuffer + TM_HEADERS_SIZE);
	dataEndPtr = dataPtr + sourceDataLength;
	while (dataPtr < dataEndPtr)
		*(dataPtr++) = *(sourceData++);

// Packet Error Control (16 bits)
	chk = 0xFFFF; // Init syndrome
// Compute CRC (all packet data except FCS field)

	chk = crc16_ccitt(telemetryBuffer, packetLength - 2);

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
	uint32_t time;
	timestamp_t t;
	t.tv_sec = 0;
	t.tv_nsec = 0;
	obc_timesync(&t, 6000);
	time = t.tv_sec;
    hex_dump(&time,4);
	return CCSDS_GenerateTelemetryPacketWithTime(telemetryBuffer,
			telemetryBufferSize, apid, serviceType, serviceSubtype, sourceData,
			sourceDataLength, time);
}


//packet with AX.25 Secondary header and Send to COM
uint8_t AX25_GenerateTelemetryPacket_Send(uint8_t * data , uint8_t data_len){

	uint8_t txBuffer[255];
	uint8_t txlen;

	if(data_len>i2c_max_size-6)
		return ERR_SIZE_ERROR;

	memcpy(&txBuffer[5],data,data_len);
    txBuffer[0]=com_tx_send;
    txBuffer[1]=0;//Frame Identification(8bits)
    txBuffer[2]= AX25_Sequence_Count(); //Master Frame Count
    txBuffer[3]= txBuffer[2];           // Virtual Channel Frame Count
    txBuffer[4]= 0xFE; //First Header Point : 0xFE = no packet fragment inside

    txBuffer[data_len+AX25_2ed_size]=TC_Count(); //tc_count
    txlen= data_len + AX25_2ed_size + 1;
    i2c_master_transaction(0, com_tx_node, &txBuffer,txlen, 0, 0, 2);




    return ERR_SUCCESS;
}


/*-----------------------------------------------
 * sendTelecommandReport_Success()
 * -----------------------------------------------
 * Input
 :
 * *telecommand : buffer containing the concerned telecommand packet
 * reportType : contains the success type (received, completed, progress, etc)
 *
 * Output :
 *
 error : ERR_SUCCESS when no error has occured, and error ID otherwise
 *
 * Description:
 * This function is called by decodeService8() every time a success message
 * needs to be downlinked to the ground station in a telemetry packet, after
 * a successful step in the execution of a service 8 function.
 *
 * -----------------------------------------------*/
uint8_t sendTelecommandReport_Success(uint8_t* telecommand, uint8_t reportType) {

	uint8_t err;
// CCSDS Source Data

	uint8_t success[TM_S1_SUCCESS_SIZE];
// CCSDS Packet length

	uint8_t packetLength = TM_NONDATA_SIZE + TM_S1_SUCCESS_SIZE;
	uint8_t temporaryBuffer[packetLength];

	success[0] = telecommand[0];
	success[1] = telecommand[1];
// Telecommand Packet ID (direct copy from telecommand)
	success[2] = telecommand[2];
	success[3] = telecommand[3];
// Packet Sequence Control (direct copy from telecommand)

// Generate CCSDS telemetry packet
	err = CCSDS_GenerateTelemetryPacket(&temporaryBuffer[0], &packetLength,
			obc_apid, CCSDS_T1_TELECOMMAND_VERIFICATION, reportType, success,
			TM_S1_SUCCESS_SIZE);

	if (err == ERR_SUCCESS) {
		   return AX25_GenerateTelemetryPacket_Send(&temporaryBuffer[0] ,packetLength);
	}
	return err;
}

/*-----------------------------------------------
 * sendTelecommandReport_Failure()
 * -----------------------------------------------
 * Input
 :
 * *telecommand : buffer containing the concerned telecommand packet
 * reportType : contains the success type (received, completed, progress, etc)
 * err : describes the nature of the failure
 *
 * Output :
 *
 none
 *
 * Description:
 * This function is called by decodeService8() every time a failure message
 * needs to be downlinked to the ground station in a telemetry packet, after
 * an unsuccesful step in the execution of a service 8 function.
 *
 * -----------------------------------------------*/
// Sends a telecommand failure report
// Report type can be any failure subtype of service type 1
// Returns encountered errors
uint8_t sendTelecommandReport_Failure(uint8_t* telecommand, uint8_t reportType,
		uint8_t err) {
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
		   return AX25_GenerateTelemetryPacket_Send(&temporaryBuffer[0],packetLength);


	return err;
}


/* Use This Function to Send System Message to Ground */
uint8_t SendPacketWithCCSDS_AX25(void * hkBuffer,uint8_t hkBufferLength,uint8_t sid,uint8_t apid,uint8_t type,uint8_t subTypes){
	uint8_t err;
	uint8_t packetLength=255;
	uint8_t messageBuffer[255];


    err = CCSDS_GenerateTelemetryPacket(&messageBuffer[0],&packetLength,apid,type,subTypes,hkBuffer,hkBufferLength);
    if(err == ERR_SUCCESS){
    	 return AX25_GenerateTelemetryPacket_Send(&messageBuffer[0],packetLength);
    }
    return CCSDS_PACKET_ERROR;
}  /* end of SendPacketWithCCSDS_AX25 */

/* Use This Function to Downlink On-Board Data */
uint8_t SendDataWithCCSDS_AX25(uint8_t datatype,uint8_t* data){  //add sid then packet it with ccsds
	/*   data type:
	 *   1 = HK
	 *   2 = inms
	 *   3 = seuv
	 *   5 = wod          */
	uint8_t datalength;
	uint8_t databuffer[250];
	uint8_t txframe[246];
    uint8_t tx_length=245;
    uint8_t err;

    if(datatype==1){
    	    databuffer[0]=phoenix_hk_sid;
    	     datalength=hk_length+1;

    	}else if(datatype==2){
    		databuffer[0]=inms_sid;
    		 datalength=inms_data_length+1;

    	}else if(datatype==3){
    		databuffer[0]=seuv_sid;
    		 datalength=seuv_length+1;

    	}else if(datatype==5){
    		databuffer[0]=wod_sid;
    	 	 datalength=wod_length+1;

    	}else
    		return Error;

        memcpy(&databuffer[1],data,datalength-1);//copy data to databuffer

    err= CCSDS_GenerateTelemetryPacket(&txframe[1],&tx_length,obc_apid, 15,9,databuffer,datalength);

    if(err==ERR_SUCCESS){
   return AX25_GenerateTelemetryPacket_Send(&txframe[0] ,tx_length);

    }else
	printf("have error on generate CCSDS packet, maybe overflow  , tx_length = %d \n",tx_length);

    return Error;
}



void decodeService129(uint8_t subType, uint8_t * telecommand){
uint8_t txBuffer[254];
uint8_t rxBuffer[254];
uint8_t completionError = ERR_SUCCESS;
uint8_t err;
uint8_t rxBufferLength;
uint8_t types=3;
uint8_t parameterLength;
// note: parameter start from telecommand[9]
/*------------------------------------------Telecommand-----------------------------------*/

/*---------------ID:1 Reset----------------*/
if(subType == Reset ){  //you should modify this ID
sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
txBuffer[0]=subType;
parameterLength = 1;  // you should modify the size of parameter part
memcpy(&txBuffer[1],telecommand+9,parameterLength);     // copy command & parameter to I2C tx buffer
if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)  //send I2C
sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
else{                                                                  //send COMPLETE_FAIL report if send fail
	completionError= I2C_SEND_ERROR;
	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
}}

/*--------------ID:2 Set_Unix_Time-----------------*/
else if(subType == Set_Unix_Time ){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 6;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*--------------ID:3 ADCS_Run_Mode----------------*/
else if(subType == ADCS_Run_Mode){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 1;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*--------------ID:4 Select_Logged_Data----------------*/
else if(subType == Selected_Logged_Data){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 13;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*--------------ID:5 Power_Control----------------*/
else if(subType == Power_Control){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 5;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*--------------ID:6 Deploy_Magnetometer_Boom----------------*/
else if(subType == Deploy_Magnetometer_Boom){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 1;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*--------------ID:7 Trigger_ADCS_Loop----------------*/
else if(subType == Trigger_ADCS_Loop){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 0;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*--------------ID:17 Set_Attitude_Estimation_Mode----------------*/
else if(subType == Set_Attitude_Estimation_Mode){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 1;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*--------------ID:18 Set_Attitude_Control_Mode----------------*/
else if(subType == Set_Attitude_Control_Mode){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 4;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*--------------ID:19 Set_Commanded_Attitude_Angles----------------*/
else if(subType == Set_Commanded_Attitude_Angles){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 6;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*--------------ID:32 Set_Wheel_Speed----------------*/
else if(subType == Set_Wheel_Speed){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 6;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*--------------ID:33 Set_Magnetorquer_Output----------------*/
else if(subType == Set_Magnetorquer_Output){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 6;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*--------------ID:102 Set_Startup_Mode----------------*/
else if(subType == Set_Startup_Mode){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 1;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*--------------ID:64 Set_SGP4_Orbit_Parameters----------------*/
else if(subType == Set_SGP4_Orbit_Parameters){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 64;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*--------------ID:80 Set_Configuration----------------*/
else if(subType == Set_Configuration){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 236;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*--------------ID:81 Set_Magnetorquer_Configuration----------------*/
else if(subType == Set_Magnetorquer_Configuration){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 13;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*--------------ID:82 Set_Wheel_Configuration----------------*/
else if(subType == Set_Wheel_Configuration){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 13;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*--------------ID:83 Set_CSS_Configuration----------------*/
else if(subType == Set_CSS_Configuration){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 14;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*--------------ID:84 Set_Sun_Sensor_Configuration----------------*/
else if(subType == Set_Sun_Sensor_Configuration){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 17;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*--------------ID:85 Set_Nadir_Sensor_Configuration----------------*/
else if(subType == Set_Nadir_Sensor_Configuration){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 57;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*--------------ID:86 Set_Magnetometer_Configuration----------------*/
else if(subType == Set_Magnetometer_Configuration){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 30;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*--------------ID:87 Set_Rate_Sensor_Configuration----------------*/
else if(subType == Set_Rate_Sensor_Configuration){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 6;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*--------------ID:88 Set_Detumbling_Control_Parameters----------------*/
else if(subType == Set_Detumbling_Control_Parameters){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 10;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*--------------ID:89 Set_Ymomentum_Control_Parameters----------------*/
else if(subType == Set_Ymomentum_Control_Parameters){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 26;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*--------------ID:90 Set_Moment_Of_Inertia----------------*/
else if(subType == Set_Moment_Of_Inertia){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 24;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*--------------ID:91 Set_Estimation_Parameters----------------*/
else if(subType == Set_Estimation_Parameters){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 26;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*--------------ID:100 Save_Configuration----------------*/
else if(subType == Save_Configuration){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 0;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*--------------ID:101 Save_Orbit_Parameters----------------*/
else if(subType == Save_Orbit_Parameters){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 0;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*--------------ID:110 Capture_And_Save_Image----------------*/
else if(subType == Capture_And_Save_Image){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 10;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*--------------ID:114 Reset_File_List----------------*/
else if(subType == Reset_File_List){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 0;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*--------------ID:115 Advance_File_List_Index----------------*/
else if(subType == Advance_File_List_Index){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 0;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*--------------ID:116 Initialize_File_Download----------------*/
else if(subType == Initialize_File_Download){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 13;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*--------------ID:117 Advance_File_Read_Pointer----------------*/
else if(subType == Advance_File_Read_Pointer){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 2;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*--------------ID:118 Erase_File----------------*/
else if(subType == Erase_File){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 13;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*--------------ID:119 Erase_All_Files----------------*/
else if(subType == Erase_All_Files){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 0;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*--------------ID:120 Set_Boot_Index----------------*/
else if(subType == Set_Boot_Index){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 1;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*--------------ID:121 Erase_Program----------------*/
else if(subType == Erase_Program){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 1;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*--------------ID:122 Upload_Program_Block----------------*/
else if(subType == Upload_Program_Block){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 255;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*--------------ID:123 Finalize_Program_Upload----------------*/
else if(subType == Finalize_Program_Upload){  //you should modify this ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
	txBuffer[0]=subType;
	parameterLength = 71;  // you should modify the size of parameter part
	memcpy(&txBuffer[1],telecommand+9,parameterLength);
	if (i2c_master_transaction(0, adcs_node,&txBuffer,parameterLength+1,0,0,adcs_delay) == E_NO_ERR)
	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}


/*------------------------------------------Telemetry-----------------------------------*/
/*--------------ID:128 Identification---------------*/
else if(subType == Identification){                //you should modify this subType ID
sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
txBuffer[0]=subType;
rxBufferLength = 8;                           // you should modify rxBufferLength
if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
	err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
    if(err==ERR_SUCCESS)
    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
    else{
    	completionError = err;
    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
}
else{                                                                  //send COMPLETE_FAIL report
	completionError= I2C_SEND_ERROR;
	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
}}

/*---------------ID:130 Communication_Status--------------*/
else if(subType == Communication_Status){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 6;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:131 Telecommand_Acknowledge--------------*/
else if(subType == Telecommand_Acknowledge){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 4;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:133 Reset_Cause--------------*/
else if(subType == Reset_Cause){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 1;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:138 Actuator_Commands--------------*/
else if(subType == Actuator_Commands){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 12;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:189 ACP_Execution_State--------------*/
else if(subType == ACP_Execution_State){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 3;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:190 ACP_Execution_Times--------------*/
else if(subType == ACP_Execution_Times){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 8;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:193 EDAC_And_Latchup_Counters--------------*/
else if(subType == EDAC_And_Latchup_Counters){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 10;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:194 Startup_Mode--------------*/
else if(subType == Startup_Mode){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 1;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:240 File_Information--------------*/
else if(subType == File_Information){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 22;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:241 File_Block_CRC--------------*/
else if(subType == File_Block_CRC){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 6;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:242 File_Data_Block--------------*/
else if(subType == File_Data_Block){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 255;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:134 Power_Control_Selection--------------*/
else if(subType == Power_Control_Selection){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 5;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:135 Power_And_Temperature_Measurements--------------*/
else if(subType == Power_And_Temperature_Measurements){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 18;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:136 ADCS_State--------------*/
else if(subType == ADCS_State){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 48;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:137 ADCS_Measurements--------------*/
else if(subType == ADCS_Measurements){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 36;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:143 Current_Time--------------*/
else if(subType == Current_Time){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 6;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:144 Current_State--------------*/
else if(subType == Current_State){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 6;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:145 Estimated_Attitude_Angles--------------*/
else if(subType == Estimated_Attitude_Angles){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 6;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:146 Estimated_Angular_Rates--------------*/
else if(subType == Estimated_Angular_Rates){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 6;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:147 Satellite_Position_LLH--------------*/
else if(subType == Satellite_Position_LLH){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 6;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:148 Satellite_Velocity_ECI--------------*/
else if(subType == Satellite_Velocity_ECI){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 6;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:149 Magnetic_Field_Vector--------------*/
else if(subType == Magnetic_Field_Vector){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 6;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:150 Coarse_Sun_Vector--------------*/
else if(subType == Coarse_Sun_Vector){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 6;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:151 Fine_Sun_Vector--------------*/
else if(subType == Fine_Sun_Vector){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 6;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:152 Nadir_Vector--------------*/
else if(subType == Nadir_Vector){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 6;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:153 Rate_Sensor_Rates--------------*/
else if(subType == Rate_Sensor_Rates){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 6;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:154 Wheel_Speed--------------*/
else if(subType == Wheel_Speed){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 6;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:173 CubeSense_Current_Measurements--------------*/
else if(subType == CubeSense_Current_Measurements){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 6;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:174 CubeControl_Current_Measurements--------------*/
else if(subType == CubeControl_Current_Measurements){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 6;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:175 Peripheral_Current_And_Temperature_Measurements--------------*/
else if(subType == Peripheral_Current_And_Temperature_Measurements){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 6;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:139 Raw_Sensor_Measurements--------------*/
else if(subType == Raw_Sensor_Measurements){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 60;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:163 Angular_Rate_Covariance--------------*/
else if(subType == Angular_Rate_Covariance){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 6;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:164 Raw_Nadir_Sensor--------------*/
else if(subType == Raw_Nadir_Sensor){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 6;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:165 Raw_Sun_Sensor--------------*/
else if(subType == Raw_Sun_Sensor){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 6;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:166 Raw_CSS--------------*/
else if(subType == Raw_CSS){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 6;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:167 Raw_Magnetometer--------------*/
else if(subType == Raw_Magnetometer){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 6;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:168 Raw_GPS_Status--------------*/
else if(subType == Raw_GPS_Status){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 6;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:169 Raw_GPS_Time--------------*/
else if(subType == Raw_GPS_Time){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 6;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:170 Raw_GPS_X--------------*/
else if(subType == Raw_GPS_X){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 6;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:171 Raw_GPS_Y--------------*/
else if(subType == Raw_GPS_Y){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 6;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:172 Raw_GPS_Z--------------*/
else if(subType == Raw_GPS_Z){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 6;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:140 Estimation_Data--------------*/
else if(subType == Estimation_Data){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 42;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:157 IGRF_Modelled_Magnetic_Field_Vector--------------*/
else if(subType == IGRF_Modelled_Magnetic_Field_Vector){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 6;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:158 Modelled_Sun_Vector--------------*/
else if(subType == Modelled_Sun_Vector){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 6;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:159 Estimated_Gyro_Bias--------------*/
else if(subType == Estimated_Gyro_Bias){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 6;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:160 Estimated_Innovation_Vector--------------*/
else if(subType == Estimated_Innovation_Vector){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 6;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:161 Quaternion_Error_Vector--------------*/
else if(subType == Quaternion_Error_Vector){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 6;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:162 Quaternion_Covariance--------------*/
else if(subType == Quaternion_Covariance){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 6;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:155 Magnetorquer_Command--------------*/
else if(subType == Magnetorquer_Command){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 6;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:156 Wheel_Speed_Commands--------------*/
else if(subType == Wheel_Speed_Commands){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 6;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:191 SGP4_Orbit_Parameters--------------*/
else if(subType == SGP4_Orbit_Parameters){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 64;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:192 Configuration--------------*/
else if(subType == Configuration){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 236;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:230 Status_Of_Image_Capture_And_Save_Operation--------------*/
else if(subType == Status_Of_Image_Capture_And_Save_Operation){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 16;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:250 Uploaded_Program_Status--------------*/
else if(subType == Uploaded_Program_Status){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 3;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}

/*---------------ID:251 Get_Flash_Program_List--------------*/
else if(subType == Get_Flash_Program_List){                   //you should modify this subType ID
	sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);
	txBuffer[0]=subType;
	rxBufferLength = 255;                                // you should modify rxBufferLength
	if (i2c_master_transaction(0, adcs_node,&txBuffer,1,&rxBuffer,rxBufferLength,adcs_delay) == E_NO_ERR){
		err=SendPacketWithCCSDS_AX25(&rxBuffer,rxBufferLength,adcs_tmp_sid,adcs_apid,types,subType);
	    if(err==ERR_SUCCESS)
	    	sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);
	    else{
	    	completionError = err;
	    	sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);}
	}
	else{                                                                  //send COMPLETE_FAIL report
		completionError= I2C_SEND_ERROR;
		sendTelecommandReport_Failure(telecommand,CCSDS_S3_COMPLETE_FAIL,completionError);
	}}


}  /* end of decodeService3*/

void decodeService3(uint8_t subType, uint8_t*telecommand){
uint8_t txBuffer[254];

// note: parameter start from telecommand[9]
/*------------------------------------------Telecommand-----------------------------------*/
#define Enable_Task_Execution 5
#define Disable_Task_Execution 6
#define DTaz 7
#define DT0200 8
#define ShutdownTransmitter 9
#define ResumeTransmitter 10
#define HKStateReport 25

/*---------------ID:5 Enable_Task_Execution----------------*/
if(subType == Enable_Task_Execution ){  //you should modify this ID


if(telecommand[9]==1)
    parameters.adcs_function= 1;
else if(telecommand[9]==2)
	parameters.inms_function= 1;
else if(telecommand[9]==3)
	parameters.seuv_function= 1;
else
	sendTelecommandReport_Failure(telecommand,CCSDS_T1_ACCEPTANCE_FAIL,CCSDS_ERR_ILLEGAL_TYPE);

sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
para_w();
sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
}


/*---------------ID:6 Disable_Task_Execution----------------*/
else if(subType == Disable_Task_Execution ){  //you should modify this ID


if(telecommand[9]==1)
    parameters.adcs_function= 0;
else if(telecommand[9]==2)
	parameters.inms_function= 0;
else if(telecommand[9]==3)
	parameters.seuv_function= 0;
else
	sendTelecommandReport_Failure(telecommand,CCSDS_T1_ACCEPTANCE_FAIL,CCSDS_ERR_ILLEGAL_TYPE);


sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
para_w();
sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
}
/*---------------ID:7 DT a~z----------------*/







/*---------------ID:8 DT 0~200----------------*/
else if(subType == DT0200 ){
	for(int a =0;a<200; a++)
	txBuffer[a]=a;
	SendPacketWithCCSDS_AX25(&txBuffer,200,0,obc_apid,3,subType);
}

/*---------------ID:9 ShutdownTransmitter----------------*/

else if(subType == ShutdownTransmitter ){  //you should modify this ID
sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
parameters.shutdown_flag= 1;
para_w();
sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
}

/*---------------ID:10 ResumeTransmitter----------------*/
else if(subType == ResumeTransmitter ){  //you should modify this ID
sendTelecommandReport_Success(telecommand, CCSDS_S3_ACCEPTANCE_SUCCESS);  //send acceptance report
parameters.shutdown_flag= 0;
para_w();
sendTelecommandReport_Success(telecommand, CCSDS_S3_COMPLETE_SUCCESS);  //send COMPLETE_success report
}

/*---------------ID:25 HK State Report----------------*/
}


void decodeCCSDS_Command(uint8_t * telecommand){

uint8_t serviceType =  telecommand[7];
uint8_t serviceSubType = telecommand[8];

switch(serviceType){
case T3_SYS_CONF:
	decodeService3(serviceSubType,telecommand);
	break;

case T129_ADCS:
	decodeService129(serviceSubType,telecommand);
	break;
default:
	sendTelecommandReport_Failure(telecommand,CCSDS_T1_ACCEPTANCE_FAIL,CCSDS_ERR_ILLEGAL_TYPE);

	break;
 }
}  /* end of decodeCCSDS_Command */


