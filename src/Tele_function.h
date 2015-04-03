/*
 * function.h
 *
 *  Created on: 2015/1/29
 *      Author: rusei
 */

#ifndef FUNCTION_H_
#define FUNCTION_H_

uint8_t CCSDS_GenerateTelemetryPacket(uint8_t* telemetryBuffer,uint8_t* telemetryBufferSize, uint16_t apid, uint8_t serviceType,uint8_t serviceSubtype, uint8_t* sourceData, uint8_t sourceDataLength);
uint8_t sendTelecommandReport_Success(uint8_t* telecommand, uint8_t reportType);
uint8_t sendTelecommandReport_Failure(uint8_t* telecommand, uint8_t reportType,uint8_t err);
void decodeCCSDS_Command(uint8_t * telecommand);

uint8_t SendPacketWithCCSDS_AX25(void * hkBuffer,uint8_t hkBufferLength,uint8_t sid,uint8_t apid,uint8_t type,uint8_t subTypes);
uint8_t SendDataWithCCSDS_AX25(uint8_t datatype,uint8_t* data);

void set_Call_Sign(int SSID);

/*Type 3 System Configuration Telecommand Table*/
#define Enable_Task_Execution 5
#define Disable_Task_Execution 6

#define DT0200 8
#define ShutdownTransmitter 9
#define ResumeTransmitter 10
#define HKStateReport 25

/*Type 129 ADCS Telecommand Table*/
#define Reset 1
#define Set_Unix_Time 2
#define ADCS_Run_Mode 3
#define Selected_Logged_Data 4
#define Power_Control 5
#define Deploy_Magnetometer_Boom 6
#define Trigger_ADCS_Loop 7
#define Set_Attitude_Estimation_Mode 17
#define Set_Attitude_Control_Mode 18
#define Set_Commanded_Attitude_Angles 19
#define Set_Wheel_Speed 32
#define Set_Magnetorquer_Output 33
#define Set_Startup_Mode 102
#define Set_SGP4_Orbit_Parameters 64
#define Set_Configuration 80
#define Set_Magnetorquer_Configuration 81
#define Set_Wheel_Configuration 82
#define Set_CSS_Configuration 83
#define Set_Sun_Sensor_Configuration 84
#define Set_Nadir_Sensor_Configuration 85
#define Set_Magnetometer_Configuration 86
#define Set_Rate_Sensor_Configuration 87
#define Set_Detumbling_Control_Parameters 88
#define Set_Ymomentum_Control_Parameters 89
#define Set_Moment_Of_Inertia 90
#define Set_Estimation_Parameters 91
#define Save_Configuration 100
#define Save_Orbit_Parameters 101
#define Capture_And_Save_Image 110
#define Reset_File_List 114
#define Advance_File_List_Index 115
#define Initialize_File_Download 116
#define Advance_File_Read_Pointer 117
#define Erase_File 118
#define Erase_All_Files 119
#define Set_Boot_Index 120
#define Erase_Program 121
#define Upload_Program_Block 122
#define Finalize_Program_Upload 123


/*ADCS Telemetry Table*/
#define Identification 128
#define Communication_Status 130
#define Telecommand_Acknowledge 131
#define Reset_Cause 133
#define Actuator_Commands 138
#define ACP_Execution_State 189
#define ACP_Execution_Times 190
#define EDAC_And_Latchup_Counters 193
#define Startup_Mode 194
#define File_Information 240
#define File_Block_CRC 241
#define File_Data_Block 242
#define Power_Control_Selection 134
#define Power_And_Temperature_Measurements 135
#define ADCS_State 136
#define ADCS_Measurements 137
#define Current_Time 143
#define Current_State 144
#define Estimated_Attitude_Angles 145
#define Estimated_Angular_Rates 146
#define Satellite_Position_LLH 147
#define Satellite_Velocity_ECI 148
#define Magnetic_Field_Vector 149
#define Coarse_Sun_Vector 150
#define Fine_Sun_Vector 151
#define Nadir_Vector 152
#define Rate_Sensor_Rates 153
#define Wheel_Speed 154
#define CubeSense_Current_Measurements 173
#define CubeControl_Current_Measurements 174
#define Peripheral_Current_And_Temperature_Measurements 175
#define Raw_Sensor_Measurements 139
#define Angular_Rate_Covariance 163
#define Raw_Nadir_Sensor 164
#define Raw_Sun_Sensor 165
#define Raw_CSS 166
#define Raw_Magnetometer 167
#define Raw_GPS_Status 168
#define Raw_GPS_Time 169
#define Raw_GPS_X 170
#define Raw_GPS_Y 171
#define Raw_GPS_Z 172
#define Estimation_Data 140
#define IGRF_Modelled_Magnetic_Field_Vector 157
#define Modelled_Sun_Vector 158
#define Estimated_Gyro_Bias 159
#define Estimated_Innovation_Vector 160
#define Quaternion_Error_Vector 161
#define Quaternion_Covariance 162
#define Magnetorquer_Command 155
#define Wheel_Speed_Commands 156
#define SGP4_Orbit_Parameters 191
#define Configuration 192
#define Status_Of_Image_Capture_And_Save_Operation 230
#define Uploaded_Program_Status 250
#define Get_Flash_Program_List 251


/*--------------------------*/

#endif /* FUNCTION_H_ */
