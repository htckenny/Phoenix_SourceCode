/*
 * adcs.c
 *ADCS test with ADCS bundle and deploy test
 *   Created on: 2016/1/26
 *       Author: Kai Wu
 *
 */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "subsystem.h"
#include <dev/i2c.h>
// #include <util/hexdump.h>
// #include <string.h> // memcpy
// #include <stdlib.h> //realloc
#include <inttypes.h>

// #define adcs_node 0x57

//The ADCS_Tasks should be launched after powering on the ADCS bundle
void ADCS_Task(void * pvParameters) {

	uint8_t txbuf[255];  //Define Tx buffer length
	uint8_t rxbuf[255];  //Define Rx buffer length

	//Useful monitoring data declaration
	// int16_t xrate = 0;        //X-axis angular rates
	// int16_t yrate = 0;        //y-axis angular rates
	// int16_t zrate = 0;        //Z-axis angular rates
	uint8_t flag_mag = 0;     //Magnetometer deployment flag
	// uint8_t flag_TRIAD = 0;   //TRIAD flag

	vTaskDelay(10 * delay_time_based); //Delay 10s when start ADCS

	printf("start to set to high initial rate detumbling\n");
	//Start high initial rate detumbling

	/*---------------------------Enable the ADCS to run mode-------------------------------*/
	txbuf[0] = 0x03;   //0d03 ADCS run mode
	txbuf[1] = 0x01;
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 2, &rxbuf, 0, adcs_delay) == E_NO_ERR)
		printf("ID:3\tSet ADCS run mode into enabled\n");


	/*---------------------------set the power control     -------------------------------*/
	txbuf[0] = 0x05;   //0d05 Set power control command
	txbuf[1] = 0x02;   //cubecontrol signal=auto(2)
	txbuf[2] = 0x01;   //cubecontrol motor=on(1)
	txbuf[3] = 0x02;   //cubesense =auto(2)
	txbuf[4] = 0x01;   //motor power=on(1)
	txbuf[5] = 0x02;   //GPS LAN Power=auto(2)
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 6, &rxbuf, 0, adcs_delay) == E_NO_ERR)
		printf("ID:05\tSet power control command\n");

	/*------------------------------Set the Control mode-----------------------------------*/
	txbuf[0] = 0x12;   //0d18 Set attitude control mode
	txbuf[1] = 0x01;   //Set to high initial rate detumbling control
	txbuf[2] = 0x00;   //Default
	txbuf[3] = 0x00;   //Default
	txbuf[4] = 0x00;   //Default
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 5, &rxbuf, 0, adcs_delay) == E_NO_ERR)
		printf("ID:18\tSet control mode into high initial rate detumbling\n");

	/*----------------------------Set the estimation mode----------------------------------*/
	txbuf[0] = 0x11;   //0d17 Set attitude estimation mode
	txbuf[1] = 0x02;   // Set to RKF mode
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 2, &rxbuf, 0, adcs_delay) == E_NO_ERR)
		printf("ID:17\tSet estimation mode into RKF\n");
	/*-------------------------------------------------------------------------------------*/
	printf("delay for 30s\n");
	vTaskDelay(30 * delay_time_based); //Delay 30 s


	printf("start to set to detumbling mode\n");
	//Start  detumbling mode

	/*---------------------------Enable the ADCS to run mode-------------------------------*/
	txbuf[0] = 0x03;   //0d03 ADCS run mode
	txbuf[1] = 0x01;
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 2, &rxbuf, 0, adcs_delay) == E_NO_ERR)
		printf("ID:3\tSet ADCS run mode into enabled\n");


	/*---------------------------set the power control     -------------------------------*/
	txbuf[0] = 0x05;   //0d05 Set power control command
	txbuf[1] = 0x02;   //cubecontrol signal=auto(2)
	txbuf[2] = 0x01;   //cubecontrol motor=on(1)
	txbuf[3] = 0x02;   //cubesense =auto(2)
	txbuf[4] = 0x01;   //motor power=on(1)
	txbuf[5] = 0x02;   //GPS LAN Power=auto(2)
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 6, &rxbuf, 0, adcs_delay) == E_NO_ERR)
		printf("ID:05\tSet power control command\n");
	/*------------------------------Set the Control mode-----------------------------------*/
	txbuf[0] = 0x12;   //0d18 Set attitude control mode
	txbuf[1] = 0x02;   //Set to detumbling control
	txbuf[2] = 0x00;   //set override flag disable
	txbuf[3] = 0x00;   //set control time out to infinite
	txbuf[4] = 0x00;   //
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 5, &rxbuf, 0, adcs_delay) == E_NO_ERR)
		printf("ID:18\tSet control mode into detumbling\n");

	/*----------------------------Set the estimation mode----------------------------------*/
	txbuf[0] = 0x11;   //0d17 Set attitude estimation mode
	txbuf[1] = 0x02;   // Set to RKF mode
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 2, &rxbuf, 0, adcs_delay) == E_NO_ERR)
		printf("ID:17\tSet estimation mode into RKF\n");
	/*-------------------------------------------------------------------------------------*/
	printf("delay for 30s\n");
	vTaskDelay(30 * delay_time_based); //Delay 30 s

	printf("start to open EKF estimate\n");
	//Start  to open EKF estimate

	/*---------------------------Enable the ADCS to run mode-------------------------------*/
	txbuf[0] = 0x03;   //0d03 ADCS run mode
	txbuf[1] = 0x01;
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 2, &rxbuf, 0, adcs_delay) == E_NO_ERR)
		printf("ID:3\tSet ADCS run mode into enabled\n");


	/*---------------------------set the power control     -------------------------------*/
	txbuf[0] = 0x05;   //0d05 Set power control command
	txbuf[1] = 0x02;   //cubecontrol signal=auto(2)
	txbuf[2] = 0x01;   //cubecontrol motor=on(1)
	txbuf[3] = 0x02;   //cubesense =auto(2)
	txbuf[4] = 0x01;   //motor power=on(1)
	txbuf[5] = 0x02;   //GPS LAN Power=auto(2)
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 6, &rxbuf, 0, adcs_delay) == E_NO_ERR)
		printf("ID:05\tSet power control command\n");
	/*------------------------------Set the Control mode-----------------------------------*/
	txbuf[0] = 0x12;   //0d18 Set attitude control mode
	txbuf[1] = 0x02;   //Set to detumbling control
	txbuf[2] = 0x00;   //set override flag disable
	txbuf[3] = 0x00;   //set control time out to infinite
	txbuf[4] = 0x00;   //
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 5, &rxbuf, 0, adcs_delay) == E_NO_ERR)
		printf("ID:18\tSet control mode into detumbling\n");

	/*----------------------------Set the estimation mode----------------------------------*/
	txbuf[0] = 0x11;   //0d17 Set attitude estimation mode
	txbuf[1] = 0x04;   // Set to EKF mode
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 2, &rxbuf, 0, adcs_delay) == E_NO_ERR)
		printf("ID:17\tSet estimation mode into EKF\n");
	/*-------------------------------------------------------------------------------------*/
	printf("delay for 30s\n");
	vTaskDelay(30 * delay_time_based); //Delay 30 s

	printf("start to  Y-Momentum Initial\n");
	//Start  to  Y-Momentum Initial

	/*---------------------------Enable the ADCS to run mode-------------------------------*/
	txbuf[0] = 0x03;   //0d03 ADCS run mode
	txbuf[1] = 0x01;
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 2, &rxbuf, 0, adcs_delay) == E_NO_ERR)
		printf("ID:3\tSet ADCS run mode into enabled\n");


	/*---------------------------set the power control     -------------------------------*/
	txbuf[0] = 0x05;   //0d05 Set power control command
	txbuf[1] = 0x02;   //cubecontrol signal=auto(2)
	txbuf[2] = 0x01;   //cubecontrol motor=on(1)
	txbuf[3] = 0x02;   //cubesense =auto(2)
	txbuf[4] = 0x01;   //motor power=on(1)
	txbuf[5] = 0x02;   //GPS LAN Power=auto(2)
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 6, &rxbuf, 0, adcs_delay) == E_NO_ERR)
		printf("ID:05\tSet power control command\n");
	/*------------------------------Set the Control mode-----------------------------------*/
	txbuf[0] = 0x12;   //0d18 Set attitude control mode
	txbuf[1] = 0x03;   //Set to Y-Momentum Initial
	txbuf[2] = 0x01;   //set override flag enable
	txbuf[3] = 0x2c;   //set control time out to infinite
	txbuf[4] = 0x01;   //
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 5, &rxbuf, 0, adcs_delay) == E_NO_ERR)
		printf("ID:18\tSet control mode into  Y-Momentum Initial \n");

	/*----------------------------Set the estimation mode----------------------------------*/
	txbuf[0] = 0x11;   //0d17 Set attitude estimation mode
	txbuf[1] = 0x04;   // Set to EKF mode
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 2, &rxbuf, 0, adcs_delay) == E_NO_ERR)
		printf("ID:17\tSet estimation mode into EKF\n");
	/*-------------------------------------------------------------------------------------*/
	printf("delay for 1minute\n");
	vTaskDelay(60 * delay_time_based); //Delay 60s
	printf("start to wait the deploy signal\n");
	while (1)
	{
		if (flag_mag == 0)
		{
			if (magnetometer_deploy == 1)
			{
				/*------------------------------Set the Control mode-----------------------------------*/
				txbuf[0] = 0x12;   //0d18 Set attitude control mode
				txbuf[1] = 0x00;   //Set to no control
				txbuf[2] = 0x00;   //set override flag disable
				txbuf[3] = 0x00;   //set control time out to infinite
				txbuf[4] = 0x00;   //
				if (i2c_master_transaction_2(0, adcs_node, &txbuf, 5, &rxbuf, 0, adcs_delay) == E_NO_ERR)
					printf("ID:18\tSet control mode into nocontrol\n");
				/*----------------------------Set the estimation mode----------------------------------*/
				txbuf[0] = 0x11;   //0d17 Set attitude estimation mode
				txbuf[1] = 0x00;   // Set to no estimate
				if (i2c_master_transaction_2(0, adcs_node, &txbuf, 2, &rxbuf, 0, adcs_delay) == E_NO_ERR)
					printf("ID:17\tSet estimation mode into no estimate\n");
				/*-------------------------------------------------------------------------------------*/
				txbuf[0] = 0x56;   //0d86 Set magnetometer configuration
				txbuf[1] = 0x46;   //mounting alpha vina:00B4/180
				txbuf[2] = 0x50;
				txbuf[3] = 0x46;   //mounting beta vina:00b4/180
				txbuf[4] = 0x50;
				txbuf[5] = 0xD8;   //mounting gamma  vina:ffa6/-90
				txbuf[6] = 0xDC;
				txbuf[7] = 0xB5;   //offset 1
				txbuf[8] = 0xFE;
				txbuf[9] = 0x32;   //offset 2
				txbuf[10] = 0x05;
				txbuf[11] = 0x70;  //offset 3
				txbuf[12] = 0x09;
				txbuf[13] = 0xCC;  //S11 Sensitivity Matrix
				txbuf[14] = 0x08;
				txbuf[15] = 0x00;  //S12 Sensitivity Matrix
				txbuf[16] = 0x00;
				txbuf[17] = 0x00;  //S13 Sensitivity Matrix
				txbuf[18] = 0x00;
				txbuf[19] = 0x00;  //S21 Sensitivity Matrix
				txbuf[20] = 0x00;
				txbuf[21] = 0xD9;  //S22 Sensitivity Matrix
				txbuf[22] = 0x08;
				txbuf[23] = 0x00;  //S23 Sensitivity Matrix
				txbuf[24] = 0x00;
				txbuf[25] = 0x00;  //S31 Sensitivity Matrix
				txbuf[26] = 0x00;
				txbuf[27] = 0x00;  //S32 Sensitivity Matrix
				txbuf[28] = 0x00;
				txbuf[29] = 0x3D;  //S33 Sensitivity Matrix
				txbuf[30] = 0x09;
				if (i2c_master_transaction_2(0, adcs_node, &txbuf, 31, &rxbuf, 0, adcs_delay) == E_NO_ERR)
					printf("ID:86\tSet magnetometer configuration\n");
				flag_mag = 1;
				printf("delay for 30s\n");
				vTaskDelay(30 * delay_time_based); //Delay 30 s
				/*------------------------------Set the Control mode-----------------------------------*/
				txbuf[0] = 0x12;   //0d18 Set attitude control mode
				txbuf[1] = 0x02;   //Set to detumbling control
				txbuf[2] = 0x00;   //set override flag disable
				txbuf[3] = 0x00;   //set control time out to infinite
				txbuf[4] = 0x00;   //
				if (i2c_master_transaction_2(0, adcs_node, &txbuf, 5, &rxbuf, 0, adcs_delay) == E_NO_ERR)
					printf("ID:18\tSet control mode into detumbling\n");
				/*----------------------------Set the estimation mode----------------------------------*/
				txbuf[0] = 0x11;   //0d17 Set attitude estimation mode
				txbuf[1] = 0x04;   // Set to EKF mode
				if (i2c_master_transaction_2(0, adcs_node, &txbuf, 2, &rxbuf, 0, adcs_delay) == E_NO_ERR)
					printf("ID:17\tSet estimation mode into EKF\n");
				/*-------------------------------------------------------------------------------------*/
			}
			else{
				vTaskDelay(3 * delay_time_based);
			}
		}
		else
		{
			printf("end of the test\n");
			vTaskDelay(300 * delay_time_based); //Delay 300 s
		}
	}


	/** End of ADCS TASK, Should never reach this line  */
	vTaskDelete(NULL);
}


 