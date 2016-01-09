/*
 * adcs.c
 *
 *  Created on: 2015/6/10
 *      Author: rusei
 */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "subsystem.h"
#include <dev/i2c.h>

// #define adcs_node 0x57
uint8_t txbuf[255];
uint8_t rxbuf[255];


void ADCS_Task(void * pvParameters) {

	vTaskDelay(10 * delay_time_based); //Delay 10s when start ADCS

	//Start detumbling
	txbuf[0] = 0x03;   //0d03 ADCS run mode
	txbuf[1] = 0x01;
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 2, &rxbuf, 0, adcs_delay) == E_NO_ERR)
		printf("ID:3\tSet ADCS run mode into enabled\n");

	txbuf[0] = 0x12;   //0d18 Set attitude control mode
	txbuf[1] = 0x02;
	txbuf[2] = 0x00;
	txbuf[3] = 0x00;
	txbuf[4] = 0x00;
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 5, &rxbuf, 0, adcs_delay) == E_NO_ERR)
		printf("ID:18\tSet control mode into detumbling\n");

	txbuf[0] = 0x11;   //0d17 Set attitude estimation mode
	txbuf[1] = 0x01;
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 2, &rxbuf, 0, adcs_delay) == E_NO_ERR)
		printf("ID:17\tSet estimation mode into MEMS rate sensing\n");

	int16_t xrate=0; //X-axis angular rates
	int16_t yrate=0; //Z-axis angular rates
	int16_t zrate=0; //Z-axis angular rates
	uint8_t flag_mag = 0; //Magnetometer deployment flag
	uint8_t flag_TRIAD = 0; //TRIAD flag

	while (1) {

		//Mode transition
		txbuf[0] = 0x92;   //0d146 Estimated angular rates
		if (i2c_master_transaction_2(0, adcs_node, &txbuf, 1, &rxbuf, 6, adcs_delay) == E_NO_ERR) {
			xrate = rxbuf[0] + (rxbuf[1] << 8); //   *256 = <<8, /256= >>8
			yrate = rxbuf[2] + (rxbuf[3] << 8); //   *256 = <<8, /256= >>8
			zrate = rxbuf[4] + (rxbuf[5] << 8); //   *256 = <<8, /256= >>8
		}
		// printf("ADCS measurement taken\n");
		// printf("Xrate = %d\n", xrate);
		// printf("Yrate = %d\n", yrate);
		// printf("Zrate = %d\n", zrate);

		if ((yrate >= -2700) && (yrate <= -1700)) {			//Please check the negative value

			if (flag_TRIAD == 0) {
				txbuf[0] = 0x11;   //0d17 Set attitude estimation mode
				txbuf[1] = 0x05;
				if (i2c_master_transaction_2(0, adcs_node, &txbuf, 2, &rxbuf, 0, adcs_delay) == E_NO_ERR)
				{
					printf("ID:17\tSet estimation mode into magnetometer and fine sun TRIAD algorithm\n");
					flag_TRIAD = 1;
				}
			}

			//Mode transition to Y-momentum with EKF
			if (xrate <= 500 && xrate >= -500 && yrate >= -2700 && yrate <= -1700 && zrate <= 500 && zrate >= -500) {
				vTaskDelay(300 * delay_time_based); // Continue using TRIAD for 300s

				txbuf[0] = 0x12;   //0d18 Set attitude control mode
				txbuf[1] = 0x03;
				txbuf[2] = 0x00;
				txbuf[3] = 0x00;
				txbuf[4] = 0x00;
				if (i2c_master_transaction(0, adcs_node, &txbuf, 5, &rxbuf, 0, adcs_delay) == E_NO_ERR)
					printf("ID:18\tSet control mode into Y-momentum\n");

				txbuf[0] = 0x11;   //0d17 Set attitude estimation mode
				txbuf[1] = 0x04;
				if (i2c_master_transaction(0, adcs_node, &txbuf, 2, &rxbuf, 0, adcs_delay) == E_NO_ERR)
					printf("ID:17\tSet estimation mode into EKF\n");
			}
			else {
				txbuf[0] = 0x12;   //0d18 Set attitude control mode
				txbuf[1] = 0x02;
				txbuf[2] = 0x00;
				txbuf[3] = 0x00;
				txbuf[4] = 0x00;
				if (i2c_master_transaction(0, adcs_node, &txbuf, 5, &rxbuf, 0, adcs_delay) == E_NO_ERR)
					printf("ID:18\tSet control mode into detumbling\n"); //go back to detumbling
			}
		}
		//Parameter changes
		//Change magnetometer configuration after deployment
		if (flag_mag == 0) {
			txbuf[0] = 0xA6;   //0d166 Raw CSS,CSS4 is below magnetometer
			if (i2c_master_transaction_2(0, adcs_node, &txbuf, 1, &rxbuf, 6, adcs_delay) == E_NO_ERR){
				// printf("CSS4 measurement= %d\n", rxbuf[3]);
			}
		}

		if ((rxbuf[3] >= 100) && flag_mag == 0)	{
			txbuf[0] = 0x56;   //0d86 Set magnetometer configuration
			txbuf[1] = 0xB4;   //mounting alpha
			txbuf[2] = 0x00;
			txbuf[3] = 0xB4;   //mounting beta
			txbuf[4] = 0x00;
			txbuf[5] = 0xA6;   //mounting gamma
			txbuf[6] = 0xFF;
			txbuf[7] = 0xB5;   //offset 1
			txbuf[8] = 0xFE;
			txbuf[9] = 0x32;   //offset 2
			txbuf[10] = 0x05;
			txbuf[11] = 0x70;  //offset 3
			txbuf[12] = 0x09;
			txbuf[13] = 0xCC;  //S11
			txbuf[14] = 0x08;
			txbuf[15] = 0x00;  //S12
			txbuf[16] = 0x00;
			txbuf[17] = 0x00;  //S13
			txbuf[18] = 0x00;
			txbuf[19] = 0x00;  //S21
			txbuf[20] = 0x00;
			txbuf[21] = 0xD9;  //S22
			txbuf[22] = 0x08;
			txbuf[23] = 0x00;  //S23
			txbuf[24] = 0x00;
			txbuf[25] = 0x00;  //S31
			txbuf[26] = 0x00;
			txbuf[27] = 0x00;  //S32
			txbuf[28] = 0x00;
			txbuf[29] = 0x3D;  //S33
			txbuf[30] = 0x09;
			if (i2c_master_transaction(0, adcs_node, &txbuf, 31, &rxbuf, 0, adcs_delay) == E_NO_ERR)
				printf("ID:86\tSet magnetometer configuration\n");
			flag_mag = 1;

			//Write the configuration to flash memory
			txbuf[0] = 0x64;   //0d100 Save current configuration to flash memory
			if (i2c_master_transaction(0, adcs_node, &txbuf, 1, &rxbuf, 0, adcs_delay) == E_NO_ERR)
				printf("ID:100\tSave current configuration to flash memory\n");

			//Check by reading HK
			txbuf[0] = 0xC0;   //0d192 Current configuration
			if (i2c_master_transaction(0, adcs_node, &txbuf, 1, &rxbuf, 236, adcs_delay) == E_NO_ERR)	{
				printf("Magnetometer mounting alpha= %d\n", (rxbuf[114] + (rxbuf[115] << 8)));
				printf("Magnetometer mounting beta= %d\n", (rxbuf[116] + (rxbuf[117] << 8)));
				printf("Magnetometer mounting gamma= %d\n", (rxbuf[118] + (rxbuf[119] << 8)));
			}
		}
		vTaskDelay(5 * delay_time_based);
	}
	/** End of ADCS TASK, Should never reach this line  */
	vTaskDelete(NULL);
}
