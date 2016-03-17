/**
	*******************************************************************
 	* @file		task_ADCS_test_3.0.c
 	* @Author 	Jerry Wu, Kai Wu
 	* @Version	V1.0
 	* @Date     2016/1/25
 	* @Update   2016/3/2
 	* @Brief	This program is the flight version for the ADCS flight
 	*			software. Pseudo data would be given from the stm MCU
 	*			as sensor values. The goal is to test the algorthm witg
 	*			various of conditional logics and statements. As a note,
 	*			the I2C stm node will be set as 127 (0X7F). Once Phoenix
 	*			is going to be delivered to VKI, the stm i2c node will
 	*           changed to adcs_node (0x57) from the OBDH side(supreme
 	*			command authority), in order to be the real flight verson.
 **********************************************************************
 **/
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "subsystem.h"
#include <dev/i2c.h>
#include <util/hexdump.h>
#include <string.h> // memcpy
#include <stdlib.h> //realloc
#include <inttypes.h>
#include <stdbool.h>

#define DATALENGTH 1;

// #define adcs_node 0x57
// #define stm_node 0x7F
//#define adcs_delay 7

uint8_t txbuf[255];  //Define Tx buffer length
uint8_t rxbuf[255];  //Define Rx buffer length
int16_t Angular_Y_rate_difference[2];
int16_t Angular_Y_reference_rate = -220;
float adcs_state_delay = 0.9;

//Declare the structure of the estimated value of angular rate
typedef struct __attribute__((packed))
{
	int16_t angular_X_rate[2];
	int16_t angular_Y_rate[2];
	int16_t angular_Z_rate[2];

} adcs_est_ang_rates_t;
adcs_est_ang_rates_t adcs_est_ang_rates;

//Delcare the structure of the estimated value of RPY angle
typedef struct __attribute__((packed))
{
	int16_t roll_angle[2];
	int16_t pitch_angle[2];
	int16_t yaw_angle[2];
} adcs_est_rpy_angle_t;
adcs_est_rpy_angle_t adcs_est_rpy_angle;

//Delcare the structure of the raw measurement value of magnetometer
typedef struct __attribute__((packed))
{
	int16_t mag_raw_measurement_X[2];
	int16_t mag_raw_measurement_Y[2];
	int16_t mag_raw_measurement_Z[2];
} adcs_mag_raw_measurement_t;
adcs_mag_raw_measurement_t adcs_mag_raw_measurement;

//Delcare the structure of the adcs status

typedef struct __attribute__((packed))
{
	int initial_flag;
	int high_initial_flag1;
	int high_initial_flag2;
	int detumbling_rkf_flag;
	int detumbling_ekf_flag;
	int y_momentum_intial_flag;
	//int y_momentum_steady_flag;
} adcs_status_t;
adcs_status_t adcs_status;

//Delcare the structure of the adcs process

typedef struct __attribute__((packed))
{
	int initialize_flag;
	int stabilize_flag;
	int start_mag_deploy_flag;
} adcs_process_t;
adcs_process_t adcs_process;


void Initial_Mode()
{
	/*---------------------------Enable the ADCS to run mode-------------------------------*/
	txbuf[0] = 0x03;   //0d03 ADCS run mode
	txbuf[1] = 0x01;
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 2, &rxbuf, 0, adcs_delay) == E_NO_ERR)
		printf("ID:3\tSet ADCS run mode into enabled\n");

	/*---------------------------set the power control-------------------------------------*/
	txbuf[0] = 0x05;   //0d05 Set power control command
	txbuf[1] = 0x02;   //cubecontrol signal=auto(2)
	txbuf[2] = 0x01;   //cubecontrol motor=on(1)
	txbuf[3] = 0x02;   //cubesense =auto(2)
	txbuf[4] = 0x01;   //motor power=on(1)
	txbuf[5] = 0x02;   //GPS LAN Power=auto(2)
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 6, &rxbuf, 0, adcs_delay) == E_NO_ERR)
		printf("ID:05\tSet power control command\n");

	/*----------------------------Set the estimation mode----------------------------------*/
	txbuf[0] = 0x11;   //0d17 Set attitude estimation mode
	txbuf[1] = 0x02;   // Set to magnetometer rate filters mode
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 2, &rxbuf, 0, adcs_delay) == E_NO_ERR)
		printf("ID:17\tSet estimation mode into Magnetometer Rate Filters\n");

	/*---------------------------Triggering detecting--------------------------------------*/
	/*-----------------------Dominant Y-spin identification--------------------------------*/

	int Initial_mode_1st_samples = 60;
	float Initial_mode_1st_conditional_ratio = 0.5;
	float Initial_mode_1st_count = 0;
	float Initial_mode_1st_count_total = 0;

	int Initial_mode_2nd_samples = 30;
	float Initial_mode_2nd_conditional_ratio = 0.8;
	float Initial_mode_2nd_count = 0;
	float Initial_mode_2nd_count_total = 0;

	/*----------------------------------Proortional condition triggering-------------------------------------*/
	while (1)
	{
		while (Initial_mode_1st_count_total < Initial_mode_1st_samples)
		{

			txbuf[0] = 0x92;   //0d146 Estimated angular rates
			if (i2c_master_transaction_2(0, stm_node, &txbuf, 1, &rxbuf, 6, adcs_delay) == E_NO_ERR)
			{
				adcs_est_ang_rates.angular_X_rate[1] = rxbuf[0] + (rxbuf[1] << 8); //   *256 = <<8, /256= >>8
				adcs_est_ang_rates.angular_Y_rate[1] = rxbuf[2] + (rxbuf[3] << 8); //   *256 = <<8, /256= >>8
				adcs_est_ang_rates.angular_Z_rate[1] = rxbuf[4] + (rxbuf[5] << 8); //   *256 = <<8, /256= >>8

				printf("\tWe are in the first logic condition");

				printf("\tEstimate X angular rate = %d", adcs_est_ang_rates.angular_X_rate[1]);
				printf("\tEstimate Z angular rate = %d\n", adcs_est_ang_rates.angular_Z_rate[1]);

			}

			if ((adcs_est_ang_rates.angular_X_rate[1] >= -500) && (adcs_est_ang_rates.angular_X_rate[1] <= 500)
			        && (adcs_est_ang_rates.angular_Z_rate[1] >= -500) && (adcs_est_ang_rates.angular_Z_rate[1] <= 500))
			{
				Initial_mode_1st_count += 1;
				Initial_mode_1st_count_total += 1;
			}
			else
			{
				Initial_mode_1st_count_total += 1;
			}

			vTaskDelay(adcs_state_delay * delay_time_based);
		}



		if ((Initial_mode_1st_count / Initial_mode_1st_count_total) >= Initial_mode_1st_conditional_ratio) //The proportion has to larger than 0.5
		{
			while (Initial_mode_2nd_count_total < Initial_mode_2nd_samples)
			{
				txbuf[0] = 0x92;   //0d146 Estimated angular rates
				if (i2c_master_transaction_2(0, stm_node, &txbuf, 1, &rxbuf, 6, adcs_delay) == E_NO_ERR)
				{
					adcs_est_ang_rates.angular_X_rate[1] = rxbuf[0] + (rxbuf[1] << 8); //   *256 = <<8, /256= >>8
					adcs_est_ang_rates.angular_Y_rate[1] = rxbuf[2] + (rxbuf[3] << 8); //   *256 = <<8, /256= >>8
					adcs_est_ang_rates.angular_Z_rate[1] = rxbuf[4] + (rxbuf[5] << 8); //   *256 = <<8, /256= >>8
				}

				printf("\tWe are in the Second logic condition (In yes result)");
				printf("\tEstimate Y angular rate = %d\n", adcs_est_ang_rates.angular_Y_rate[1]);

				if ((adcs_est_ang_rates.angular_Y_rate[1] >= -3000) && (adcs_est_ang_rates.angular_Y_rate[1] <= 3000))
				{
					Initial_mode_2nd_count += 1;
					Initial_mode_2nd_count_total += 1;
				}
				else
				{
					Initial_mode_2nd_count_total += 1;
				}

				vTaskDelay(adcs_state_delay * delay_time_based);
			}

			if ((Initial_mode_2nd_count / Initial_mode_2nd_count_total) >= Initial_mode_2nd_conditional_ratio)
			{
				adcs_status.initial_flag = 0;
				adcs_status.high_initial_flag1 = 0;
				adcs_status.high_initial_flag2 = 0;

				adcs_process.initialize_flag = 0;
				adcs_process.stabilize_flag = 1;
				printf("U jump to Detumbling Control mode\n");
				break;
			}
			else
			{
				adcs_status.initial_flag = 0;
				adcs_status.high_initial_flag1 = 1;
				adcs_status.high_initial_flag2 = 0;

				printf("U jump to High Intitial Rate Detumbling mode\n");
				break;
			}
		}
		else
		{
			while (Initial_mode_2nd_count_total < Initial_mode_2nd_samples)
			{

				txbuf[0] = 0x92;   //0d146 Estimated angular rates
				if (i2c_master_transaction_2(0, stm_node, &txbuf, 1, &rxbuf, 6, adcs_delay) == E_NO_ERR)
				{
					adcs_est_ang_rates.angular_X_rate[1] = rxbuf[0] + (rxbuf[1] << 8); //   *256 = <<8, /256= >>8
					adcs_est_ang_rates.angular_Y_rate[1] = rxbuf[2] + (rxbuf[3] << 8); //   *256 = <<8, /256= >>8
					adcs_est_ang_rates.angular_Z_rate[1] = rxbuf[4] + (rxbuf[5] << 8); //   *256 = <<8, /256= >>8
				}

				printf("\tWe are in the Second logic condition(In no result)\n");

				printf("\tEstimate X angular rate = %d", adcs_est_ang_rates.angular_X_rate[1]);
				printf("\tEstimate Y angular rate = %d", adcs_est_ang_rates.angular_Y_rate[1]);
				printf("\tEstimate Z angular rate = %d", adcs_est_ang_rates.angular_Z_rate[1]);


				if ((adcs_est_ang_rates.angular_X_rate[1] >= -1500) && (adcs_est_ang_rates.angular_X_rate[1] <= 1500)
				        && (adcs_est_ang_rates.angular_Y_rate[1] >= -1500) && (adcs_est_ang_rates.angular_Y_rate[1] <= 1500)
				        && (adcs_est_ang_rates.angular_Z_rate[1] >= -1500) && (adcs_est_ang_rates.angular_Z_rate[1] <= 1500))

				{
					Initial_mode_2nd_count += 1;
					Initial_mode_2nd_count_total += 1;
				}
				else
				{
					Initial_mode_2nd_count_total += 1;
				}

				vTaskDelay(adcs_state_delay * delay_time_based);
			}

			if ((Initial_mode_2nd_count / Initial_mode_2nd_count_total) >= Initial_mode_2nd_conditional_ratio)
			{
				adcs_status.initial_flag = 0;
				adcs_status.high_initial_flag1 = 0;
				adcs_status.high_initial_flag2 = 0;

				adcs_process.initialize_flag = 0;
				adcs_process.stabilize_flag = 1;
				printf("U jump to Detumbling Control mode\n");
				break;
			}
			else
			{
				adcs_status.initial_flag = 0;
				adcs_status.high_initial_flag1 = 0;
				adcs_status.high_initial_flag2 = 1;

				printf("U jump to High Intitial Rate Detumbling mode\n");
				break;
			}
		}
		vTaskDelay(adcs_state_delay * delay_time_based);
		break;
	}
}

void High_Initial_Rate_Detumbling(int high_initial_identifier)
{
	/*---------------------------Enable the ADCS to run mode-------------------------------*/
	txbuf[0] = 0x03;   //0d03 ADCS run mode
	txbuf[1] = 0x01;
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 2, &rxbuf, 0, adcs_delay) == E_NO_ERR)
		printf("ID:3\tSet ADCS run mode into enabled\n");

	/*---------------------------set the power control-------------------------------------*/
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
		printf("ID:17\tSet estimation mode into Magnetometer Rate Filter\n");

	/*---------------------------Triggering detecting--------------------------------------*/
	int High_Initial_mode_count = 0;
	int High_Initial_mode_flag = 0;
	int High_Initial_mode_qualified_samples = 60;


	while (1)
	{
		txbuf[0] = 0x92;   //0d146 Estimated angular rates
		if (i2c_master_transaction_2(0, stm_node, &txbuf, 1, &rxbuf, 6, adcs_delay) == E_NO_ERR)
		{
			adcs_est_ang_rates.angular_X_rate[1] = rxbuf[0] + (rxbuf[1] << 8); //   *256 = <<8, /256= >>8
			adcs_est_ang_rates.angular_Y_rate[1] = rxbuf[2] + (rxbuf[3] << 8); //   *256 = <<8, /256= >>8
			adcs_est_ang_rates.angular_Z_rate[1] = rxbuf[4] + (rxbuf[5] << 8); //   *256 = <<8, /256= >>8
		}

		/*---------------------------Continously condition triggering (Previously in Yes result)-----------------------------*/
		if (high_initial_identifier == 1)
		{
			printf("\tEstimate Y angular rate = %d\n", adcs_est_ang_rates.angular_Y_rate[1]);
			if (High_Initial_mode_flag == 0)
			{
				if ((adcs_est_ang_rates.angular_Y_rate[1] >= -3000) && (adcs_est_ang_rates.angular_Y_rate[1] <= 3000))
				{
					High_Initial_mode_count = 0;
					High_Initial_mode_count += 1;
					High_Initial_mode_flag = 1;
				}
				else
				{
					High_Initial_mode_flag = 0;
				}
			}
			else
			{
				if (High_Initial_mode_count != 0 )
				{
					if ((adcs_est_ang_rates.angular_Y_rate[1] >= -3000) && (adcs_est_ang_rates.angular_Y_rate[1] <= 3000)
					        && (adcs_est_ang_rates.angular_Y_rate[0] <= 3000) && (adcs_est_ang_rates.angular_Y_rate[0] <= 3000))

						High_Initial_mode_count += 1;

					else
					{
						High_Initial_mode_flag = 0;
						High_Initial_mode_count = 0;
					}
				}
				else
				{
					High_Initial_mode_flag = 0;
					High_Initial_mode_count = 0;
				}

			}


			if (High_Initial_mode_count > High_Initial_mode_qualified_samples)
			{
				adcs_status.initial_flag = 0;
				adcs_status.high_initial_flag1 = 0;
				adcs_status.high_initial_flag2 = 0;

				adcs_process.initialize_flag = 0;
				adcs_process.stabilize_flag = 1;
				printf("u jump to detumbling mode\n");
				break;
			}
			else
			{
				adcs_status.initial_flag = 0;
				adcs_status.high_initial_flag1 = 1;
				adcs_status.high_initial_flag2 = 0;

				printf("u stay in High Intitial Rate Detumbling\n");
				//break;
			}

			vTaskDelay(adcs_state_delay * delay_time_based);
			adcs_est_ang_rates.angular_Y_rate[0] = adcs_est_ang_rates.angular_Y_rate[1];
		}

		else
		{
			printf("\tWe jump to the second logic condition (In no result)\n");
			printf("\tEstimate X angular rate = %d", adcs_est_ang_rates.angular_X_rate[1]);
			printf("\tEstimate Y angular rate = %d", adcs_est_ang_rates.angular_Y_rate[1]);
			printf("\tEstimate Z angular rate = %d", adcs_est_ang_rates.angular_Z_rate[1]);
			/*---------------------------Continously condition triggering (Previously in No result)-----------------------------*/

			if (High_Initial_mode_flag == 0)
			{
				if ((adcs_est_ang_rates.angular_X_rate[1] >= -1500) && (adcs_est_ang_rates.angular_X_rate[1] <= 1500)
				        && (adcs_est_ang_rates.angular_Y_rate[1] >= -1500) && (adcs_est_ang_rates.angular_Y_rate[1] <= 1500)
				        && (adcs_est_ang_rates.angular_Z_rate[1] >= -1500) && (adcs_est_ang_rates.angular_Z_rate[1] <= 1500))
				{
					High_Initial_mode_count = 0;
					High_Initial_mode_count += 1;
					High_Initial_mode_flag = 1;
				}

				else
				{
					High_Initial_mode_flag = 0;
				}
			}
			else
			{
				if (High_Initial_mode_count != 0 )
				{
					if ((adcs_est_ang_rates.angular_X_rate[1] >= -1500) && (adcs_est_ang_rates.angular_X_rate[1] <= 1500)
					        && (adcs_est_ang_rates.angular_Y_rate[1] >= -1500) && (adcs_est_ang_rates.angular_Y_rate[1] <= 1500)
					        && (adcs_est_ang_rates.angular_Z_rate[1] >= -1500) && (adcs_est_ang_rates.angular_Z_rate[1] <= 1500))
					{
						if ((adcs_est_ang_rates.angular_X_rate[0] >= -1500) && (adcs_est_ang_rates.angular_X_rate[0] <= 1500)
						        && (adcs_est_ang_rates.angular_Y_rate[0] >= -1500) && (adcs_est_ang_rates.angular_Y_rate[0] <= 1500)
						        && (adcs_est_ang_rates.angular_Z_rate[0] >= -1500) && (adcs_est_ang_rates.angular_Z_rate[0] <= 1000))

							High_Initial_mode_count += 1;

						else
						{
							High_Initial_mode_flag = 0;
							High_Initial_mode_count = 0;
						}
					}
					else
					{
						High_Initial_mode_flag = 0;
						High_Initial_mode_count = 0;
					}
				}
				else
				{
					High_Initial_mode_flag = 0;
					High_Initial_mode_count = 0;
				}
			}


			if (High_Initial_mode_count > High_Initial_mode_qualified_samples)
			{
				printf("u success\n");

				adcs_status.initial_flag = 0;
				adcs_status.high_initial_flag1 = 0;
				adcs_status.high_initial_flag2 = 0;

				adcs_process.initialize_flag = 0;
				adcs_process.stabilize_flag = 1;
				printf("u jump to detumbling mode\n");
				break;
			}

			else
			{

				adcs_status.initial_flag = 0;
				adcs_status.high_initial_flag1 = 0;
				adcs_status.high_initial_flag2 = 1;

				printf("u stay in High Intitial Rate Detumbling\n");
				//break;
			}

			vTaskDelay(adcs_state_delay * delay_time_based);
			adcs_est_ang_rates.angular_X_rate[0] = adcs_est_ang_rates.angular_X_rate[1];
			adcs_est_ang_rates.angular_Z_rate[0] = adcs_est_ang_rates.angular_Z_rate[1];
		}
	}
	vTaskDelay(adcs_state_delay * delay_time_based);
}

void Detumbling_Control_RKF()
{
	/*---------------------------Enable the ADCS to run mode-------------------------------*/
	txbuf[0] = 0x03;   //0d03 ADCS run mode
	txbuf[1] = 0x01;
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 2, &rxbuf, 0, adcs_delay) == E_NO_ERR)
		printf("ID:3\tSet ADCS run mode into enabled\n");

	/*---------------------------set the power control-------------------------------------*/
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
	txbuf[1] = 0x02;   //Set to high initial rate detumbling control
	txbuf[2] = 0x00;   //Default
	txbuf[3] = 0x00;   //Default
	txbuf[4] = 0x00;   //Default
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 5, &rxbuf, 0, adcs_delay) == E_NO_ERR)
		printf("ID:18\tSet control mode into detumbling control\n");

	/*----------------------------Set the estimation mode----------------------------------*/
	txbuf[0] = 0x11;   //0d17 Set attitude estimation mode
	txbuf[1] = 0x02;   // Set to magnetomter rate filter mode
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 2, &rxbuf, 0, adcs_delay) == E_NO_ERR)
		printf("ID:17\tSet estimation mode into Magnetometer Rate Filter\n");

	/*---------------------------Triggering detecting--------------------------------------*/

	int Detumbling_RKF_mode_count = 0;
	int Detumbling_RKF_mode_flag = 0;
	int Detumbling_RKF_mode_qualified_samples = 60;

	while (1)
	{
		txbuf[0] = 0x92;   //0d146 Estimated angular rates
		if (i2c_master_transaction_2(0, stm_node, &txbuf, 1, &rxbuf, 6, adcs_delay) == E_NO_ERR)
		{
			adcs_est_ang_rates.angular_X_rate[1] = rxbuf[0] + (rxbuf[1] << 8); //   *256 = <<8, /256= >>8
			adcs_est_ang_rates.angular_Y_rate[1] = rxbuf[2] + (rxbuf[3] << 8); //   *256 = <<8, /256= >>8
			adcs_est_ang_rates.angular_Z_rate[1] = rxbuf[4] + (rxbuf[5] << 8); //   *256 = <<8, /256= >>8
		}

		/*---------------------------Continously condition triggering (Previously in Yes result)-----------------------------*/
		printf("\tEstimate Y angular rate = %d\n", adcs_est_ang_rates.angular_Y_rate[1]);

		Angular_Y_rate_difference[1] = adcs_est_ang_rates.angular_Y_rate[1] - Angular_Y_reference_rate;
		Angular_Y_rate_difference[0] = adcs_est_ang_rates.angular_Y_rate[0] - Angular_Y_reference_rate;

		if (Detumbling_RKF_mode_flag == 0)
		{
			if ((Angular_Y_rate_difference[1] >= -50) && (adcs_est_ang_rates.angular_Y_rate[1] <= 50))
			{
				Detumbling_RKF_mode_count = 0;
				Detumbling_RKF_mode_count += 1;
				Detumbling_RKF_mode_flag = 1;
			}
			else
			{
				Detumbling_RKF_mode_flag = 0;
			}
		}
		else
		{
			if (Detumbling_RKF_mode_count != 0 )
			{
				if ((Angular_Y_rate_difference[1] >= -50) && (Angular_Y_rate_difference[1] <= 50)
				        && (Angular_Y_rate_difference[0] >= -50) && (Angular_Y_rate_difference[0] <= 50))

					Detumbling_RKF_mode_count += 1;

				else
				{
					Detumbling_RKF_mode_flag = 0;
					Detumbling_RKF_mode_count = 0;
				}
			}
			else
			{
				Detumbling_RKF_mode_flag = 0;
				Detumbling_RKF_mode_count = 0;
			}

		}


		if (Detumbling_RKF_mode_count  >= Detumbling_RKF_mode_qualified_samples)
		{
			adcs_status.detumbling_rkf_flag = 0;
			adcs_status.detumbling_ekf_flag = 1;
			adcs_status.y_momentum_intial_flag = 0;

			printf("u jump to Detumbling Control with EKF\n");
			break;
		}
		else
		{
			adcs_status.detumbling_rkf_flag = 1;
			adcs_status.detumbling_ekf_flag = 0;
			adcs_status.y_momentum_intial_flag = 0;

			printf("u stay in Detumbling Control with RKF\n");
		}

		vTaskDelay(adcs_state_delay * delay_time_based);
		adcs_est_ang_rates.angular_Y_rate[0] = adcs_est_ang_rates.angular_Y_rate[1];
		Angular_Y_rate_difference[0] = Angular_Y_rate_difference[1];
	}


}

void Detumbling_Control_EKF()
{
	/*---------------------------Enable the ADCS to run mode-------------------------------*/
	txbuf[0] = 0x03;   //0d03 ADCS run mode
	txbuf[1] = 0x01;
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 2, &rxbuf, 0, adcs_delay) == E_NO_ERR)
		printf("ID:3\tSet ADCS run mode into enabled\n");

	/*---------------------------set the power control-------------------------------------*/
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
	txbuf[1] = 0x02;   //Set to high initial rate detumbling control
	txbuf[2] = 0x00;   //Default
	txbuf[3] = 0x00;   //Default
	txbuf[4] = 0x00;   //Default
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 5, &rxbuf, 0, adcs_delay) == E_NO_ERR)
		printf("ID:18\tSet control mode into detumbling control\n");

	/*----------------------------Set the estimation mode----------------------------------*/
	txbuf[0] = 0x11;   //0d17 Set attitude estimation mode
	txbuf[1] = 0x04;   // Set to Extended kalman filter mode
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 2, &rxbuf, 0, adcs_delay) == E_NO_ERR)
		printf("ID:17\tSet estimation mode into Extended Kalman Filter\n");
	/*---------------------------Triggering detecting--------------------------------------*/

	int Detumbling_EKF_mode_count = 0;
	int Detumbling_EKF_mode_flag = 0;
	int Detumbling_EKF_mode_qualified_samples = 60;

	while (1)
	{
		txbuf[0] = 0x92;   //0d146 Estimated angular rates
		if (i2c_master_transaction_2(0, stm_node, &txbuf, 1, &rxbuf, 6, adcs_delay) == E_NO_ERR)
		{
			adcs_est_ang_rates.angular_X_rate[1] = rxbuf[0] + (rxbuf[1] << 8); //   *256 = <<8, /256= >>8
			adcs_est_ang_rates.angular_Y_rate[1] = rxbuf[2] + (rxbuf[3] << 8); //   *256 = <<8, /256= >>8
			adcs_est_ang_rates.angular_Z_rate[1] = rxbuf[4] + (rxbuf[5] << 8); //   *256 = <<8, /256= >>8
		}
		vTaskDelay(0.1 * delay_time_based);
		txbuf[0] = 0x91;   //0d146 Estimated RPY angle
		if (i2c_master_transaction_2(0, stm_node, &txbuf, 1, &rxbuf, 6, adcs_delay) == E_NO_ERR)
		{
			adcs_est_rpy_angle.pitch_angle[1] = rxbuf[2] + (rxbuf[3] << 8); //   *256 = <<8, /256= >>8
		}

		/*---------------------------Continously condition triggering (Previously in Yes result)-----------------------------*/
		printf("\tEstimate X angular rate = %d\n", adcs_est_ang_rates.angular_X_rate[1]);
		printf("\tEstimate Y angular rate = %d\n", adcs_est_ang_rates.angular_Y_rate[1]);
		printf("\tEstimate Z angular rate = %d\n", adcs_est_ang_rates.angular_Z_rate[1]);
		printf("\tEstimate Pitch angle = %d\n", adcs_est_rpy_angle.pitch_angle[1]);

		Angular_Y_rate_difference[1] = adcs_est_ang_rates.angular_Y_rate[1] - Angular_Y_reference_rate;
		Angular_Y_rate_difference[0] = adcs_est_ang_rates.angular_Y_rate[0] - Angular_Y_reference_rate;


		if (Detumbling_EKF_mode_flag == 0)
		{
			if ((Angular_Y_rate_difference[1] >= -50) && (Angular_Y_rate_difference[1] <= 50))
				if ((adcs_est_rpy_angle.pitch_angle[1] >= -2000) && (adcs_est_rpy_angle.pitch_angle[1] <= 2000))
					if ((adcs_est_ang_rates.angular_X_rate[1] >= -500) && (adcs_est_ang_rates.angular_X_rate[1] <= 500)
					        && (adcs_est_ang_rates.angular_Z_rate[1] >= -500) && (adcs_est_ang_rates.angular_Z_rate[1] <= 500))
					{
						Detumbling_EKF_mode_count = 0;
						Detumbling_EKF_mode_count += 1;
						Detumbling_EKF_mode_flag = 1;
					}
					else
					{
						Detumbling_EKF_mode_flag = 0;
					}
				else
				{
					Detumbling_EKF_mode_flag = 0;
				}
			else
			{
				Detumbling_EKF_mode_flag = 0;
			}
		}
		else
		{
			if (Detumbling_EKF_mode_count != 0 )
			{
				if ((Angular_Y_rate_difference[1] >= -50) && (Angular_Y_rate_difference[1] <= 50)
				        && (Angular_Y_rate_difference[0] >= -50) && (Angular_Y_rate_difference[1] <= 50))
					if ((adcs_est_rpy_angle.pitch_angle[1] >= -2000) && (adcs_est_rpy_angle.pitch_angle[1] <= 2000)
					        && (adcs_est_rpy_angle.pitch_angle[0] >= -2000) && (adcs_est_rpy_angle.pitch_angle[0] <= 2000))
						if ((adcs_est_ang_rates.angular_X_rate[1] >= -500) && (adcs_est_ang_rates.angular_X_rate[1] <= 500)
						        && (adcs_est_ang_rates.angular_Z_rate[1] >= -500) && (adcs_est_ang_rates.angular_Z_rate[1] <= 500)
						        && (adcs_est_ang_rates.angular_X_rate[0] >= -500) && (adcs_est_ang_rates.angular_X_rate[0] <= 500)
						        && (adcs_est_ang_rates.angular_Z_rate[0] >= -500) && (adcs_est_ang_rates.angular_Z_rate[0] <= 500))

							Detumbling_EKF_mode_count += 1;

						else
						{
							Detumbling_EKF_mode_flag = 0;
							Detumbling_EKF_mode_count = 0;
						}
					else
					{
						Detumbling_EKF_mode_flag = 0;
						Detumbling_EKF_mode_count = 0;
					}
				else
				{
					Detumbling_EKF_mode_flag = 0;
					Detumbling_EKF_mode_count = 0;
				}

			}
			else
			{
				Detumbling_EKF_mode_flag = 0;
				Detumbling_EKF_mode_count = 0;
			}

		}


		if (Detumbling_EKF_mode_count  >= Detumbling_EKF_mode_qualified_samples)
		{
			adcs_status.detumbling_rkf_flag = 0;
			adcs_status.detumbling_ekf_flag = 0;
			adcs_status.y_momentum_intial_flag = 1;

			printf("u jump to Y_momentum_initial_state\n");
			break;
		}
		else
		{
			adcs_status.detumbling_rkf_flag = 0;
			adcs_status.detumbling_ekf_flag = 1;
			adcs_status.y_momentum_intial_flag = 0;

			printf("u stay in Detumbling Control with EKF\n");
		}

		vTaskDelay(adcs_state_delay * delay_time_based);
		adcs_est_ang_rates.angular_X_rate[0] = adcs_est_ang_rates.angular_X_rate[1];
		adcs_est_ang_rates.angular_Y_rate[0] = adcs_est_ang_rates.angular_Y_rate[1];
		adcs_est_ang_rates.angular_Z_rate[0] = adcs_est_ang_rates.angular_Z_rate[1];
		Angular_Y_rate_difference[0] = Angular_Y_rate_difference[1];
		adcs_est_rpy_angle.pitch_angle[0] = adcs_est_rpy_angle.pitch_angle[1];
	}
}

void Y_Momontum_Stabilized_Initial()
{
	/*---------------------------Enable the ADCS to run mode-------------------------------*/
	txbuf[0] = 0x03;   //0d03 ADCS run mode
	txbuf[1] = 0x01;
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 2, &rxbuf, 0, adcs_delay) == E_NO_ERR)
		printf("ID:3\tSet ADCS run mode into enabled\n");

	/*---------------------------set the power control-------------------------------------*/
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
	txbuf[1] = 0x03;   //Set to high initial rate detumbling control
	txbuf[2] = 0x01;   //Default
	txbuf[3] = 0x00;   //Default
	txbuf[4] = 0x00;   //Default
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 5, &rxbuf, 0, adcs_delay) == E_NO_ERR)
		printf("ID:18\tSet control mode into y-momentum stabilized - initial\n");

	/*----------------------------Set the estimation mode----------------------------------*/
	txbuf[0] = 0x11;   //0d17 Set attitude estimation mode
	txbuf[1] = 0x04;   // Set to extended kalman filter mode
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 2, &rxbuf, 0, adcs_delay) == E_NO_ERR)
		printf("ID:17\tSet estimation mode into Extended Kalman Filter\n");

	/*---------------------------Triggering detecting--------------------------------------*/
	int8_t ADCS_current_state = 0;
	bool steady_state_flag;
	int Steady_state_restart_count = 0;
	int Steady_state_restart_flag = 0;
	int Steady_state_restart_qualified_samples = 5;
	while (1)
	{

		txbuf[0] = 0x90;   //0d144 read the current status of ADCS
		if (i2c_master_transaction_2(0, stm_node, &txbuf, 1, &rxbuf, 6, adcs_delay) == E_NO_ERR)
		{
			ADCS_current_state = rxbuf[0]; //   *256 = <<8, /256= >>8
		}

		steady_state_flag = (int8_t)((ADCS_current_state << 0 ) < 0); //The "int8_t" has 8 bits, the 0th represents the status of Y-momentum steady-state

		printf("%2x\n", ADCS_current_state);
		printf("%d\n", steady_state_flag);

		if (steady_state_flag == 1)
		{
			//vTaskDelay(10 * delay_time_based);   //Jerry test
			printf("Enter Y_Momontum_Stabilized_Steady_State\n");


			txbuf[0] = 0x92;   //0d146 Estimated angular rates
			if (i2c_master_transaction_2(0, stm_node, &txbuf, 1, &rxbuf, 6, adcs_delay) == E_NO_ERR)
			{
				adcs_est_ang_rates.angular_X_rate[1] = rxbuf[0] + (rxbuf[1] << 8); //   *256 = <<8, /256= >>8
				adcs_est_ang_rates.angular_Y_rate[1] = rxbuf[2] + (rxbuf[3] << 8); //   *256 = <<8, /256= >>8
				adcs_est_ang_rates.angular_Z_rate[1] = rxbuf[4] + (rxbuf[5] << 8); //   *256 = <<8, /256= >>8
			}

			if (Steady_state_restart_flag == 0)
			{
				if ((adcs_est_ang_rates.angular_X_rate[1] <= -1000) || (adcs_est_ang_rates.angular_X_rate[1] >= 1000)
				        || (adcs_est_ang_rates.angular_Y_rate[1] <= -1000) || (adcs_est_ang_rates.angular_Y_rate[1] >= 1000)
				        || (adcs_est_ang_rates.angular_Z_rate[1] <= -1000) || (adcs_est_ang_rates.angular_Z_rate[1] >= 1000))
				{
					Steady_state_restart_count = 0;
					Steady_state_restart_count += 1;
					Steady_state_restart_flag = 1;
				}
				else
				{
					Steady_state_restart_flag = 0;
				}
			}
			else
			{
				if (Steady_state_restart_count != 0 )
				{
					if ((adcs_est_ang_rates.angular_X_rate[1] <= -1000) || (adcs_est_ang_rates.angular_X_rate[1] >= 1000)
					        || (adcs_est_ang_rates.angular_Y_rate[1] <= -1000) || (adcs_est_ang_rates.angular_Y_rate[1] >= 1000)
					        || (adcs_est_ang_rates.angular_Z_rate[1] <= -1000) || (adcs_est_ang_rates.angular_Z_rate[1] >= 1000))
					{
						if ((adcs_est_ang_rates.angular_X_rate[0] >= -1000) || (adcs_est_ang_rates.angular_X_rate[0] >= 1000)
						        || (adcs_est_ang_rates.angular_Y_rate[0] >= -1000) || (adcs_est_ang_rates.angular_Y_rate[0] >= 1000)
						        || (adcs_est_ang_rates.angular_Z_rate[0] >= -1000) || (adcs_est_ang_rates.angular_Z_rate[0] >= 1000))
						{
							Steady_state_restart_count += 1;
						}
						else
						{
							Steady_state_restart_flag = 0;
							Steady_state_restart_count = 0;
						}
					}
					else
					{
						Steady_state_restart_flag = 0;
						Steady_state_restart_count = 0;
					}
				}
				else
				{
					Steady_state_restart_flag = 0;
					Steady_state_restart_count = 0;
				}

			}

			if (Steady_state_restart_count  >= Steady_state_restart_qualified_samples)
			{
				adcs_status.initial_flag = 1;
				adcs_status.high_initial_flag1 = 0;
				adcs_status.high_initial_flag2 = 0;

				adcs_process.initialize_flag = 1;
				adcs_process.stabilize_flag = 0;
				printf("The satellite is really unstable\n");
				printf("u restart from initialize process\n");
				break;
			}
			else
			{
				if (magnetometer_deploy == 1)
				{
					printf("Start Magnetometer Deploy!\n");

					adcs_status.detumbling_rkf_flag = 0;
					adcs_status.detumbling_ekf_flag = 0;
					adcs_status.y_momentum_intial_flag = 0;
					break;
				}
				else
				{
					printf("U Stay in Y_Momontum_Stabilized_Steady_State\n");
				}

			}
		}
		else
		{
			printf("U Stay in Y_Momontum_Stabilized_Initial_State\n");
		}
		vTaskDelay(adcs_state_delay * delay_time_based);
		adcs_est_ang_rates.angular_X_rate[0] = adcs_est_ang_rates.angular_X_rate[1];
		adcs_est_ang_rates.angular_Y_rate[0] = adcs_est_ang_rates.angular_Y_rate[1];
		adcs_est_ang_rates.angular_Z_rate[0] = adcs_est_ang_rates.angular_Z_rate[1];
	}
}


void Magnetometer_Calibration_Setup()
{
	txbuf[0]  = 0x56;  //0d86 Set magnetometer configuration
	txbuf[1]  = 0x46;  //mounting alpha vina:00B4/180
	txbuf[2]  = 0x50;
	txbuf[3]  = 0x46;  //mounting beta vina:00b4/180
	txbuf[4]  = 0x50;
	txbuf[5]  = 0xD8;  //mounting gamma  vina:ffa6/-90
	txbuf[6]  = 0xDC;
	txbuf[7]  = 0xB5;  //offset 1
	txbuf[8]  = 0xFE;
	txbuf[9]  = 0x32;  //offset 2
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

	if (ground_Test_Mode == 0) //Save the configuration setting to the flash memory
	{
		txbuf[0] = 0x64;
		if (i2c_master_transaction_2(0, adcs_node, &txbuf, 1, &rxbuf, 0, adcs_delay) == E_NO_ERR)
			printf("ID:100\tSave current configuration to flash memory\n");
	}
}

void Magnetometer_Deployment_Process()
{
	/*---------------------------Enable the ADCS to run mode-------------------------------*/
	txbuf[0] = 0x03;   //0d03 ADCS run mode
	txbuf[1] = 0x01;
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 2, &rxbuf, 0, adcs_delay) == E_NO_ERR)
		printf("ID:3\tSet ADCS run mode into enabled\n");

	/*---------------------------set the power control-------------------------------------*/
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
	txbuf[1] = 0x00;   //Set to no control
	txbuf[2] = 0x00;   //set override flag disable
	txbuf[3] = 0x00;   //set control time out to infinite
	txbuf[4] = 0x00;   //
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 5, &rxbuf, 0, adcs_delay) == E_NO_ERR)
		printf("ID:18\tSet control mode into no control mode\n");

	/*----------------------------Set the estimation mode----------------------------------*/
	txbuf[0] = 0x11;   //0d17 Set attitude estimation mode
	txbuf[1] = 0x00;   // Set to extended kalman filter mode
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 2, &rxbuf, 0, adcs_delay) == E_NO_ERR)
		printf("ID:17\tSet estimation mode into no estimator mode\n");

	/*----------------------------Start to deploy the magnetometer--------------------------------*/
	int mag_deploy_status_flag = 0;  // 0: Haven't 1: Successfully 2: Unsuccessfully (Already deployed)
	int mag_deploy_check_count = 0;
	int mag_deploy_time_count = 0;
	uint8_t CSS4_value = 0;
	int CSS4_threshold_value = 	100;
	while (adcs_process.start_mag_deploy_flag == 1)
	{
		while (mag_deploy_status_flag == 0)
		{
			txbuf[0] = 0xA6;   //0d166 Raw CSS,CSS4 is below magnetometer
			if (i2c_master_transaction_2(0, stm_node, &txbuf, 1, &rxbuf, 6, adcs_delay) == E_NO_ERR)
			{
				CSS4_value = rxbuf[3];
			}

			vTaskDelay(0.1 * delay_time_based);
			if (CSS4_value < CSS4_threshold_value)
			{
				txbuf[0] = 0xA7;   //0d167 RAW magnetic
				if (i2c_master_transaction_2(0, stm_node, &txbuf, 1, &rxbuf, 6, adcs_delay) == E_NO_ERR)
				{
					adcs_mag_raw_measurement.mag_raw_measurement_X[0] = rxbuf[0] + (rxbuf[1] << 8); //   *256 = <<8, /256= >>8, display in LSB
					adcs_mag_raw_measurement.mag_raw_measurement_Y[0] = rxbuf[2] + (rxbuf[3] << 8); //   *256 = <<8, /256= >>8, display in LSB
					adcs_mag_raw_measurement.mag_raw_measurement_Z[0] = rxbuf[4] + (rxbuf[5] << 8); //   *256 = <<8, /256= >>8, display in LSB
				}
			}

			printf("CSS4 measurement= %d\n", CSS4_value);

			if (CSS4_value > CSS4_threshold_value)
			{
				// Verify the deployment by checking the raw value of magnetometer measurement
				// New_X_value =   Previous_Y_value
				// New_Y_value = -(Previous_X_value)
				// New_Z_value =   Previous_Z_value

				vTaskDelay(1 * delay_time_based); // Wait for the magnetometer deployment

				txbuf[0] = 0xA7;   //0d167 RAW magnetic
				if (i2c_master_transaction_2(0, stm_node, &txbuf, 1, &rxbuf, 6, adcs_delay) == E_NO_ERR)
				{
					adcs_mag_raw_measurement.mag_raw_measurement_X[1] = rxbuf[0] + (rxbuf[1] << 8); //   *256 = <<8, /256= >>8, display in LSB
					adcs_mag_raw_measurement.mag_raw_measurement_Y[1] = rxbuf[2] + (rxbuf[3] << 8); //   *256 = <<8, /256= >>8, display in LSB
					adcs_mag_raw_measurement.mag_raw_measurement_Z[1] = rxbuf[4] + (rxbuf[5] << 8); //   *256 = <<8, /256= >>8, display in LSB
				}

				printf("\t(Previous)\n");
				printf("\tRaw X Magnetic Measurement = %d", adcs_mag_raw_measurement.mag_raw_measurement_X[0]);
				printf("\tRaw Y Magnetic Measurement = %d", adcs_mag_raw_measurement.mag_raw_measurement_Y[0]);
				printf("\tRaw Z Magnetic Measurement = %d\n", adcs_mag_raw_measurement.mag_raw_measurement_Z[0]);

				printf("\t(Latest)\n");
				printf("\tRaw X Magnetic Measurement = %d", adcs_mag_raw_measurement.mag_raw_measurement_X[1]);
				printf("\tRaw Y Magnetic Measurement = %d", adcs_mag_raw_measurement.mag_raw_measurement_Y[1]);
				printf("\tRaw Z Magnetic Measurement = %d\n", adcs_mag_raw_measurement.mag_raw_measurement_Z[1]);

				if ((abs(adcs_mag_raw_measurement.mag_raw_measurement_X[1] - adcs_mag_raw_measurement.mag_raw_measurement_Y[0]) < 2000)
				        && (abs(adcs_mag_raw_measurement.mag_raw_measurement_Y[1] + adcs_mag_raw_measurement.mag_raw_measurement_X[0]) < 2000)
				        && (abs(adcs_mag_raw_measurement.mag_raw_measurement_Z[1] - adcs_mag_raw_measurement.mag_raw_measurement_Z[0]) < 2000))
				{
					Magnetometer_Calibration_Setup();
					vTaskDelay(1 * delay_time_based);
					mag_deploy_status_flag = 1;
					/*--------------------Maybe add something-----------------------*/
					printf("The magnetometer configuration had been succesfully calibrated and verified\n");
					break;
				}
				else
				{
					mag_deploy_status_flag = 2;
					vTaskDelay(1 * delay_time_based);
				}
			}

			if (mag_deploy_check_count > 10)
			{
				//Send deploy command again
				mag_deploy_check_count = 0;
				mag_deploy_time_count += 1;
			}
			if (mag_deploy_time_count > 2)
			{
				mag_deploy_status_flag = 2;
			}
			mag_deploy_check_count += 1;
			vTaskDelay(adcs_state_delay * delay_time_based);
		}

		if (mag_deploy_status_flag == 1)
		{
			adcs_status.initial_flag = 1;
			adcs_status.high_initial_flag1 = 0;
			adcs_status.high_initial_flag2 = 0;

			adcs_process.initialize_flag = 1;
			adcs_process.stabilize_flag = 0;
			printf("u restart from initialize process\n");
			break;
		}
		vTaskDelay(adcs_state_delay * delay_time_based);
	}
}


void Initialize_Process()
{
	adcs_status.initial_flag = 1;
	adcs_status.high_initial_flag1 = 0;
	adcs_status.high_initial_flag2 = 0;

	while (1)
	{
		if ((adcs_status.initial_flag == 1) && (adcs_status.high_initial_flag1 == 0) && (adcs_status.high_initial_flag2 == 0))
		{
			Initial_Mode();
			//break;
		}
		else if ((adcs_status.initial_flag == 0) && (adcs_status.high_initial_flag1 == 1) && (adcs_status.high_initial_flag2 == 0))
		{
			High_Initial_Rate_Detumbling(1);
			//break;
		}
		else if ((adcs_status.initial_flag == 0) && (adcs_status.high_initial_flag1 == 0) && (adcs_status.high_initial_flag2 == 1))
		{
			High_Initial_Rate_Detumbling(2);
			//break;
		}
		else
		{
			break;
		}
	}
}

void Stabilize_Process()
{
	adcs_status.detumbling_rkf_flag = 1;
	adcs_status.detumbling_ekf_flag = 0;
	adcs_status.y_momentum_intial_flag = 0;

	/*Comment the line below will make the progress start from Y-momentum mode*/
	// adcs_status.detumbling_rkf_flag = 0;
	// adcs_status.detumbling_ekf_flag = 0;
	// adcs_status.y_momentum_intial_flag = 1;

	while (1)
	{
		if ((adcs_status.detumbling_rkf_flag == 1) && (adcs_status.detumbling_ekf_flag == 0) && (adcs_status.y_momentum_intial_flag == 0))
		{
			Detumbling_Control_RKF();
			//break;
		}
		else if ((adcs_status.detumbling_rkf_flag == 0) && (adcs_status.detumbling_ekf_flag == 1) && (adcs_status.y_momentum_intial_flag == 0))
		{
			Detumbling_Control_EKF();
			//break;
		}
		else if ((adcs_status.detumbling_rkf_flag == 0) && (adcs_status.detumbling_ekf_flag == 0) && (adcs_status.y_momentum_intial_flag == 1))
		{
			Y_Momontum_Stabilized_Initial();
			//break;
		}
		else
		{
			break;
		}
	}
}


//The ADCS_Tasks should be launched after powering on the ADCS bundle

void ADCS_Task(void * pvParameters)
{

	vTaskDelay(10 * delay_time_based);   //Delay 15s to start ADCS
	printf("Press Stm reset button!\n"); //Set up the stm for loading pseudo data
	vTaskDelay(5 * delay_time_based);    //Delay 15s to start ADCS

	adcs_process.initialize_flag = 1;
	adcs_process.stabilize_flag = 0;
	adcs_process.start_mag_deploy_flag = 0;

	/*Comment the line below will make the progress start from Y-momentum mode*/
	// adcs_process.initialize_flag = 0;
	// adcs_process.stabilize_flag = 1;
	// adcs_process.start_mag_deploy_flag = 0;

	while (1)
	{

		if ((adcs_process.initialize_flag == 1) && (adcs_process.stabilize_flag == 0))
		{
			Initialize_Process();
			//break;
		}

		else if ((adcs_process.initialize_flag == 0) && (adcs_process.stabilize_flag == 1))
		{

			if ((magnetometer_deploy == 1) && (adcs_process.start_mag_deploy_flag == 0))
			{
				adcs_process.start_mag_deploy_flag = 1;
				Magnetometer_Deployment_Process();
				printf("End of deployment");
			}
			else
			{
				Stabilize_Process();
			}
		}
		/** End of ADCS TASK, Should never reach this line  */
	}
	vTaskDelete(NULL);
}
