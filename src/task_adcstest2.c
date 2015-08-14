/*
 * task_ADCSpressure.c
 *
 *  Created on: 	2015/04/30
 *  Last updated: 	2015/05/04
 *      Author: Kenny Huang
 */
/**
 * this task is for testing I2C communication between ADCS and OBDH
 */
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <time.h>
#include <math.h>

#include <fat_sd/ff.h>
#include "fs.h"
#include <dev/i2c.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <util/error.h>
#include <util/hexdump.h>
#include <util/csp_buffer.h>
#include <util/log.h>
#include <util/driver_debug.h>
#include <util/delay.h>
#include <dev/cpu.h>

#include <dev/usart.h>
#include <io/nanopower2.h>
#include <io/nanomind.h>
#include <csp/csp.h>
#include <csp/csp_endian.h>

#include "parameter.h"
#include "subsystem.h"
#include "mpio.h"
 #include <io/nanopower2.h>
void vADCS(void * pvParameters){
	unsigned int outputs=0x0b;
	eps_output_set((uint8_t) outputs);
	uint8_t txbuf[255]; 
	uint8_t ucharAdcs[50];
	int delay_time = 0.3;
	int testNum ;
	int addr = 87;
	vTaskDelay(5*1000);	
	int errorOccur =0;
	while(errorOccur!=1){
		errorOccur =0;
		// testNum = 1 +rand()%5;
		testNum = 1 ;
		switch(testNum){
			case 1:
				txbuf[0] = 0x80;	//0d128

				if(i2c_master_transaction(0,addr, &txbuf,1,&ucharAdcs,8,20) == E_NO_ERR){
					printf("ID: 128\tIdentification information222222222222222222222\n");
					hex_dump(ucharAdcs,8);
				}
				else{
					errorOccur=1;
					printf("ADCS rx error!!!!!222222222222222222222\n");
				}
				// else
				// 	break;
				if(ucharAdcs[0]!=0x0A){
					printf("ucahr  = %02x \n",ucharAdcs[0] );
					errorOccur=1;
					printf("FLIP ERRORS detected on ADCS !!!222222222222222222!!!\n");
					break;
				}
				printf("-------adcs frame-------222222222222222222222\n");
				// if(i2c_master_transaction(0,127, &txbuf,1,&ucharAdcs,0,5) == E_NO_ERR){
				// 	printf("ID\n");
				// 	// hex_dump(ucharAdcs,8);
				// }
				vTaskDelay(delay_time*1000);	
				break;
			case 2:    
				txbuf[0] = 0x02;	//0d02
				txbuf[1] = 0x80;
				txbuf[2] = 0x3e;
				txbuf[3] = 0xca;
				txbuf[4] = 0xd0;
				txbuf[5] = 0x00;
				txbuf[6] = 0x00;
				if(i2c_master_transaction(0,addr, &txbuf,7,&ucharAdcs,0,7) == E_NO_ERR){
					printf("ID: 02\tSet current unix time\n");
				}
				else
					errorOccur=1;
				txbuf[6]++;
				if(txbuf[6]==0xff)
					txbuf[6]=0x00;
				vTaskDelay(delay_time*1000);
				break;
			case 3:
				txbuf[0] = 0x11;	//0d17
				txbuf[1] = 0x01;
				
				
				if(i2c_master_transaction(0,addr, &txbuf,2,&ucharAdcs,0,7) == E_NO_ERR){
					printf("ID: 17\tSet attitude estimation mode\n");
				}
				else
					errorOccur=1;
				txbuf[1]++;
				if(txbuf[1]==0x05)
					txbuf[1]=0x00;
				vTaskDelay(delay_time*1000);	
				break;
			// case 4:
			// 	txbuf[0] = 0x03;	//0d03
			// 	txbuf[1] = 0x01;
								
			// 	if(i2c_master_transaction(0,82, &txbuf,2,&ucharAdcs,0,5) == E_NO_ERR){
			// 		printf("ID: 3\tSet ADCS Run Mode = [enable]\n");
			// 	}
			// 	txbuf[0] = 0x05;	//0d05
			// 	txbuf[1] = 0x00;
			// 	txbuf[2] = 0x01;	
			// 	txbuf[3] = 0x00;
			// 	txbuf[4] = 0x01;
			// 	txbuf[5] = 0x00;	
			// 	if(i2c_master_transaction(0,82, &txbuf,6,&ucharAdcs,0,5) == E_NO_ERR){
			// 		printf("ID: 5\tTurn on Cubecontrol Motor, Motor Driver\n");
			// 	}
			// 	txbuf[0] = 0x20;	//0d32
			// 	txbuf[1] = 0x00;
			// 	txbuf[2] = 0x00;	
			// 	txbuf[3] = 0xE8;
			// 	txbuf[4] = 0x03;
			// 	txbuf[5] = 0x00;
			// 	txbuf[6] = 0x00;	
			// 	if(i2c_master_transaction(0,82, &txbuf,7,&ucharAdcs,0,5) == E_NO_ERR){
			// 		printf("ID: 32\tSet Wheel Speed to 1000\n");
			// 	}
			// 	txbuf[0] = 0x9A;	//0d03
			// 	while (!((ucharAdcs[2] + (ucharAdcs[3]<<8) )>=980)){
			// 		if(i2c_master_transaction(0,82, &txbuf,1,&ucharAdcs,6,5) == E_NO_ERR){
			// 			hex_dump(ucharAdcs,6);
			// 			printf("wheel speed= %d\n",ucharAdcs[2] + (ucharAdcs[3]<<8) );
			// 			printf("\E[2A\r");
			// 			vTaskDelay(1000);
			// 		}
			// 	}
			// 	txbuf[0] = 0x20;	//0d32
			// 	txbuf[1] = 0x00;
			// 	txbuf[2] = 0x00;	
			// 	txbuf[3] = 0x00;
			// 	txbuf[4] = 0x00;
			// 	txbuf[5] = 0x00;
			// 	txbuf[6] = 0x00;	
			// 	if(i2c_master_transaction(0,82, &txbuf,7,&ucharAdcs,0,5) == E_NO_ERR){
			// 		printf("ID: 32\tSet Wheel Speed to 0\n");
			// 	}		
			// 	vTaskDelay(delay_time*1000);
			// 	break;
			// 	
			case 4:
				txbuf[0] = 0x1A;	//0d26
				if(i2c_master_transaction(0,80, &txbuf,1,&ucharAdcs,16,7) == E_NO_ERR){
					printf("COM HK\n");
					hex_dump(ucharAdcs,16);
				}
				else
					errorOccur=1;	
				vTaskDelay(delay_time*1000);
				break;
			case 5:
				txbuf[0] = 0x90;	//0d144
				if(i2c_master_transaction(0,109, &txbuf,1,&ucharAdcs,4,7) == E_NO_ERR){
					printf("Interface Board 3V3 current\n");
					hex_dump(ucharAdcs,4);
				}
				else
					errorOccur=1;	
				vTaskDelay(delay_time*1000);
				break;
			case 6:
				txbuf[0] = 0x90;	//0d144
				if(i2c_master_transaction(0,110, &txbuf,0,&ucharAdcs,5,7) == E_NO_ERR){
					printf("SEUV CH1 rx\n");
					hex_dump(ucharAdcs,5);
				}
				else
					errorOccur=1;	
				vTaskDelay(delay_time*1000);
				break;
			default:
				break;
		}	
		txbuf[0] = 0x08;	//0d32


		if(i2c_master_transaction(0,2, &txbuf,1,&ucharAdcs,45,20) == E_NO_ERR){
			printf("EPS HK\n");
			hex_dump(ucharAdcs,45);
		}
		else{
			errorOccur=1;
			printf("EPSrx error!!!!! 2222222222222222222222\n");
		}

		// else
		// 	break;
		vTaskDelay(delay_time*1000);	
	}	
	vTaskDelete(NULL);	
}