/*
 * task_INMS.c
 *
 *  Created on: 	2014/10/06
 *  Last updated: 	2015/04/06
 *      Author: Kenny Huang
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
#include <nanomind.h>
#include <csp/csp.h>
#include <csp/csp_endian.h>

#include "parameter.h"
#include "subsystem.h"

#define scriptNum 			7
#define isSimulator 		0
#define delay_time_based	configTICK_RATE_HZ

typedef struct __attribute__((packed)) {
	int tt_hour, tt_min, tt_sec, tt_seq;
} timetable;

int rec[scriptNum];

int scriptRunning = 0;		/* initializing for identifying which script is running */
int obcSuErrFlag = 0;		/* for the detection of the OBC_SU_ERR */
int maxlength = 0;
uint32_t epoch_sec[scriptNum];/* the seconds from UTC epoch time 2000/01/01 00:00:00 Am */
uint32_t refsec[scriptNum] ;

/**
 * This function is used for sorting the script time
 * @param
 */
void bubblesort(uint32_t refsec[]) {
	int i, c, d;
	int j = 0;
	uint32_t swap;
	uint32_t aftsec[scriptNum];
	for (i = 0; i < scriptNum; i++) {
		rec[i] = 0;
		aftsec[i] = refsec[i];
	}
	//	for(c=0;c<(scriptNum-1);c++){
	//		for(d=0;d< (scriptNum-c-1);d++){
	//			if(refsec[d] > refsec[d+1]){
	//				swap = refsec[d];
	//				refsec[d] = refsec[d+1];
	//				refsec[d+1] = swap;
	//			}
	//		}
	//	}
	for (c = 0; c < (scriptNum - 1); c++) {
		for (d = 0; d < (scriptNum - c - 1); d++) {
			if (aftsec[d] > aftsec[d + 1]) {
				swap = aftsec[d];
				aftsec[d] = aftsec[d + 1];
				aftsec[d + 1] = swap;
			}
		}
	}
	for (i = 0; i < scriptNum; i++) {
		for (j = 0; j < scriptNum; j++) {
			if (refsec[i] == aftsec[j]) {
				rec[j] = i;
			}
		}
	}
}
void inmssort(uint32_t *sec) {


	timestamp_t t;
	t.tv_sec = 0;
	t.tv_nsec = 0;
	obc_timesync(&t, 6000);
	//time_t tt = t.tv_sec;
	//printf("t.tv_sec = %d\n",t.tv_sec);
	for (int i = 0; i < scriptNum; i++) {
		refsec[i] = 0;
	}
	for (int i = 0; i < scriptNum; i++) {
		printf("sec[%d] = %" PRIu32 "\n", i, sec[i]);
	}
	printf("t.tv_sec = %" PRIu32 "\n", t.tv_sec);
	for (int i = 0; i < scriptNum; i++) {
		refsec[i] = (sec[i] - t.tv_sec);
		printf("refsec[%d] = %" PRIu32 "\n", i, refsec[i]);
	}


	bubblesort(refsec);
}
/*FLETCHER-16  checksum based on REQ: INMS-I-127*/
uint16_t fletcher(uint8_t *script, size_t length) {
	
	/* Another way of implementing Flectcher-16 checksum*/
	// uint16_t sum1 = 0xff, sum2 = 0xff;

 //    while (length) {
 //            size_t tlen = length > 20 ? 20 : length;
 //            length -= tlen;
 //            do {
 //                sum2 += sum1 += *script++;
 //            } while (--tlen);
 //            sum1 = (sum1 & 0xff) + (sum1 >> 8);
 //            sum2 = (sum2 & 0xff) + (sum2 >> 8);
 //    }
 //    /* Second reduction step to reduce sums to 8 bits */
 //    sum1 = (sum1 & 0xff) + (sum1 >> 8);
 //    sum2 = (sum2 & 0xff) + (sum2 >> 8);
 //    printf("%" PRIu16 "\n", sum2 << 8 | sum1 ); 
 //    return sum2 << 8 | sum1;


	uint16_t C0_int = 0, C1_int = 0;
	uint16_t XSUM_W = 0xFFFF;	//0xFFFF = 0d65535
	for (unsigned int i = 0; i < length; i++) {
		C0_int = C0_int + script[i];
		C1_int = C1_int + C0_int;
		C0_int = (C0_int) % (255);
		C1_int = (C1_int) % (255);
	}
	XSUM_W = (C0_int) || (C1_int << 8);
	printf("sum = %d\n", XSUM_W);
	return XSUM_W;
}
/*Return the system time with big clock(Real seconds) or small clock(86400)*/
uint32_t timeGet (int clockType) {
	uint32_t onBoardTime = 0;
	timestamp_t t;
	t.tv_sec = 0;
	t.tv_nsec = 0;
	obc_timesync(&t, 1000);

	onBoardTime = (clockType == 0) ? ( t.tv_sec) : ( t.tv_sec % 86400);
	return onBoardTime;
}

int inmsJumpScriptCheck (int currentScript) {
	/*After finish each command, perform this check to see if the next script is coming */
	uint32_t timeRef ;
	timeRef = timeGet(0);

	// Scan next Script 
	if (currentScript != 6)
		currentScript++;
	// 
	printf("\t\t\t\ttimer =  %" PRIu32 "\n", timeRef);
	printf("\t\t\t\tepoch =  %" PRIu32 "\n", epoch_sec[rec[currentScript]]);
	printf("\E[2A\r");

	if (timeRef > epoch_sec[rec[currentScript]] - 10) {
		return 1;
	}
	else
		return 0;
}


/**
 * This task is used for receiving the packet from INMS, should check the Rx buffer every one seconds
 * @param
 */
void vTaskInmsReceive(void * pvParameters) {
	int numReceive = 0;
	char ucharAdcs[22];
	char ucharTotal[174 + 22];		//response packet is FIXED 174 BYTES+ADCS 22 BYTES

	int receiveFlag = 0;
	uint8_t rxbuf[48];
	uint8_t txbuf = 0x88; 			// ID = 136 Current ADCS state
	timestamp_t t;

	for (int k = 0; k < 22; k++) {
		ucharAdcs[k] = 0;
	}
	for (int k = 0; k < 174 + 22; k++) {
		ucharTotal[k] = 0;
	}
	portTickType xLastWakeTime;
	const portTickType xFrequency = 1500;

	while (1) {
		/*response_pkt*/
		// if (parameters.inms_function == 0)
		// vTaskDelete(NULL);	
		xLastWakeTime = xTaskGetTickCount();
		receiveFlag++;
		/*
			INMS requirement generate packet within 400 seconds
		 */
		if (receiveFlag >= 400) {
			obcSuErrFlag = 1;
		}
		numReceive = usart_messages_waiting(2);
		if (numReceive != 0) {
			printf("numReceive = %d\n", numReceive);
		}
		int ADCSexist = 0;
		if ( numReceive  != 0) {
			printf("Get %d response packet!\n", numReceive + 22);
			if (ADCSexist == 1) {
				t.tv_sec = 0;
				t.tv_nsec = 0;
				obc_timesync(&t, 6000);
				memcpy(&ucharAdcs[0], &t.tv_sec, 4);
				if (i2c_master_transaction(0, adcs_node, &txbuf, 1, &rxbuf, 48, adcs_delay) == E_NO_ERR) {
					memcpy(&ucharAdcs[4], &rxbuf[18], 18);
					printf("Get Time, Attitude, Position from ADCS");
				}
				else {
					printf("Get data from ADCS FAILED \r\n");
				}
				for (int i = 0; i < 22; i++) {
					ucharTotal[i] = ucharAdcs[i];
				}
				for (int i = 22; i < 196; i++) {
					ucharTotal[i] = usart_getc(2);
				}
			}
			else {
				for (int i = 0; i < 22; i++) {
					ucharTotal[i] = 0xFF;
				}
				for (int i = 22; i < 196; i++) {
					ucharTotal[i] = usart_getc(2);
				}
			}

			hex_dump(ucharTotal, numReceive + 22);
			// if(ucharTotal[22]==0xBB); 	//SU_ERR detected!
			// 	obcSuErrFlag = 5;
			//inms_data_write((uint8_t *)ucharTotal);
			numReceive = 0;
			/* ---------------------    */
			receiveFlag = 0;
		}
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
	}
}


void vTaskinms(void * pvParameters) {
	
	printf("Start INMS task......\n");
	vTaskDelay(5 * delay_time_based);
	while (1) {
		vTaskDelay(3 * delay_time_based);
		if (inms_status == 1) { 
			// printf(" %d \n", parameters.inms_function);
			//  if(parameters.inms_function==0)
			// vTaskDelete(NULL);
			int len[scriptNum];	//the length of each script
			for (int i = 0; i < scriptNum; i++) {
				epoch_sec[i] = 0;
			}
			printf("[-------------STEP #1 : Getting length of all scripts-------------]\n");
			/* STEP #1 Get the length of all of the scripts*/
			for (int i = 0; i < scriptNum; i++) {
				printf("script %d: ", i);
				len[i] = inms_script_length(i);
				if (len[i] >= maxlength) {
					maxlength = len[i];
				}
			}
			printf("MAX length : %d\n", maxlength);
			uint8_t script[scriptNum][maxlength];           //(option) Use pointer  check test/jagged array

			printf("[-------------STEP #2a : Performing Fletcher 16-------------]\n");
			/**
			 * STEP #2 Read the script, put the data into script[]
			 * and perform Fletcher-16 checksum and additional check
			 */
			for (int i = 0; i < scriptNum; i++) {
				inms_script_read(i, len[i], &script[i]);
				uint16_t xsum = fletcher(script[i], len[i]);
				if (xsum == 0) {
					printf("No. %d script XSUM through Fletcher-16 [PASS]\n", i);
				}
				else {
					printf("No. %d script XSUM through Fletcher-16 [FAIL]\n", i);
					obcSuErrFlag = 6;
				}
			}
			printf("[-------------STEP #2b: Performing additional check with script length-------------]\n");
			for (int i = 0; i < scriptNum; i++) {
				if (len[i] == script[i][0]) {
					printf("No. %d script length check [PASS]\n", i);
				}
				else {
					printf("No. %d script length check [FAIL]\n", i);
					obcSuErrFlag = 2;
				}
			}

			printf("[-------------STEP #3 : Converting 4 bytes times into seconds-------------]\n");
			/*STEP #3 Convert the 4 byte time stamps into seconds since UTC epoch ;*/
			for (int i = 0; i < scriptNum; i++) {
				epoch_sec[i] = (script[i][2]) + (script[i][3] << 8) + (script[i][4] << 16) + (script[i][5] << 24);
	//			printf("%d = %" PRIu32 "\n", i, epoch_sec[i]);
			}

			/**
			 * Sort these 7 epoch times by calling the function inmssort
			 */
			inmssort(epoch_sec);
			for (int i = 0; i < scriptNum; i++) {
				printf("[%d] => %d\n", i, rec[i]);
			}

			vTaskDelay(2 * delay_time_based);

			/* for temporary setting,  now only have idle0.bin, idle1.bin*/
			if (isSimulator) {
				rec[0] = 1;
				rec[1] = 0;
			}
			/*                                                                                                 */
			// rec[0] = 0;
			// rec[1] = 3;
			// rec[2] = 4;
			printf("[-------------STEP #4 : Recording TimeTable with structure-------------]\n");
			/*STEP #4 Record the TimeTable with structure*/
			for (int i = 0; i < scriptNum; i++) {
				if (inms_status == 0)
					break;
				/*Initialize*/
				int flag = 0;		//record which byte is running now
				int ttflag = 0;		//times_table flag
				timetable timetable_t[80]; 		//(option) Use pointer
				flag = 15;
				/*Mark an IDLE-SLOT script buffer as the RUNNING-SCRIPT*/
				scriptRunning =  rec[i];	//for the need to jump to the next script

				while (script[rec[i]][flag] != 0x55) {		//0x55 = EOT
					printf(" cmd= %d\n", script[rec[i]][flag]);
					if (script[rec[i]][flag] == 0x41)	{ 									//S1
						timetable_t[ttflag].tt_hour	=  script[rec[i]][flag - 1];
						timetable_t[ttflag].tt_min	=  script[rec[i]][flag - 2];
						timetable_t[ttflag].tt_sec	=  script[rec[i]][flag - 3];
						timetable_t[ttflag].tt_seq	= 1;
						printf("----------------Sequence 1----------------\n");
					}
					else if (script[rec[i]][flag] == 0x42) {									//S2
						timetable_t[ttflag].tt_hour	=  script[rec[i]][flag - 1];
						timetable_t[ttflag].tt_min	=  script[rec[i]][flag - 2];
						timetable_t[ttflag].tt_sec	=  script[rec[i]][flag - 3];
						timetable_t[ttflag].tt_seq	= 2;
						printf("----------------Sequence 2----------------\n");
					}
					else if (script[rec[i]][flag] == 0x43) {									//S3
						timetable_t[ttflag].tt_hour	=  script[rec[i]][flag - 1];
						timetable_t[ttflag].tt_min	=  script[rec[i]][flag - 2];
						timetable_t[ttflag].tt_sec	=  script[rec[i]][flag - 3];
						timetable_t[ttflag].tt_seq	= 3;
						printf("----------------Sequence 3----------------\n");
					}
					else if (script[rec[i]][flag] == 0x44) {									//S4
						timetable_t[ttflag].tt_hour	=  script[rec[i]][flag - 1];
						timetable_t[ttflag].tt_min	=  script[rec[i]][flag - 2];
						timetable_t[ttflag].tt_sec	=  script[rec[i]][flag - 3];
						timetable_t[ttflag].tt_seq	= 4;
						printf("----------------Sequence 4----------------\n");
					}
					else if (script[rec[i]][flag] == 0x45) {									//S5
						timetable_t[ttflag].tt_hour	=  script[rec[i]][flag - 1];
						timetable_t[ttflag].tt_min	=  script[rec[i]][flag - 2];
						timetable_t[ttflag].tt_sec	=  script[rec[i]][flag - 3];
						timetable_t[ttflag].tt_seq	= 5;
						printf("----------------Sequence 5----------------\n");
					}
					else {
						ttflag--;
					}
					printf("hour:%d, min:%d, sec:%d, seq:%d\n", timetable_t[ttflag].tt_hour, timetable_t[ttflag].tt_min, timetable_t[ttflag].tt_sec, timetable_t[ttflag].tt_seq);
					flag ++;
					ttflag++;
					printf("flag = %d, ttflag = %d\n", flag, ttflag);
					if (flag >= 1000)
						break;
				}
				flag++;
				ttflag--;

				int ttflagMax = ttflag;
				printf("ttflagMax = %d\n", ttflagMax );
				printf("[-------------STEP #5 : Checking the correctness of the sequence-------------]\n");

				/*Check if the first sequence is S1 [Req-INMS-I-228]*/
				if (timetable_t[0].tt_seq == 1) {
					printf("The first sequence is S1!!\n");
				}
				else {
					printf("The first sequence is not S1, ERROR! \n");
					obcSuErrFlag = 5;
				}
				/*
					Check if the Script sequence slots be used sequentially [Req-INMS-I-229]
					need more scenario to test
				*/
				int last_seq = timetable_t[0].tt_seq;
				for (int i = 1; i < ttflag; i++) {
					if (timetable_t[i].tt_seq - last_seq < 2) {
						last_seq = timetable_t[i].tt_seq;
					}
					else {
						printf("The sequence slots are not used sequentially, ERROR! \n");
						obcSuErrFlag = 5;
					}
				}
				printf("[-------------STEP #6 :Setting End of time flag-------------]\n");
				int eotflag [20];  //need prove
				int eotnum = 0;
				int nnflag = flag;
				for (int i = 0; i < 20; i++) {
					eotflag[i] = 0;
				}
				//find where is OBC_EOT 0xFE
				eotflag[eotnum++] = nnflag;
				while (nnflag <= script[rec[i]][0]) {
					if (script[rec[i]][nnflag] == 254) {	//0xFE = 0d254
						eotflag[eotnum] = nnflag + 3;
						eotnum++;
					}
					nnflag ++;
				}
				for (int i = 0; i < eotnum; i++) {
					printf("EOTflag[%d] =%d\n", i, eotflag[i]);
				}

				printf("[-------------STEP #7 : Executing command-------------]\n");
				/*Start execute the command by following the timetable*/
				uint32_t tTable_24 = 0;
				uint32_t refTime = 0;
				ttflag = 0;
				printf("%d", timetable_t[ttflag].tt_seq);
				int printTime = 1;
				int seqcount = 0;
				uint32_t first_time = 0 ;
				if (i == 0){
					while(1){
						if (first_time > epoch_sec[rec[i]]){
							break;
						}
					
						vTaskDelay(1 * delay_time_based);
						first_time = timeGet(0);
						// printf("time = %" PRIu32 "\n",first_time);
						printf("\t\t\t\tdiff = %" PRIu32 "\n", epoch_sec[rec[i]] - first_time);
						printf("\E[1A\r");
					}
				}
				while (1) {
					if (inmsJumpScriptCheck(i))
						break;
					if (inms_status == 0)
						break;
					//printf("virtual clock = %d\n",virtual_clock);  //-----need  test------//
					if (seqcount > ttflagMax) {
						ttflag = 0;
						// break;
					}
					vTaskDelay(1 * delay_time_based);
					refTime = timeGet(1);  //get the small clock time

					tTable_24  = timetable_t[ttflag].tt_hour * 3600 + timetable_t[ttflag].tt_min * 60 +  timetable_t[ttflag].tt_sec;
					if (printTime == 1) {
						printf("%" PRIu32 " -- %" PRIu32 "\n", refTime, tTable_24);
						printf("\E[1A\r");

					}
					if ((refTime  ==  tTable_24 ) || ((refTime - 1 ) ==  tTable_24 )  ) {
						printTime = 0;
						if (timetable_t[ttflag].tt_seq == 1) {
							flag = eotflag[0];
							printf("\n\n\n putting flag 1 done\n");
						}
						else if (timetable_t[ttflag].tt_seq == 2) {
							flag = eotflag[1];
							printf("\n\n\n putting flag 2 done\n");
						}
						else if (timetable_t[ttflag].tt_seq == 3) {
							flag = eotflag[2];
							printf("\n\n\n putting flag 3 done\n");
							// printf("flag = %d\n", flag);
						}
						else if (timetable_t[ttflag].tt_seq == 4) {
							flag = eotflag[3];
							printf("\n\n\n putting flag 4 done\n");
						}
						else if (timetable_t[ttflag].tt_seq == 5) {
							flag = eotflag[4];
							printf("\n\n\n putting flag 5 done\n");
						}
						//printf("putting flag done1\n");
						int leng;
						int delayTimeNow = 0 ;
						int delayTimeTarget = 0;
	//					int numReceive = 0;
						int tempTime = 0;

	//					char uchar[174];			//response packet is FIXED 174 BYTES
						printf("flag = %d, total = %d\n", flag, script[rec[i]][0]);
						while (flag <= script[rec[i]][0]) {
							printf("Into the sequence  loop : Sending command....\n");
							leng = script[rec[i]][flag + 3]; //get Command's length
							printf("\nFlag = %d\n", flag);
							printf("Leng = %d\n", leng);
							printf("Content = %d\n", script[rec[i]][flag + 2]);
	//						if(script[rec[i]][flag+1]*60+script[rec[i]][flag]==1110){
	//							printf("Transfer 1110 to 20s");
	//							vTaskDelay(1000*20);    //delay for testing SU_DUMP
	//						}
	//						else{
	//							vTaskDelay(1000*(script[rec[i]][flag+1]*60+script[rec[i]][flag]));    //delay min+sec
	//						}
							printf("delay: %d\n", script[rec[i]][flag + 1] * 60 + script[rec[i]][flag]);

							if (script[rec[i]][flag + 2] == 241) {   																 //OBC_SU_ON  = 241
								int numGabage = 0;
								power_control(4, ON);	//command EPS to POWER ON INMS
								vTaskDelay(0.5 * delay_time_based);

								numGabage = usart_messages_waiting(2);
								while (numGabage != 0) {
									usart_getc(2);
									numGabage = usart_messages_waiting(2);
								}
								printf("2\n");
								xTaskCreate(vTaskInmsReceive, (const signed char*) "INMSR", 1024 * 4, NULL, 3, &inms_task_receive);
								inms_task_receive_flag = 1;
								/*----For simulator----*/
								if (isSimulator) {
									for (int j = 2; j <= leng + 3; j++) {
										usart_putstr(2, (char *)&script[rec[i]][flag + j], 1);
									}
								}
								/*----------------------------*/
								printf("COMMAND: OBC_SU_ON...........\n");
							}
							else if (script[rec[i]][flag + 2] == 242) { 														//OBC_SU_OFF = 242
	 							printf("delete inms task receive\n");
								vTaskDelete(inms_task_receive);
								vTaskDelay(0.5 * delay_time_based);
								power_control(4, OFF);	//command EPS to POWER OFF INMS

								/*----For simulator----*/
								if (isSimulator) {
									for (int j = 2; j <= leng + 3; j++) {
										usart_putstr(2, (char *)&script[rec[i]][flag + j], 1);
									}
								}
								/*----------------------------*/
								printf("COMMAND: OBC_SU_OFF...........\n");
							}
							else if (script[rec[i]][flag + 2] == 254) { 															//0xFE = OBC_EOT  = 254 ------------------------------------->check 0xFD or 0xFE
								printf("COMMAND: OBC_EOT...........\n");
								//eotCount++;
								/*----For simulator----*/
								if (isSimulator) {
									for (int j = 2; j <= leng + 3; j++) {
										usart_putstr(2, (char *)&script[rec[i]][flag + j], 1);
									}
								}
								/*----------------------------*/
								ttflag++;
								seqcount++;
								printTime = 1;
								/*delay for OBC_EOT*/
								delayTimeNow = refTime;
								delayTimeTarget = delayTimeNow + script[rec[i]][flag + 1] * 60 + script[rec[i]][flag];
								tempTime = delayTimeNow;
								while (delayTimeTarget != delayTimeNow) {
									printf("%d-----------%d\n", delayTimeNow - tempTime, delayTimeTarget - tempTime);
									printf("\E[1A\r");
									delayTimeNow = delayTimeNow + 1;
									vTaskDelay(1 * delay_time_based);
								}

								/*delay*/
								break;
							}
							else {
								for (int j = 2; j <= leng + 3; j++) {
									usart_putstr(2, (char *)&script[rec[i]][flag + j], 1); //(char *) check !!
									printf("COMMAND : %02x \n", script[rec[i]][flag + j]);
								}
								vTaskDelay(0.2 * delay_time_based);
							}
							/*delay*/
							delayTimeNow = refTime;
							if (script[rec[i]][flag + 1] * 60 + script[rec[i]][flag] == 1110) {
								delayTimeTarget = delayTimeNow + 66;
							}
							else {
								delayTimeTarget = delayTimeNow + script[rec[i]][flag + 1] * 60 + script[rec[i]][flag];
							}
							tempTime = delayTimeNow;
							portTickType xLastWakeTime;
							const portTickType xFrequency = 1000;

							while (delayTimeTarget != delayTimeNow) {
								xLastWakeTime = xTaskGetTickCount();
								printf("%d-----------%d\n", delayTimeNow - tempTime, delayTimeTarget - tempTime);
								printf("\E[1A\r");
								delayTimeNow = delayTimeNow + 1;
								vTaskDelayUntil( &xLastWakeTime, xFrequency );
							}
							/*delay*/
							printf("leng = %d\n", leng);
							flag = flag + leng + 4;
							printf("flag = %d\n", flag);

							if (inmsJumpScriptCheck(i))
								break;
						}
					}
				}
			}
		}
		else {
			printf("INMS handler state is disabled\n");
			printf("use TC 8-17 to enable it\n");
		}
	}
}

/**
 * This is the task for inms monitor, if the over-current condition exist,
 * it will send error flag to INMS error handler.
 *
 */
void vTaskInmsCurrentMonitor(void * pvParameters) {
	int currentValue_5	=	0;
	int currentValue_33	=	0;
	/**
	 * INMS_ICD issue 10
	 * The nominal current for +5V and +3V3 voltage rails are:
	 *  +5V : 140mA (168 mA peak during SU_SCI)
	 *  +3V3: 15mA
	 */
	int overCurrent_5 	= 	300;	/* mA */
	int overCurrent_33 	= 	50;	/* mA */

	uint8_t Current_5V_reg	=	0xB0;
	uint8_t Current_33V_reg	=	0x90;

	uint8_t uchar5[4];
	uint8_t uchar33[4];

	while (1) {
		/*
			Get current sensor data from ADC
		 */
		if (i2c_master_transaction(0, 109, &Current_5V_reg, 1, &uchar5, 4, 2) == E_NO_ERR) {
			// printf("Get 5V Current information");
		}
		if (i2c_master_transaction(0, 109, &Current_33V_reg, 1, &uchar33, 4, 2) == E_NO_ERR) {
			// printf("Get 3.3V Current information");
		}
		/**
		 * These formula is calculated depends on the electrical circuit
		 */
		currentValue_5 	= ((uchar5[0] << 8) + (uchar5[1])) / (0.47 / 2 * 20);
		currentValue_33 = ((uchar33[0] << 8) + (uchar33[1])) / (6.8 / 2 * 20);

		// printf("Current_5V: %d mA\n",currentValue_5);
		// printf("Current_3.3V: %d mA\n",currentValue_33);

		if (currentValue_5 >= overCurrent_5) {
			obcSuErrFlag = 4;
			printf("Over-current Condition Detected, 5V: %d mA\n", currentValue_5);
		}
		if (currentValue_33 >= overCurrent_33) {
			obcSuErrFlag = 4;
			printf("Over-current Condition Detected, 3V: %d mA\n", currentValue_33);
		}

		// if (parameters.inms_function == 0)
		// 	vTaskDelete(NULL);

		vTaskDelay(5 * delay_time_based);
	}
}

void vTaskInmsErrorHandle(void * pvParameters) {
	vTaskDelay(5 * delay_time_based);
	uint8_t rsp_err_code = 0;
	uint8_t seq_cnt = 0;
	uint8_t obcerrpacket[174];
	int len[scriptNum];
	uint8_t ucharAdcs[22];
	uint8_t txbuf = 0x22; 			//defined by ADCS interface doc
	uint8_t errPacketTotal [174 + 22];		//response packet is FIXED 174 BYTES+ADCS 22 BYTES
	for (int i = 0; i < 22; i++) {
		ucharAdcs[i] = 0xFF;
	}
	for (int i = 0; i < scriptNum; i++) {
		printf("script %d: ", i);
		len[i] = inms_script_length(i);
		if (len[i] >= maxlength) {
			maxlength = len[i];
		}
	}
	uint8_t script[scriptNum][maxlength];
	// uint8_t *script[scriptNum];
	// for (int i = 0; i < scriptNum; i++) {
	// 	script[i] = (uint8_t *)malloc(sizeof(int) * len[i]);
	// }
	for (int i = 0; i < scriptNum; i++) {
		inms_script_read(i, len[i], &script[i]);
		// printf("num = %d\n", i);
	}
	xTaskCreate( vTaskinms, (const signed char*) "INMS", 1024 * 4, NULL, 3, &inms_task);
	inms_task_flag = 1;
	while (1) {
		// printf("obcSuErrFlag = %d\n", obcSuErrFlag);
		switch (obcSuErrFlag)  {
		case 0:		//No error
			rsp_err_code = 0x00;
			// power_control(4,OFF);
			break;
		case 1:		//INMS packet time-out error
			rsp_err_code = 0xf0;
			// power_control(4,OFF);
			power_control(4, OFF);
			break;
		case 2:		//INMS packet length error detected
			rsp_err_code = 0xf1;
			//turn off INMS
			power_control(4, OFF);
			break;
		case 3:		//INMS emergency turn OFF executed
			rsp_err_code = 0xf2;
			//turn off INMS
			power_control(4, OFF);
			break;
		case 4:		//INMS over-current condition detected
			rsp_err_code = 0xf3;
			//turn off INMS
			power_control(4, OFF);
			break;
		case 5:		//INMS script command error detected
			rsp_err_code = 0xf4;
			//turn off INMS
			power_control(4, OFF);
			break;
		case 6:		/*INMS checksum error detected*/ /*self defined*/
			rsp_err_code = 0xf5;
			power_control(4, OFF);
			break;
		//others to be defined by PHOENIX team

		default:
			break;
		}
		if (rsp_err_code != 0) {
			if (i2c_master_transaction(0, adcs_node, &txbuf, 1, &ucharAdcs, 22, 2) == E_NO_ERR) {
				printf("Get Time, Attitude, Position from ADCS");
			}
			obcerrpacket[0] = 0xfa;
			obcerrpacket[1] = seq_cnt; //not sure what is it
			obcerrpacket[2] = rsp_err_code;
			obcerrpacket[3] = script[scriptRunning][len[scriptRunning] - 2];
			obcerrpacket[4] = script[scriptRunning][len[scriptRunning] - 1];
			for (int i = 5; i < 5 + 10; i++) {
				obcerrpacket[i] = script[scriptRunning][i - 3];
			}
			int jflag = 15;
			int sflag;

			for (int i = 0; i < 7; i++) {
				obcerrpacket[jflag] = script[i][len[i] - 2];
				obcerrpacket[jflag + 1] = script[i][len[i] - 1];
				sflag = 2;
				for (int j = jflag + 2; j < jflag + 12; j++, sflag++) {
					obcerrpacket[j] = script[i][sflag];
				}
				jflag = jflag + 12;
			}
			printf("obcErrorPacket: \n");
			hex_dump(obcerrpacket, 174);

			for (int i = 0; i < 22; i++) {
				errPacketTotal[i] = ucharAdcs[i];
			}
			for (int i = 22; i < 174 + 22; i++) {
				errPacketTotal[i] = obcerrpacket[i - 22];
			}
			// hex_dump(errPacketTotal,196);
			inms_data_write_dup(errPacketTotal);
			vTaskSuspend(inms_task);
			printf("suspend inms for 60 sec\n");
				
			vTaskDelay(60 * delay_time_based); //ICD p50  wait 60 seconds
			/* turn on INMS */
			vTaskResume(inms_task);
			// power_control(4,ON);
			// to next Times-Table
			obcSuErrFlag = 0;
			if (seq_cnt == 255)
				seq_cnt = 0;
			else
				seq_cnt++;
		}
		// if (parameters.inms_function == 0)
			// vTaskDelete(NULL);
		vTaskDelay(1 * delay_time_based);
	}
}


