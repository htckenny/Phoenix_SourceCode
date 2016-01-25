/*
 * task_INMS.c
 *
 *  Created on: 	2014/10/06
 *  Last updated: 	2016/01/03
 *      Author: Kenny Huang
 */

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <time.h>
#include <math.h>

#include <fat_sd/ff.h>
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
#include "fs.h"


#define isSimulator 			0
#define isFunctionTest			1

typedef struct __attribute__((packed)) {
	int tt_hour, tt_min, tt_sec, tt_seq;
} timetable;

int rec[scriptNum];

int scriptRunning = 0;		/* initializing for identifying which script is running */
int obcSuErrFlag = 0;		/* for the detection of the OBC_SU_ERR */
int error_flag = 0;
int maxlength = 0;
uint32_t epoch_sec[scriptNum];/* the seconds from UTC epoch time 2000/01/01 00:00:00 Am */
uint32_t refsec[scriptNum] ;

/**
 * This function is used for sorting the script time
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
/**
 * Pre handler for bubble sort
 */
void inmssort(uint32_t *sec) {

	timestamp_t t;
	t.tv_sec = 0;
	t.tv_nsec = 0;
	obc_timesync(&t, 6000);
	for (int i = 0; i < scriptNum; i++) {
		refsec[i] = 0;
	}
	for (int i = 0; i < scriptNum; i++) {
		printf("sec[%d] = %" PRIu32 "\n", i, sec[i]);
	}
	// printf("t.tv_sec = %" PRIu32 "\n", t.tv_sec);
	for (int i = 0; i < scriptNum; i++) {
		refsec[i] = (sec[i] - (t.tv_sec - 946684800));
		printf("refsec[%d] = %" PRIu32 "\n", i, refsec[i]);
	}
	bubblesort(refsec);
}

/**
 * Calculate the FLETCHER-16 checksum, the formula is based on the REQ: INMS-I-127
 * @param  script contents of the script
 * @param  length lengh of the script
 * @return        check sum results
 */
uint16_t fletcher(uint8_t *script, size_t length) {

	uint16_t C0_int = 0, C1_int = 0;
	uint16_t XSUM_W = 0xFFFF;	//0xFFFF = 0d65535

	for (unsigned int i = 0; i < length; i++) {
		C0_int = C0_int + script[i];
		C1_int = C1_int + C0_int;
		C0_int = (C0_int) % (255);
		C1_int = (C1_int) % (255);
	}

	XSUM_W = (C0_int) || (C1_int << 8);
	return XSUM_W;
}

/* Return the system time with big clock(Real seconds) or small clock(86400) */
uint32_t timeGet (int clockType) {
	uint32_t onBoardTime = 0;
	timestamp_t t;
	t.tv_sec = 0;
	t.tv_nsec = 0;
	obc_timesync(&t, 1000);

	onBoardTime = (clockType == 0) ? ( t.tv_sec) : ( t.tv_sec % 86400);
	return onBoardTime;
}

/**
 * This function is used to determine if the next script is coming or not
 * @param  currentScript the next script's ref num
 * @return               1: if the next script is coming, 0: No script is coming
 */
int inmsJumpScriptCheck (int currentScript) {
	/* After finish each command, perform this check to see if the next script is coming */
	uint32_t timeRef ;
	timeRef = timeGet(0);
	timeRef -= 946684800;
	/* Scan next Script's time */
	if (currentScript != 6)
		currentScript++;

	// printf("\t\t\t\t\t\ttimer =  %" PRIu32 "\n", timeRef);
	// printf("\t\t\t\t\t\tepoch =  %" PRIu32 "\n", epoch_sec[rec[currentScript]]);
	printf("\n\t\t\t\t\t\tNext Script:  %" PRIu32 "\n", epoch_sec[rec[currentScript]] - timeRef);
	printf("\E[2A\r");

	/* jump to next script if the time is within 10 seconds */
	if (isFunctionTest)
		return 0;
	else
		return (timeRef > epoch_sec[rec[currentScript]] - 10) ? 1 : 0;

}

/**
 * This task is used for receiving the packet from INMS, should check the Rx buffer every one second
 */
void vTaskInmsReceive(void * pvParameters) {
	int numReceive = 0;
	char ucharAdcs[22];
	char ucharTotal[174 + 22];		//response packet is FIXED 174 BYTES+ADCS 22 BYTES
	// char ucharBuffer[174*10];	// buffer overflow
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
	const portTickType xFrequency = 1 * delay_time_based;

	while (1) {
		/* response_pkt */
		xLastWakeTime = xTaskGetTickCount();
		receiveFlag++;

		/* INMS requirement generate packet within 400 seconds */
		if (receiveFlag >= parameters.INMS_timeout) {
			obcSuErrFlag = 1;
		}
		numReceive = usart_messages_waiting(2);
		// if (numReceive != 0) {
		// 	printf("numReceive = %d\n", numReceive);
		// }
		int ADCSexist = 0;
		if (numReceive != 0) {
			printf("Get response packet  %d!\n", numReceive);

			if (ADCSexist == 1) {
				t.tv_sec = 0;
				t.tv_nsec = 0;
				obc_timesync(&t, 6000);
				memcpy(&ucharAdcs[0], &t.tv_sec, 4);
				if (i2c_master_transaction_2(0, adcs_node, &txbuf, 1, &rxbuf, 48, adcs_delay) == E_NO_ERR) {
					memcpy(&ucharAdcs[4], &rxbuf[18], 18);
					printf("Get Time, Attitude, Position from ADCS\n");
				}
				else {
					printf("Get data from ADCS FAILED \r\n");
				}
				for (int i = 0; i < 22; i++) {
					ucharTotal[i] = ucharAdcs[i];
				}
				for (int i = 22; i < inms_data_length; i++) {
					ucharTotal[i] = usart_getc(2);
				}
			}
			else {
				for (int i = 0; i < 22; i++) {
					ucharTotal[i] = 0xFF;
				}
				for (int i = 22; i < inms_data_length; i++) {
					ucharTotal[i] = usart_getc(2);
				}
			}
			if (ucharTotal[22] == 0xBB) { 	// SU_ERR detected!
				obcSuErrFlag = 5;
			}
			hex_dump(ucharTotal, inms_data_length);
			// inms_data_write_dup((uint8_t *)ucharTotal);
			// vTaskDelay(2 * delay_time_based);
			// hex_dump(ucharTotal, inms_data_length);
			

			numReceive = 0;
			receiveFlag = 0;
		}

		// vTaskDelay()
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
	}
}
/**
 * This is the main INMS task that perform the script handling.
 */
void vTaskinms(void * pvParameters) {

	int script_read_result[scriptNum] = {0};
	uint16_t script_xsum_result[scriptNum] = {0};
	int sequence_time_based;
	int first_rollover = 0;
	printf("Start INMS task......\n");
	vTaskDelay(5 * delay_time_based);
	while (1) {
		vTaskDelay(3 * delay_time_based);
		if (parameters.inms_status == 1) {
			int len[scriptNum];	// the length of each script
			for (int i = 0; i < scriptNum; i++) {
				epoch_sec[i] = 0;
			}
			printf("[-------------STEP #1 : Getting length of all scripts-------------]\n");
			/* STEP #1 Get the length of all of the scripts*/
			for (int i = 0; i < scriptNum; i++) {
				len[i] = inms_script_length_flash(i);
				printf("script %d: %d\n", i, len[i]);
				if (len[i] >= maxlength) {
					maxlength = len[i];
				}
			}
			printf("MAX length : %d\n", maxlength);
			uint8_t script[scriptNum][maxlength];           //(option) Use pointer  check test/jagged array
			for (int i = 0 ; i < scriptNum ; i++) {
				for (int j = 0; j < maxlength; j++) {
					script[i][j] = 0;
				}
			}
			printf("[-------------STEP #2a : Performing Fletcher 16-------------]\n");
			/**
			 * STEP #2 Read the script, put the data into script[]
			 * and perform Fletcher-16 checksum and additional check
			 */
			for (int i = 0; i < scriptNum; i++) {
				script_read_result[i] =  inms_script_read_flash(i, len[i], &script[i]);
				script_xsum_result[i] = fletcher(script[i], len[i]);
				if (script_read_result[i] == Error || script_xsum_result[i] != 0) {
					printf("No. %d script XSUM through Fletcher-16 [FAIL]\n", i);
					obcSuErrFlag = 6;
				}
				else {
					printf("No. %d script XSUM through Fletcher-16 [PASS]\n", i);
				}
			}
			printf("[-------------STEP #2b: Performing additional check with script length-------------]\n");
			for (int i = 0; i < scriptNum; i++) {
				if (len[i] == script[i][0] + (script[i][1] << 8)) {
					printf("No. %d script length check [PASS]\n", i);
				}
				else {
					printf("No. %d script length check [FAIL]\n", i);
					obcSuErrFlag = 2;
				}
			}

			printf("[-------------STEP #3 : Converting 4 bytes times into seconds-------------]\n");
			/* STEP #3 Convert the 4 byte time stamps into seconds since UTC epoch */
			for (int i = 0; i < scriptNum; i++) {
				if (script_read_result[i] == No_Error)
					epoch_sec[i] = (script[i][2]) + (script[i][3] << 8) + (script[i][4] << 16) + (script[i][5] << 24);
				else
					epoch_sec[i] = 0;
				#if isFunctionTest
					epoch_sec[i] -= (3029529600);
				#endif
			}

			/**
			 * Sort these 7 epoch times by calling the function inmssort
			 */
			inmssort(epoch_sec);
			// for (int i = 0; i < scriptNum; i++) {
			// printf("[%d] => %d\n", i, rec[i]);
			// }

			vTaskDelay(1 * delay_time_based);

			/* for temporary setting,  now only have idle0.bin, idle1.bin*/
			// if (isSimulator) {
			// rec[0] = 1;
			// rec[1] = 0;
			// }
			/**
			 * modify here if wanna change the sequence for debugging
			 * For example:
			 * rec[0] = 1;
			 * rec[1] = 0;
			 */
			#if isFunctionTest
				rec[0] = Test_Script;
			#endif

			printf("After Modification\n");
			for (int i = 0; i < scriptNum; i++) {
				printf("[%d] => %d\n", i, rec[i]);
			}

			printf("[-------------STEP #4 : Recording TimeTable with structure-------------]\n");
			/* STEP #4 Record the TimeTable with structure */
			for (int i = 0; i < scriptNum; i++) {
				if (script_xsum_result[rec[i]] != 0) {
					parameters.inms_status = 0;
					para_w_flash();
					break;
				}
				if (parameters.inms_status == 0)
					break;
				/*Initialize*/
				int flag = 0;		//record which byte is running now
				int ttflag = 0;		//times_table flag
				timetable timetable_t[80]; 		//(option) Use pointer
				flag = 15;
				/* Mark an IDLE-SLOT script buffer as the RUNNING-SCRIPT */
				scriptRunning =  rec[i];	//for the need to jump to the next script
				sequence_time_based = epoch_sec[rec[i]] % 86400;
				int printtflag = -1;
				/* EOT = 0x55 */
				while (script[rec[i]][flag] != 0x55) {
					if (script_read_result[i] != No_Error)
						break;

					/* S1 = 0x41 */
					if (script[rec[i]][flag] == 0x41)	{
						timetable_t[ttflag].tt_hour	=	script[rec[i]][flag - 1];
						timetable_t[ttflag].tt_min	=	script[rec[i]][flag - 2];
						timetable_t[ttflag].tt_sec	=	script[rec[i]][flag - 3];
						timetable_t[ttflag].tt_seq	=	1;
						printf("Sequence 1: \n");
					}
					/* S2 = 0x42 */
					else if (script[rec[i]][flag] == 0x42) {
						timetable_t[ttflag].tt_hour	=	script[rec[i]][flag - 1];
						timetable_t[ttflag].tt_min	=	script[rec[i]][flag - 2];
						timetable_t[ttflag].tt_sec	=	script[rec[i]][flag - 3];
						timetable_t[ttflag].tt_seq	=	2;
						printf("Sequence 2: \n");
					}
					/* S3 = 0x43 */
					else if (script[rec[i]][flag] == 0x43) {
						timetable_t[ttflag].tt_hour	=	script[rec[i]][flag - 1];
						timetable_t[ttflag].tt_min	=	script[rec[i]][flag - 2];
						timetable_t[ttflag].tt_sec	=	script[rec[i]][flag - 3];
						timetable_t[ttflag].tt_seq	=	3;
						printf("Sequence 3: \n");
					}
					/* S4 = 0x44 */
					else if (script[rec[i]][flag] == 0x44) {
						timetable_t[ttflag].tt_hour	=	script[rec[i]][flag - 1];
						timetable_t[ttflag].tt_min	=	script[rec[i]][flag - 2];
						timetable_t[ttflag].tt_sec	=	script[rec[i]][flag - 3];
						timetable_t[ttflag].tt_seq	=	4;
						printf("Sequence 4: \n");
					}
					/* S5 = 0x45 */
					else if (script[rec[i]][flag] == 0x45) {
						timetable_t[ttflag].tt_hour	=	script[rec[i]][flag - 1];
						timetable_t[ttflag].tt_min	=	script[rec[i]][flag - 2];
						timetable_t[ttflag].tt_sec	=	script[rec[i]][flag - 3];
						timetable_t[ttflag].tt_seq	=	5;
						printf("Sequence 5: \n");
					}
					else {
						ttflag--;
					}

					if (printtflag != ttflag) {
						printf("hour:%d, min:%d, sec:%d, seq:%d\n", timetable_t[ttflag].tt_hour, timetable_t[ttflag].tt_min, timetable_t[ttflag].tt_sec, timetable_t[ttflag].tt_seq);
						printtflag = ttflag;
					}
					flag ++;
					ttflag++;
					// printf("flag = %d, ttflag = %d\n", flag, ttflag);
					if (flag >= 1000)
						break;
				}
				flag++;
				ttflag--;

				int ttflagMax = ttflag;
				// printf("ttflagMax = %d\n", ttflagMax );
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
				while (nnflag <= (script[rec[i]][0]  + (script[rec[i]][1] << 8))) {
					if (script[rec[i]][nnflag] == 254) {	//0xFE = 0d254
						eotflag[eotnum] = nnflag + 3;
						eotnum++;
					}
					nnflag ++;
				}
				for (int i = 0; i < eotnum; i++) {
					printf("EOTflag[%d] =%d\n", i, eotflag[i]);
				}

				printf("[---------STEP #7 : Executing command at script %d---------]\n", rec[i]);
				/* Start execute the command by following the timetable */
				uint32_t tTable_24 = 0;
				uint32_t refTime = 0;
				ttflag = 0;
				printf("%d", timetable_t[ttflag].tt_seq);
				int printTime = 1;
				int seqcount = 0;
				uint32_t first_time = 0 ;
				if (i == 0) {
					while (1) {
						first_time = timeGet(0);
						first_time -= 946684800;
						// printf("time = %" PRIu32 "\n",first_time);
						printf("\n\t\t\t\t\t\tdiff = %" PRIu32 "\n", epoch_sec[rec[i]] - first_time);
						printf("\E[2A\r");

						if (first_time > epoch_sec[rec[i]] - 10)
							break;
						vTaskDelay(1 * delay_time_based);
						if (parameters.inms_status == 0)
							break;
					}
				}
				while (1) {
					if (inmsJumpScriptCheck(i) && i != scriptNum - 1)
						break;
					if (parameters.inms_status == 0)
						break;
					if (seqcount > ttflagMax) {
						ttflag = 0;
						seqcount = 0;
					}

					vTaskDelay(1 * delay_time_based);
					refTime = timeGet(1);  //get the small clock time


					first_time = timeGet(0) - 946684800;
					tTable_24  = timetable_t[ttflag].tt_hour * 3600 + timetable_t[ttflag].tt_min * 60 +  timetable_t[ttflag].tt_sec;

					if (printTime == 1) {
						printf("\n%.5" PRIu32 " -- %.5" PRIu32 "\n", refTime, tTable_24);
						printf("%.5" PRIu32 "\n", (tTable_24 < refTime) ? tTable_24 - refTime + 86400 : tTable_24 - refTime  );
						printf("\E[3A\r");
					}
					if ((refTime  ==  tTable_24 ) || ((refTime - 1 ) ==  tTable_24 )) {
						printTime = 0;
						if (timetable_t[ttflag].tt_seq == 1) {
							flag = eotflag[0];
							printf("\n\n\nputting flag 1 done\n");
						}
						else if (timetable_t[ttflag].tt_seq == 2) {
							flag = eotflag[1];
							printf("\n\n\nputting flag 2 done\n");
						}
						else if (timetable_t[ttflag].tt_seq == 3) {
							flag = eotflag[2];
							printf("\n\n\nputting flag 3 done\n");
						}
						else if (timetable_t[ttflag].tt_seq == 4) {
							flag = eotflag[3];
							printf("\n\n\nputting flag 4 done\n");
						}
						else if (timetable_t[ttflag].tt_seq == 5) {
							flag = eotflag[4];
							printf("\n\n\nputting flag 5 done\n");
						}
						int leng;
						int delayTimeNow = 0 ;
						int delayTimeTarget = 0;
						int tempTime = 0;
						int inTheSequence = 0;
						while (flag <= (script[rec[i]][0]  + (script[rec[i]][1] << 8))) {
							printf("In the sequence loop : Sending command.\n");
							leng = script[rec[i]][flag + 3]; //get Command's length

							if (error_flag == 1) {
								error_flag = 0;
								if (inTheSequence == 1) {
									seqcount ++;
									ttflag++;
									printTime = 1;
									break;
								}
							}

							inTheSequence = 1;
							/* OBC_SU_ON  = 0xF1 = 0d241 */
							if (script[rec[i]][flag + 2] == 241) {
								int numGabage = 0;
								power_control(4, ON);	//command EPS to POWER ON INMS
								vTaskDelay(0.5 * delay_time_based);

								numGabage = usart_messages_waiting(2);
								while (numGabage != 0) {
									usart_getc(2);
									numGabage = usart_messages_waiting(2);
								}
								// if (inms_task_receive == NULL)
								xTaskCreate(vTaskInmsReceive, (const signed char*) "INMSR", 1024 * 4, NULL, 2, &inms_task_receive);
								/* ---- For simulator ---- */
								#if isSimulator
									for (int j = 2; j <= leng + 3; j++) {
										usart_putstr(2, (char *)&script[rec[i]][flag + j], 1);
									}
								#endif
								/* ----------------------- */

								printf("COMMAND: OBC_SU_ON...........\n");
							}
							/* OBC_SU_OFF = 0xF2 = 0d242 */
							else if (script[rec[i]][flag + 2] == 242) {
								printf("delete inms task receive\n");
								
								vTaskDelay(1 * delay_time_based);
								vTaskDelete(inms_task_receive);
								power_control(4, OFF);


								// }
								/* ---- For simulator ---- */
								#if isSimulator
									for (int j = 2; j <= leng + 3; j++) {
										usart_putstr(2, (char *)&script[rec[i]][flag + j], 1);
									}
								#endif
								/* --------------------- */

								printf("COMMAND: OBC_SU_OFF...........\n");
							}
							/* OBC_EOT  = 0xFE = 0d254 */
							else if (script[rec[i]][flag + 2] == 254) {
								printf("COMMAND: OBC_EOT...........\n");
								ttflag++;
								seqcount++;
								printTime = 1;

								/* delay for OBC_EOT */
								delayTimeNow = refTime;
								delayTimeTarget = delayTimeNow + script[rec[i]][flag + 1] * 60 + script[rec[i]][flag];
								tempTime = delayTimeNow;
								while (delayTimeTarget != delayTimeNow) {
									printf("\n%d-----------%d\n", delayTimeNow - tempTime, delayTimeTarget - tempTime);
									printf("\E[2A\r");
									delayTimeNow = delayTimeNow + 1;
									vTaskDelay(1 * delay_time_based);
								}
								break;
							}
							else {
								for (int j = 2; j <= leng + 3; j++) {
									usart_putstr(2, (char *)&script[rec[i]][flag + j], 1); //(char *) check !!
									printf("COMMAND : %02x \n", script[rec[i]][flag + j]);
								}
								vTaskDelay(0.2 * delay_time_based);
							}
							/* delay */
							delayTimeNow = refTime;
							/* Remove before flight */
							// if (script[rec[i]][flag + 1] * 60 + script[rec[i]][flag] == 1110) {
							// delayTimeTarget = delayTimeNow + 66;
							// }
							// else {
							delayTimeTarget = delayTimeNow + script[rec[i]][flag + 1] * 60 + script[rec[i]][flag];
							// }
							tempTime = delayTimeNow;
							portTickType xLastWakeTime;
							const portTickType xFrequency = 1 * delay_time_based;

							while (delayTimeTarget != delayTimeNow) {
								xLastWakeTime = xTaskGetTickCount();
								printf("\n%d-----------%d\n", delayTimeNow - tempTime, delayTimeTarget - tempTime);
								printf("\E[2A\r");
								delayTimeNow = delayTimeNow + 1;
								vTaskDelayUntil( &xLastWakeTime, xFrequency );
							}
							flag = flag + leng + 4;
							if (inmsJumpScriptCheck(i) && i != scriptNum - 1) {
								break;
							}

						}
					}
					else {

						// printf("%" PRIu32 ", %" PRIu32 "\n", epoch_sec[rec[i]] + tTable_24 - sequence_time_based, first_time );
						if (refTime == 0) {
							first_rollover = 1;
						}
						if ((epoch_sec[rec[i]] + tTable_24 - sequence_time_based)  < first_time ) {
							ttflag++;
							seqcount++;
							// jump_delay_time = 1;
						}
						// else
							// jump_delay_time = 0;
						// printf("\t\tdifftime: %" PRIu32 "\n",first_time - (epoch_sec[rec[i]] - sequence_time_based) );
						if ((first_time - (epoch_sec[rec[i]] - sequence_time_based)) >= 86400 && first_time > (epoch_sec[rec[i]] - sequence_time_based) ) {
							if (first_rollover) {
								first_rollover = 0;
								ttflag = 0;
								seqcount = 0;
							}
							else if (first_rollover == 0) {
								ttflag--;
								seqcount--;
							}
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
	uint16_t currentValue_5		=	0;
	uint16_t currentValue_33	=	0;
	/**
	 * INMS_ICD issue 10
	 * The nominal current for +5V and +3V3 voltage rails are:
	 *  +5V : 140mA (168 mA peak during SU_SCI)
	 *  +3V3: 15mA
	 */
	double overCurrent_5 		= 	250;	/* mA */
	double overCurrent_33 		= 	30;		/* mA */

	vTaskDelay(5 * delay_time_based);
	while (1) {
		/* Get current sensor data from ADC */
		currentValue_5 = Interface_5V_current_get() ;
		currentValue_33 = Interface_3V3_current_get();

		printf("\n\t\t\tCurrent_5V: %.3lf mA\n", (double)currentValue_5 / 4.7);
		printf("\t\t\tCurrent_3.3V: %.3lf mA\n", (double)currentValue_33 / 68);
		// printf("%" PRIu16 "  -  %" PRIu16 "\n", currentValue_5, overCurrent_5);

		printf("\E[3A\r");
		if ((double)currentValue_5 / 4.7 >= overCurrent_5) {
			obcSuErrFlag = 4;
			printf("Over-current Condition Detected, 5V: %d mA\n", currentValue_5);
		}
		if ((double)currentValue_33 / 68 >= overCurrent_33) {
			obcSuErrFlag = 4;
			printf("Over-current Condition Detected, 3V: %d mA\n", currentValue_33);
		}
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
	uint8_t txbuf = 0x22; 					//defined by ADCS interface doc
	uint8_t errPacketTotal [174 + 22];		//response packet is FIXED 174 BYTES+ADCS 22 BYTES
	for (int i = 0; i < 22; i++) {
		ucharAdcs[i] = 0xFF;
	}
	for (int i = 0; i < scriptNum; i++) {
		len[i] = inms_script_length_flash(i);
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
		inms_script_read_flash(i, len[i], &script[i]);
		// printf("num = %d\n", i);
	}
	if (inms_task == NULL)
		xTaskCreate( vTaskinms, (const signed char*) "INMS", 1024 * 4, NULL, 2, &inms_task);
	while (1) {
		switch (obcSuErrFlag)  {
		/* No error */
		case 0:
			rsp_err_code = 0x00;
			break;
		/* INMS packet time-out error */
		case 1:
			rsp_err_code = 0xf0;
			break;
		/* INMS packet length error detected */
		case 2:
			rsp_err_code = 0xf1;
			break;
		/* INMS emergency turn OFF executed */
		case 3:
			rsp_err_code = 0xf2;
			break;
		/* INMS over-current condition detected */
		case 4:
			rsp_err_code = 0xf3;
			break;
		/* INMS script command error detected */
		case 5:
			rsp_err_code = 0xf4;
			break;
		/* ------------------------------------ */	
		/* others to be defined by PHOENIX team */				
		/* ------------------------------------ */	
		/* INMS checksum error detected */ 
		case 6:
			rsp_err_code = 0xf5;
			break;
		
		default:
			break;
		}
		if (rsp_err_code != 0) {
			vTaskDelay(1 * delay_time_based);
			if (i2c_master_transaction_2(0, adcs_node, &txbuf, 1, &ucharAdcs, 22, adcs_delay) == E_NO_ERR) {
				printf("Get Time, Attitude, Position from ADCS\n");
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

			for (int i = 0; i < 22; i++) {
				errPacketTotal[i] = ucharAdcs[i];
			}
			for (int i = 22; i < 174 + 22; i++) {
				errPacketTotal[i] = obcerrpacket[i - 22];
			}
			hex_dump(errPacketTotal, inms_data_length);
			inms_data_write_dup(errPacketTotal);

			if (inms_task_receive != NULL) {
				vTaskDelete(inms_task_receive);
			}
			power_control(4, OFF);
			#if isSimulator
				unsigned int cmd1 = 0xf2;
				unsigned int cmd2 = 0x01;
				unsigned int cmd3 = 0x00;
				char cmd_1 = (char) cmd1;
				char cmd_2 = (char) cmd2;
				char cmd_3 = (char) cmd3;
				usart_putstr(2, &cmd_1, 1);
				usart_putstr(2, &cmd_2, 1);
				usart_putstr(2, &cmd_3, 1);
			#endif



			printf("suspend inms for 60 sec\n");
			vTaskSuspend(inms_task);

			vTaskDelay(60 * delay_time_based); //ICD p50  wait 60 seconds
			/* turn on INMS */
			vTaskResume(inms_task);
			// power_control(4,ON);
			// to next Times-Table
			error_flag = 1;
			obcSuErrFlag = 0;
			if (seq_cnt == 255)
				seq_cnt = 0;
			else
				seq_cnt++;
		}
		vTaskDelay(1 * delay_time_based);
	}
}


