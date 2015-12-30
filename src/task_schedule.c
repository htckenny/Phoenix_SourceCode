/*
 * task_schedule.c
 *
 *  Created on: 2015/7/9
 *      Author: Kenny
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <util/delay.h>
#include <util/timestamp.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <time.h>
#include <nanomind.h>
#include <dev/i2c.h>
#include <inttypes.h>
#include <stdint.h>
#include <csp/csp_endian.h>
 
#include "fs.h"
#include "tele_function.h"
#include "parameter.h"
#include "subsystem.h"


#define maxlength 	20
#define maxNum		50

uint8_t sche_buf[maxNum * maxlength];
uint8_t sche_buf_sort[maxNum * maxlength];
int sort_seq [maxNum];
int bufNum = 0;
/* swap function used in quicksort  */
void swap(uint32_t *a, uint32_t *b)
{
	uint32_t temp = *a;
	*a = *b;
	*b = temp;
}
/* quick sort algorithm */
void quicksort(uint32_t *data, int left, int right)
{
	uint32_t pivot;
	int i, j;
	if (left >= right) { return; }

	pivot = data[left];
	i = left + 1;
	j = right;

	while (1) {
		while (i <= right) {
			if (data[i] > pivot) { break; }
			i = i + 1;
		}
		while (j > left) {
			if (data[j] < pivot) {
				break;
			}
			j = j - 1;
		}
		if (i > j) { break; }
		swap(&data[i], &data[j]);
	}
	swap(&data[left], &data[j]);
	quicksort(data, left, j - 1);
	quicksort(data, j + 1, right);
}
/* sort the original schedule, and record with the sort_seq array */
void schedule_sort(int number)
{
	uint32_t sche_time [maxNum] = {0};
	uint32_t sche_time_sort [maxNum] = {0};


	for (int i = 0 ; i < number ; i++) {
		memcpy(&sche_time[i], &sche_buf[2 + i * 20], 4);
		memcpy(&sche_time_sort[i], &sche_buf[2 + i * 20], 4);
		// sche_time[i] = 	(sche_buf[i][1] << 24)
		//                 + (sche_buf[i][2] << 16)
		//                 + (sche_buf[i][3] << 8)
		//                 + (sche_buf[i][4])
		//                 + (sche_buf[i][5] >> 8);
		// sche_time_sort[i] =  (sche_buf[i][1] << 24)
		//                      + (sche_buf[i][2] << 16)
		//                      + (sche_buf[i][3] << 8)
		//                      + (sche_buf[i][4])
		//                      + (sche_buf[i][5] >> 8);
		sche_time[i] = csp_ntoh32(sche_time[i]) ;
		sche_time_sort[i] = csp_ntoh32(sche_time_sort[i]) ;                    
	}
	
	quicksort(sche_time_sort, 0, number - 1);


	for (int i = 0 ; i < number ; i++) {
		for (int j = 0 ; j < number ; j++) {
			// printf("%"PRIu32"\t%"PRIu32"\n", sche_time_sort[i], sche_time[j]);
			if (sche_time_sort[i] == sche_time[j]) {
				sort_seq[i] = j ;
				break;
			}
		}
	}
}
/* function to send command */
void sendCommand (uint8_t * telecommand)
{
 	uint8_t serviceType = telecommand[7];
	uint8_t serviceSubType = telecommand[8];
	// printf("serv %d\n",  serviceType);
	// printf("servsub %d\n", serviceSubType);
	switch (serviceType) {
	case T3_SYS_CONF :
		// printf("here\n");
		decodeService3(serviceSubType, telecommand);
		break ;
	case T11_OnBoard_Schedule :
		decodeService11(serviceSubType, telecommand);
		break;
	case T8_function_management:
		decodeService8(serviceSubType,telecommand);
		break;
	case T13_LargeData_Transfer:
		decodeService13(serviceSubType,telecommand);
		break;
	case T15_dowlink_management:
		decodeService15(serviceSubType,telecommand);
		break;
	case T131_ADCS:
		decodeService131(serviceSubType,telecommand);
		break;
	case T132_SEUV:
		decodeService132(serviceSubType,telecommand);
		break;

	default:
		sendTelecommandReport_Failure(telecommand, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
		break;
	}
}
/* function to add seven byte with contents of zero to match the telecommand */
void init_command_buf(uint8_t *sche_buf, int buf_length, uint8_t *final_buf) {
	int total_length = 0;
	for (int i = 0 ; i < 5 ; i++) {
		final_buf[i] = 0;
	}
	total_length = buf_length - 7 ;
	
	memcpy(&final_buf[5], &total_length, 1);
	final_buf[6] = 0;
	printf("buf length = %d\n", buf_length);
	for (int i = 7 ; i < buf_length ; i++) {
		final_buf[i] = sche_buf[i];
	}
	// for(int i = 0 ; i < buf_length;i++){
	// 	printf("%x ", final_buf[i]);
	// }
}
/* the function to find the maximum buffer*/
/* this function still need to verify !!!*/
int findMaxBuf(uint8_t sortbuf[])
{
	int length = 0;
	// int scan_length = 0;
	for (int i = 0 ; i < maxNum ; i++) {
		if (sortbuf[20 * i] != 0 && sortbuf[20 * i] != 0xA5) {
			length = i + 1;
		}
		else
			break;
	}
	return length ;
}
uint32_t update_time()
{
	timestamp_t t;
	t.tv_sec = 0;
	t.tv_nsec = 0;
	obc_timesync(&t, 6000);
	return t.tv_sec ;
}
/* the main task, keep scaning if there's updated schedule, and follows the time to send command  */
void Schedule_Task(void * pvParameters)
{
	// schedule_series_number = 1;
	uint32_t sche_time[maxNum] = { 0 };
	uint32_t onBoardTime;

	onBoardTime = update_time();
	uint8_t tele_buf[210];
	// printf("Before schedule read\n");
	schedule_read(sche_buf);
	// printf("After schedule read\n");
	int lastNum = 0;
	lastNum = findMaxBuf(sche_buf);
	printf("last = %d\n", lastNum );
	while (1) {

		if (schedule_unlink_flag == 1) {
			lastNum = 0;
			for (int i = 0; i < maxNum; i++) {
				sche_buf[i] = 0;
			}
			schedule_unlink_flag = 0;
		}
		else {
			// printf("%d\n", sche_buf);
			schedule_read(sche_buf);
			lastNum = findMaxBuf(sche_buf);
			// printf("Here!!\n");
		}
		schedule_sort(lastNum);
		printf("last Num : %d\n", lastNum);
		// schedule_new_command_flag = 0;
		// }
		// else {
		// 	schedule_sort(lastNum);
		// }
		for (int i = 0 ; i < lastNum ; i++) {
			/* check if there's new command comming in or something changed in the buffer */
			if (schedule_new_command_flag == 1) {
				schedule_new_command_flag = 0;
				break;
			}
			/*
			   the sche_buf is recorded as follows:
			   [0]	[1]	[2]	[3]	[4]	[5]	[6]		[7]		[8]		[9]	[10].....
			   seq	len	Absolute Time Tag		Telecommand Packet
			   									type	s_type	para........
			 */
			// printf("sort_seq %d= %d\n", i, sort_seq[i]);
			memcpy(&sche_time[sort_seq[i]], &sche_buf[sort_seq[i] * 20 + 2], 4);
			sche_time[sort_seq[i]] = csp_ntoh32(sche_time[sort_seq[i]]);
			sche_time[sort_seq[i]] += 946684800;
			/* Add seven byte with contents of zero to match the telecommand */
			// init_command_buf(*sche_buf[sort_seq[i]], sizeof(sche_buf[sort_seq[i]]), *tele_buf);
			// printf("%d\n", sche_buf[sort_seq[i] * 20 + 1]);
			init_command_buf(&sche_buf[sort_seq[i] *20], sche_buf[sort_seq[i] * 20 + 1], tele_buf);

			while (1) {

				if (schedule_new_command_flag == 1) {
					break;
				}
				printf("sche_time = %" PRIu32 " onBoardTime = %" PRIu32 "\n", sche_time[sort_seq[i]], onBoardTime);
				// printf("\E[1A\r");

				onBoardTime = update_time();
				/* determine if the schedule time has exceeded */
				if (sche_time[sort_seq[i]] <= onBoardTime) {
					break;
				}
				/* 5 seconds margin to send the command */
				if (sche_time[sort_seq[i]] - onBoardTime <= 3) {
					sendCommand(tele_buf);
					printf("send command !!\n");
					// for (int i = 7; i < 10+7; ++i)
					// {
					// 	printf("tele[%d] = %d \n", i, tele_buf[i]);
					// }
					*tele_buf = 0;
					break;
				}
				vTaskDelay(1 * delay_time_based);
			}
		}
		vTaskDelay(3 * delay_time_based);
		// printf("there are %d command in the list\n", lastNum);
		// printf("\E[1A\r");
	}
}
