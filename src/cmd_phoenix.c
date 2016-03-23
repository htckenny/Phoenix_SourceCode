/*
 * cmd_phoenix.c
 *
 *  Created on: 	2015/12/25
 *  Last updated:	2015/12/28
 *      Author: Kenny Huang
 */
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>

#include <nanomind.h>
#include <util/console.h>
#include <util/timestamp.h>
#include <util/vermagic.h>
#include <util/log.h>
#include <csp/csp_endian.h>
#include <util/hexdump.h>
#include <dev/i2c.h>
#include <dev/usart.h>
#include <vfs/vfs.h>

#include "parameter.h"
#include "subsystem.h"
#include "tele_function.h"
#include "fs.h"

int eps_switch(struct command_context * ctx) {
	unsigned int buffer;
	extern void EPS_Task(void * pvParameters);

	if (ctx->argc < 2) {
		return CMD_ERROR_SYNTAX;
	}
	if (sscanf(ctx->argv[1], "%u", &buffer) != 1) {
		return CMD_ERROR_SYNTAX;
	}
	if (buffer == 1)
		xTaskCreate(EPS_Task, (const signed char *) "EPSS", 1024 * 4, NULL, 2, NULL);
	return CMD_ERROR_NONE;
}
int firstflight_switch(struct command_context * ctx) {
	unsigned int buffer;
	if (ctx->argc < 2) {
		return CMD_ERROR_SYNTAX;
	}
	if (sscanf(ctx->argv[1], "%u", &buffer) != 1) {
		return CMD_ERROR_SYNTAX;
	}
	// if (buffer == 1) {
	parameters.first_flight = buffer;
	para_w_flash();
	return CMD_ERROR_NONE;
}
int moveScriptFromSD (struct command_context *ctx) {

	int in, out, fdold, fdnew;
	char old[21], new[21], buf[512];
	/* Get args */
	if (ctx->argc != 1)
		return CMD_ERROR_SYNTAX;

	for (int i = 0 ; i < scriptNum ; i++) {

		sprintf(old, "/sd%d/INMS/idle%d.bin", parameters.SD_partition_flag, i);
		sprintf(new, "/boot/INMS/idle%d.bin", i);

		fdold = open(old, O_RDONLY);
		if (fdold < 0) {
			printf("cp: failed to open %s\r\n", old);
			return CMD_ERROR_FAIL;
		}

		fdnew = open(new, O_CREAT | O_TRUNC | O_WRONLY);
		if (fdnew < 0) {
			printf("cp: failed to open %s\r\n", old);
			close(fdold);
			return CMD_ERROR_FAIL;
		}
		do {
			in = read(fdold, buf, sizeof(buf));
			if (in > 0) {
				out = write(fdnew, buf, in);
				if (in != out) {
					printf("cp: failed to write to %s\r\n", new);
					close(fdold);
					close(fdnew);
					return CMD_ERROR_FAIL;
				}
			}
		} while (in > 1);

		printf("cp: success write into %s\r\n", new);
		close(fdold);
		close(fdnew);
	}
	return CMD_ERROR_NONE;
}

int measure_INMS_current(struct command_context * ctx) {
	uint16_t current_3V3, current_5V0;

	current_5V0 = Interface_5V_current_get();
	current_3V3 = Interface_3V3_current_get();

	printf("5V Supply: %.3f mA\n", (double)current_5V0 / 4.7);
	printf("3.3V Supply: %.3f mA\n", (double)current_3V3 / 68);

	return CMD_ERROR_NONE;
}

int load_INMS_script(struct command_context * ctx) {
	unsigned int buffer;

	if (ctx->argc < 2) {
		return CMD_ERROR_SYNTAX;
	}
	if (sscanf(ctx->argv[1], "%u", &buffer) != 1) {
		return CMD_ERROR_SYNTAX;
	}
	Test_Script = buffer;
	return CMD_ERROR_NONE;
}

int mode_switch(struct command_context * ctx) {
	unsigned int buffer;
	extern void ModeControl_Task(void * pvParameters);
	if (ctx->argc < 2) {
		return CMD_ERROR_SYNTAX;
	}
	if (sscanf(ctx->argv[1], "%u", &buffer) != 1) {
		return CMD_ERROR_SYNTAX;
	}
	if (buffer == 1) {
		xTaskCreate(ModeControl_Task, (const signed char * ) "MC", 1024 * 4, NULL, 1, &mode_task);
	}
	else if (buffer == 0) {
		vTaskDelete(mode_task);
	}
	return CMD_ERROR_NONE;
}

int INMSsend_handler(struct command_context * ctx) {

	unsigned int cmd1;
	unsigned int cmd2;
	unsigned int cmd3;
	unsigned int cmd4;
	unsigned int cmd5;
	unsigned int cmd6;
	unsigned int cmd7;
	unsigned int cmd8;

	if (!(ctx->argc >= 4 && ctx->argc <= 9)) {
		printf("inms check length error\r\n");
		return CMD_ERROR_SYNTAX;
	}
	if (sscanf(ctx->argv[1], "%X", &cmd1) != 1) {
		printf("inms cmd1 error\r\n");
		return CMD_ERROR_SYNTAX;
	}
	if (sscanf(ctx->argv[2], "%X", &cmd2) != 1) {
		printf("inms cmd2 error\r\n");
		return CMD_ERROR_SYNTAX;
	}
	if (sscanf(ctx->argv[3], "%X", &cmd3) != 1) {
		printf("inms cmd3 error\r\n");
		return CMD_ERROR_SYNTAX;
	}
	if (ctx->argc == 5) {
		if (sscanf(ctx->argv[4], "%X", &cmd4) != 1) {
			printf("inms cmd4 error\r\n");
			return CMD_ERROR_SYNTAX;
		}
	}
	if (ctx->argc == 8) {
		if (sscanf(ctx->argv[4], "%X", &cmd4) != 1) {
			printf("inms cmd4 error\r\n");
			return CMD_ERROR_SYNTAX;
		}
		if (sscanf(ctx->argv[5], "%X", &cmd5) != 1) {
			printf("inms cmd5 error\r\n");
			return CMD_ERROR_SYNTAX;
		}
		if (sscanf(ctx->argv[6], "%X", &cmd6) != 1) {
			printf("inms cmd6 error\r\n");
			return CMD_ERROR_SYNTAX;
		}
		if (sscanf(ctx->argv[7], "%X", &cmd7) != 1) {
			printf("inms cmd7 error\r\n");
			return CMD_ERROR_SYNTAX;
		}
	}
	if (ctx->argc == 9) {
		if (sscanf(ctx->argv[4], "%X", &cmd4) != 1) {
			printf("inms cmd4 error\r\n");
			return CMD_ERROR_SYNTAX;
		}
		if (sscanf(ctx->argv[5], "%X", &cmd5) != 1) {
			printf("inms cmd5 error\r\n");
			return CMD_ERROR_SYNTAX;
		}
		if (sscanf(ctx->argv[6], "%X", &cmd6) != 1) {
			printf("inms cmd6 error\r\n");
			return CMD_ERROR_SYNTAX;
		}
		if (sscanf(ctx->argv[7], "%X", &cmd7) != 1) {
			printf("inms cmd7 error\r\n");
			return CMD_ERROR_SYNTAX;
		}
		if (sscanf(ctx->argv[8], "%X", &cmd8) != 1) {
			printf("inms check point5 error\r\n");
			return CMD_ERROR_SYNTAX;
		}
	}

	char cmd01 = (char)cmd1;
	char cmd02 = (char)cmd2;
	char cmd03 = (char)cmd3;
	char cmd04 = (char)cmd4;
	char cmd05 = (char)cmd5;
	char cmd06 = (char)cmd6;
	char cmd07 = (char)cmd7;
	char cmd08 = (char)cmd8;

	//	char inmscmd[]= {0xF1,0x01,0x01};
	printf("send uart to port 2\n\r");
	int nums = 0;
	char uchar[174 * 10];

	usart_putstr(2, &cmd01, 1);
	usart_putstr(2, &cmd02, 1);
	usart_putstr(2, &cmd03, 1);
	if (ctx->argc == 5) {
		usart_putstr(2, &cmd04, 1);
	}
	if (ctx->argc == 8) {
		usart_putstr(2, &cmd04, 1);
		usart_putstr(2, &cmd05, 1);
		usart_putstr(2, &cmd06, 1);
		usart_putstr(2, &cmd07, 1);
	}
	if (ctx->argc == 9) {
		usart_putstr(2, &cmd04, 1);
		usart_putstr(2, &cmd05, 1);
		usart_putstr(2, &cmd06, 1);
		usart_putstr(2, &cmd07, 1);
		usart_putstr(2, &cmd08, 1);
	}

	//inms 0x04 0x02 0x02 0x40
	vTaskDelay(2 * delay_time_based);
	nums = usart_messages_waiting(2);
	//printf(" %d \n\r ",nums);
	if (nums != 0) {
		printf("seems get something!\n\r");
		for (int f = 0; f < nums; f++) {
			uchar[f] = usart_getc(2);
		}
		printf("\n");
	}
	hex_dump(uchar, nums);
	return CMD_ERROR_NONE;
}
int INMSreceive_handler(struct command_context * ctx) {

	printf("receive  uart from port 2\n\r");
	int nums = 0;
	char uchar[174 * 10];

	nums = usart_messages_waiting(2);
	if (nums != 0) {
		printf("seems get something! %d\n\r", nums);
		for (int f = 0; f < nums; f++) {
			uchar[f] = usart_getc(2);
		}
		printf("\n");
	}
	hex_dump(uchar, nums);
	return CMD_ERROR_NONE;
}
int I2Csend_handler(struct command_context * ctx) {
	unsigned int rx;
	unsigned int node;
	unsigned int para[255];
	int i;
	if (ctx->argc < 3) {
		return CMD_ERROR_SYNTAX;
	}
	if (sscanf(ctx->argv[1], "%u", &node) != 1) {
		return CMD_ERROR_SYNTAX;
	}
	if (sscanf(ctx->argv[2], "%u", &rx) != 1) {
		return CMD_ERROR_SYNTAX;
	}

	if (ctx->argc > 3) {
		for (i = 0; i < (ctx->argc - 3); i++) {
			sscanf(ctx->argv[i + 3], "%u", &para[i]);
		}
	}

//-------------finish read typing-----------------//
	uint8_t val[rx];
	uint8_t tx[255];
	printf("Send I2C Message [node %2X  rx %d ", node, rx);
	if (ctx->argc > 3) {
		printf("parameter");
		for (i = 0; i < (ctx->argc - 3); i++) {
			printf("%2X  ", para[i]);
			tx[i] = (uint8_t)para[i];
		}
	}
	printf("] \n");
	if (ctx->argc > 3) {
		if ( i2c_master_transaction_2(0, node, &tx, ctx->argc - 3, &val, rx, 10) != E_NO_ERR) {
			printf("No reply from node %x \r\n", node);
			return CMD_ERROR_NONE;
		}
	}
	else {
		if ( i2c_master_transaction_2(0, node, 0, 0, &val, rx, 10) != E_NO_ERR) {
			printf("No reply from node %x \r\n", node);
			return CMD_ERROR_NONE;
		}
	}
	if (rx > 0)
		hex_dump(&val, rx);
	return CMD_ERROR_NONE;
}
int INMS_switch(struct command_context * ctx) {
	unsigned int buffer;
	extern void vTaskinms(void * pvParameters);
	extern void vTaskInmsReceive(void * pvParameters);
	extern void vTaskInmsTemperatureMonitor(void * pvParameters);
	extern void vTaskInmsErrorHandle(void * pvParameters);
	extern void vTaskInmsCurrentMonitor(void * pvParameters);

	if (ctx->argc < 2) {
		return CMD_ERROR_SYNTAX;
	}
	if (sscanf(ctx->argv[1], "%u", &buffer) != 1) {
		return CMD_ERROR_SYNTAX;
	}
	if (buffer == 1) {
		// xTaskCreate(vTaskInmsReceive, (const signed char*) "INMSR", 1024 * 4, NULL, 2, &inms_task_receive);
		xTaskCreate(vTaskinms, (const signed char * ) "INMS", 1024 * 4, NULL, 2, &inms_task);
		// xTaskCreate(vTaskInmsErrorHandle, (const signed char * ) "INMS_EH", 1024 * 4, NULL, 2, &inms_error_handle);
		// xTaskCreate(vTaskInmsCurrentMonitor, (const signed char * ) "INMS_CM", 1024 * 4, NULL, 2, &inms_current_moniter);
		xTaskCreate(vTaskInmsTemperatureMonitor, (const signed char * ) "InmsTM", 1024 * 4, NULL, 1, &inms_temp_moniter);
	}
	else if (buffer == 0) {
		vTaskDelete(inms_task);
		// vTaskDelete(inms_error_handle);
		// vTaskDelete(inms_task_receive);
		// vTaskDelete(inms_current_moniter);
		vTaskDelete(inms_temp_moniter);
	}
	return CMD_ERROR_NONE;
}
int check_mode(struct command_context * ctx) {
	printf("%d\n", HK_frame.mode_status_flag);
	return CMD_ERROR_NONE;
}
int adcs_switch(struct command_context * ctx) {
	unsigned int buffer;
	extern void ADCS_Task(void * pvParameters);
	extern void EOP_Task(void * pvParameters);
	extern void vTaskfstest(void * pvParameters);
	if (ctx->argc < 2) {
		return CMD_ERROR_SYNTAX;
	}
	if (sscanf(ctx->argv[1], "%u", &buffer) != 1) {
		return CMD_ERROR_SYNTAX;
	}
	if (buffer == 1) {
		xTaskCreate(ADCS_Task, (const signed char * ) "ADCS", 1024 * 4, NULL, 1, &adcs_task);
	}
	else if (buffer == 0) {
		vTaskDelete(adcs_task);
	}
	else if (buffer == 2) {
		xTaskCreate(vTaskfstest, (const signed char *) "FS_T", 1024 * 4, NULL, 2, NULL);
	}
	else if (buffer == 3) {
		xTaskCreate(EOP_Task, (const signed char * ) "EOP", 1024 * 4, NULL, 2, &eop_task);
	}
	return CMD_ERROR_NONE;
}

int seuv_switch(struct command_context * ctx) {
	unsigned int buffer;
	extern void SolarEUV_Task(void * pvParameters);
	if (ctx->argc < 2) {
		return CMD_ERROR_SYNTAX;
	}
	if (sscanf(ctx->argv[1], "%u", &buffer) != 1) {
		return CMD_ERROR_SYNTAX;
	}
	if (buffer == 1) {
		xTaskCreate(SolarEUV_Task, (const signed char * ) "SEUV", 1024 * 4, NULL, 3, &seuv_task);
		// xTaskCreate(SEUV_CurrentMonitor, (const signed char * ) "SEUV_CM", 1024 * 4, NULL, 3, NULL);
	}
	else if (buffer == 0) {
		vTaskDelete(seuv_task);
	}
	return CMD_ERROR_NONE;
}
int telecom(struct command_context * ctx) {
	unsigned int buffer;
	extern void Telecom_Task(void * pvParameters);
	if (ctx->argc < 2) {
		return CMD_ERROR_SYNTAX;
	}
	if (sscanf(ctx->argv[1], "%u", &buffer) != 1) {
		return CMD_ERROR_SYNTAX;
	}
	if (buffer == 1) {
		xTaskCreate(Telecom_Task, (const signed char * ) "COM", 1024 * 4, NULL, 2, &com_task);
	}
	else if (buffer == 0) {
		vTaskDelete(com_task);
	}
	return CMD_ERROR_NONE;
}
int ir(struct command_context * ctx) {

	unsigned int buffer;
	int len;

	if (ctx->argc < 2) {
		return CMD_ERROR_SYNTAX;
	}
	if (sscanf(ctx->argv[1], "%u", &buffer) != 1) {
		return CMD_ERROR_SYNTAX;
	}
	printf("slot  = %d\n", buffer);
	len = inms_script_length_flash((int)buffer);
	uint8_t script[len] ;
	printf("length = %d\n", len);
	if (inms_script_read_flash(buffer, len, &script) == No_Error)
		hex_dump(&script, len);
	else {
		return CMD_ERROR_FAIL;
	}

	return CMD_ERROR_NONE;
}
// Simulate receive telecommand and execute it
int ct(struct command_context * ctx) {

	unsigned int serviceType;
	unsigned int serviceSubType;
	unsigned int buffers[200];
	uint8_t para[50] = {0};
	// int i;


	if (ctx->argc < 3) {
		return CMD_ERROR_SYNTAX;
	}
	if (sscanf(ctx->argv[1], "%u", &serviceType) != 1) {
		return CMD_ERROR_SYNTAX;
	}
	if (sscanf(ctx->argv[2], "%u", &serviceSubType) != 1) {
		return CMD_ERROR_SYNTAX;
	}
	// printf("%d\n", ctx->argc);
	if (ctx->argc > 3) {
		for (int i = 0; i < (ctx->argc - 3); i++) {
			sscanf(ctx->argv[i + 3], "%u", &buffers[i + 9]);
			para[i + 9] = (uint8_t)buffers[i + 9];
		}
	}
	para[5] = 5 + (ctx->argc) - 4;
	para[7] = serviceType;
	para[8] = serviceSubType;

	switch (serviceType) {

	case T3_SYS_CONF:
		decodeService3(serviceSubType, para);
		break;
	case T8_function_management:
		decodeService8(serviceSubType, para);
		break;
	case T11_OnBoard_Schedule:
		decodeService11(serviceSubType, para);
		break;
	case T13_LargeData_Transfer:
		decodeService13(serviceSubType, para);
		break;
	case T15_dowlink_management:
		decodeService15(serviceSubType, para);
		break;
	case T131_ADCS:
		decodeService131(serviceSubType, para);
		break;
	case T132_SEUV:
		decodeService132(serviceSubType, para);
		break;
	default:
		sendTelecommandReport_Failure(para, CCSDS_T1_ACCEPTANCE_FAIL, CCSDS_ERR_ILLEGAL_TYPE);
		break;
	}
	timestamp_t t;
	t.tv_sec = 0;
	t.tv_nsec = 0;
	obc_timesync(&t, 6000);
	lastCommandTime = t.tv_sec;
	return CMD_ERROR_NONE;
}

int jumpTime(struct command_context * ctx)
{
	timestamp_t t;
	t.tv_sec = 0;
	t.tv_nsec = 0 ;
	obc_timesync(&t, 1000);
	unsigned int waitingTime = 0;
	/* Set time in lib-c */

	if (sscanf(ctx->argv[1], "%u", &waitingTime) != 1) {
		printf("Should input some time");
	}
	t.tv_sec += waitingTime;
	obc_timesync(&t, 1000);
	printf("\n");
	return CMD_ERROR_NONE;
}
int scheduleDelete(struct command_context * ctx) {

	uint32_t time_absolute_1, time_absolute_2;
	uint8_t telecommand[256] = {0};
	// uint8_t length = (ctx->argc) - 4;
	unsigned int range;
	// unsigned int subtype;
	// unsigned int buffer[255] = {0};
	if (!ctx->argc >= 4) {
		return CMD_ERROR_SYNTAX;
	}
	if (sscanf(ctx->argv[1], "%u", &range) != 1) {
		return CMD_ERROR_SYNTAX;
	}
	if (sscanf(ctx->argv[2], "%" PRIu32 "", &time_absolute_1) != 1) {
		return CMD_ERROR_SYNTAX;
	}
	if (range == 1) {
		if (sscanf(ctx->argv[3], "%" PRIu32 "", &time_absolute_2) != 1) {
			return CMD_ERROR_SYNTAX;
		}
	}
	/* length of the telecommand */
	telecommand[4] = ctx->argc + 5 ;
	/* 4 bytes time */

	telecommand[5] = 0;
	telecommand[7] = 11;
	telecommand[8] = 6;
	telecommand[9] = range;
	memcpy(&telecommand[10], &time_absolute_1, 4);
	memcpy(&telecommand[15], &time_absolute_2, 4);

	if (schedule_delete(range, telecommand) == 1) {
		return CMD_ERROR_FAIL;
	}
	else {
		schedule_new_command_flag = 1 ;
		// parameters.schedule_series_number ++;
		// para_w();
		return CMD_ERROR_NONE;
	}

}
int scheduleRelated(struct command_context * ctx) {
	unsigned int command_type;
	int32_t time_shift;
	// int time_range;
	uint8_t telecommand[4] = {0};
	if (!ctx->argc >= 2) {
		return CMD_ERROR_SYNTAX;
	}
	if (sscanf(ctx->argv[1], "%u", &command_type) != 1) {
		return CMD_ERROR_SYNTAX;
	}
	if (sscanf(ctx->argv[2], "%" PRIu32 "", &time_shift) != 1) {
		return CMD_ERROR_SYNTAX;
	}
	switch (command_type) {
	/* schedule reset */
	case 1 :
		if (schedule_reset_flash() == 1) {
			return CMD_ERROR_FAIL;
		}
		else {
			schedule_new_command_flag = 1;
			return CMD_ERROR_NONE;
		}
		break;
	/* schedule dump */
	case 2 :
		if (schedule_dump() == 1)
			return CMD_ERROR_FAIL;
		else
			return CMD_ERROR_NONE;
		break;
	/* schedule shift */
	case 3 :
		printf("time_shift = %" PRIu32 "\n", time_shift);
		memcpy(&telecommand, &time_shift, 4);
		for (int i = 0; i < 4 ; i++) {
			printf("%d\n", telecommand[i]);
		}
		if (schedule_shift(telecommand) == 1)
			return CMD_ERROR_FAIL;
		else {
			printf("no error\n");
			return CMD_ERROR_NONE;
		}
		break;
	/* schedule delete */
	// case 4 :
	// 	if (sscanf(ctx->argv[2], "%u", &time_range) != 1) {
	// 		return CMD_ERROR_SYNTAX;
	// 	}

	// 	if (schedule_delete(time_range, ) == 1)
	// 		return CMD_ERROR_FAIL;
	// 	else
	// 		return CMD_ERROR_NONE;
	// 	break;
	default:
		return CMD_ERROR_SYNTAX;
	}

}
int scheduleWrite(struct command_context * ctx)
{
	uint32_t time_absolute;
	uint8_t telecommand[256] = {0};
	uint8_t length = (ctx->argc) - 4;
	unsigned int type;
	unsigned int subtype;
	unsigned int buffer[255] = {0};
	if (!ctx->argc >= 4) {
		return CMD_ERROR_SYNTAX;
	}
	if (sscanf(ctx->argv[1], "%" PRIu32 "", &time_absolute) != 1) {
		return CMD_ERROR_SYNTAX;
	}
	if (sscanf(ctx->argv[2], "%u", &type) != 1) {
		return CMD_ERROR_SYNTAX;
	}
	if (sscanf(ctx->argv[3], "%u", &subtype) != 1) {
		return CMD_ERROR_SYNTAX;
	}
	for (int i = 4; i < (length + 4); i++) {
		if (sscanf(ctx->argv[i], "%u", &buffer[i - 4]) != 1) {
			return CMD_ERROR_SYNTAX;
		}
	}
	// printf("test1 \n");
	/* length of the telecommand */
	telecommand[0] = ctx->argc + 5 ;
	/* 4 bytes time */
	memcpy(&telecommand[1], &time_absolute, 4);
	telecommand[5] = 0;
	telecommand[6] = type;
	telecommand[7] = subtype;

	for (int i = 0 ; i < length ; i++) {
		telecommand[i + 8] = buffer[i];
	}
	// printf("test2\n");
	// if (sscanf(ctx->argv[2], "%u", &len) != 1) {
	// 	return CMD_ERROR_SYNTAX;
	// }
	for (int i = 0 ; i < ctx->argc + 4; i++) {
		// if(telecommand[i] != 0){
		printf("%x ", telecommand[i]);
		// }
	}
	printf("\ntelecommand scan\n");
	para_r_flash();

	if (schedule_write_flash(telecommand) == 1) {
		return CMD_ERROR_FAIL;
	}
	else {
		schedule_new_command_flag = 1 ;
		// parameters.schedule_series_number ++;
		// para_w();
		return CMD_ERROR_NONE;
	}
}

int telecomtest(struct command_context * ctx)
{
	int num;
	int len;
	uint8_t packet[256];

	if (ctx->argc != 3) {
		return CMD_ERROR_SYNTAX;
	}
	if (sscanf(ctx->argv[1], "%u", &num) != 1) {
		return CMD_ERROR_SYNTAX;
	}
	if (sscanf(ctx->argv[2], "%u", &len) != 1) {
		return CMD_ERROR_SYNTAX;
	}

	for (int b = 1; b < 256; b++)
		packet[b] = 255;

	packet[0] = com_tx_send;
	for (int a = 0; a < num; a++) {
		SendPacketWithCCSDS_AX25(&packet, len, 10, 1, 1);
	}

	return CMD_ERROR_NONE;
}

int ccsds_send(struct command_context * ctx) {

	if (ctx->argc < 3)
		return CMD_ERROR_SYNTAX;

	int type;
	int subtype;
	int buffer[255];
	uint8_t txbuf[255];
	uint8_t length = (ctx->argc) - 3;
	printf("length  %d\n", length);
	if (sscanf(ctx->argv[1], "%u", &type) != 1)
		return CMD_ERROR_SYNTAX;
	if (sscanf(ctx->argv[2], "%u", &subtype) != 1)
		return CMD_ERROR_SYNTAX;

	for (int a = 3; a < (length + 3); a++) {
		if (sscanf(ctx->argv[a], "%x", &buffer[a - 3]) != 1) {
			return CMD_ERROR_SYNTAX;
		}
	}


	for (int a = 3; a < (length + 3); a++)
		txbuf[a - 3] = (uint8_t)buffer[a - 3];


	hex_dump(&txbuf, length);

	while (1) {
		if (SendPacketWithCCSDS_AX25(&txbuf, length, obc_apid, type, subtype) != ERR_SUCCESS)
			break;
		vTaskDelay(10 * delay_time_based);
	}
//   SendPacketWithCCSDS_AX25(&txbuf,length,obc_apid,type,subtype);

	return CMD_ERROR_NONE;
}


extern void thermal_test(void * pvParameters);
int T_status = 0;
xTaskHandle T_task;

int T_test(struct command_context * ctx) {
	int mode;

	if (ctx->argc != 2)
		return CMD_ERROR_SYNTAX;

	if (sscanf(ctx->argv[1], "%u", &mode) != 1)
		return CMD_ERROR_SYNTAX;

	if (mode == 1)
		if (T_status == 0) {
			xTaskCreate(thermal_test, (const signed char *) "T_Test", 1024 * 4, NULL, 2, &T_task);
			T_status = 1;
		}

	if (mode == 0)
		if (T_status == 1) {
			vTaskDelete(T_task);
			T_status = 0;
		}

	return CMD_ERROR_NONE;
}

/*---------------ID:9 ShutdownTransmitter----------------*/
int shutdown_tm(struct command_context * ctx) {

	int off_on;

	if (ctx->argc != 2)
		return CMD_ERROR_SYNTAX;

	if (sscanf(ctx->argv[1], "%u", &off_on) != 1)
		return CMD_ERROR_SYNTAX;

	if (off_on == 1)
	{
		parameters.shutdown_flag = 1;
		para_w_flash();
		printf("Shutdown Command Detected!! \r\n");

	}

	if (off_on == 0)
	{
		parameters.shutdown_flag = 0;
		para_w_flash();
		printf("Resume Command Detected!! \r\n");

	}


	return CMD_ERROR_NONE;
}


int jump_mode(struct command_context * ctx) {
	int mode;

	if (ctx->argc != 2)
		return CMD_ERROR_SYNTAX;


	if (sscanf(ctx->argv[1], "%u", &mode) != 1)
		return CMD_ERROR_SYNTAX;

	HK_frame.mode_status_flag = (uint8_t)mode;

	return CMD_ERROR_NONE;
}


int parawrite(struct command_context * ctx) {
	parameter_init();
	return CMD_ERROR_NONE;
}

int pararead(struct command_context * ctx) {

	if (ctx->argc != 1)
		return CMD_ERROR_SYNTAX;

	para_r_flash();
	printf("First Flight \t\t\t%d\n", (int) parameters.first_flight);
	printf("shutdown_flag \t\t\t%d\n", (int) parameters.shutdown_flag);
	printf("ant_deploy_flag \t\t%d\n", (int) parameters.ant_deploy_flag);
	printf("wod_store_count \t\t%d\n", (int) parameters.wod_store_count);
	printf("inms_store_count \t\t%d\n", (int) parameters.inms_store_count);
	printf("seuv_store_count \t\t%d\n", (int) parameters.seuv_store_count);
	printf("hk_store_count \t\t\t%d\n", (int) parameters.hk_store_count);
	printf("eop_store_count \t\t%d\n", (int) parameters.eop_store_count);
	printf("obc_packet_sequence_count \t%d\n", (int) parameters.obc_packet_sequence_count);
	printf("vbat_recover_threshold \t\t%d\n", (int) parameters.vbat_recover_threshold);
	printf("vbat_safe_threshold \t\t%d\n", (int) parameters.vbat_safe_threshold);
	printf("schedule_series_number \t\t%d\n", (int) parameters.schedule_series_number);
	printf("Error_series_number \t\t%d\n", (int) parameters.ErrSerialNumber);
	printf("SEUV mode \t\t\t%d\n", (int) parameters.seuv_mode);
	printf("SEUV period \t\t\t%d\n", (int) parameters.seuv_period);
	printf("SEUV sample rate \t\t%d\n", (int) parameters.seuv_sample_rate);
	printf("reboot_count \t\t\t%d\n", (int) parameters.reboot_count);
	printf("SD_partition_flag \t\t%d\n", (int) parameters.SD_partition_flag);
	printf("inms_status\t\t\t%d\n", (int) parameters.inms_status);
	printf("INMS_timeout\t\t\t%d\n", (int) parameters.INMS_timeout);
	printf("combitrate\t\t\t%d\n", (int) parameters.com_bit_rates);

	return CMD_ERROR_NONE;
}

int paradelete(struct command_context * ctx) {
	para_d_flash();
	return CMD_ERROR_NONE;
}

int T_data_del(struct command_context * ctx) {
	T_data_d();
	return CMD_ERROR_NONE;
}

int alldatadelete(struct command_context * ctx) {

	para_d_flash();
	inms_data_delete();
	wod_delete();
	seuv_delete();
	hk_delete();
	T_data_d();
	return CMD_ERROR_NONE;
}

int inmsdatadelete(struct command_context * ctx) {
	inms_data_delete();
	return CMD_ERROR_NONE;
}
int woddelete(struct command_context * ctx) {
	wod_delete();
	return CMD_ERROR_NONE;
}
int seuvdelete(struct command_context * ctx) {
	seuv_delete();
	return CMD_ERROR_NONE;
}
int hkdelete(struct command_context * ctx) {
	hk_delete();
	return CMD_ERROR_NONE;
}


int idleunlock(struct command_context * ctx) {
	parameters.ant_deploy_flag = 1;
	idleunlocks = 1 ;
	return CMD_ERROR_NONE;
}


int testmode(struct command_context * ctx) {

	if (mode_task != NULL)
		vTaskDelete(mode_task);
	if (bat_check_task != NULL)
		vTaskDelete(bat_check_task);
	if (com_task != NULL)
		vTaskDelete(com_task);
	if (wod_task != NULL)
		vTaskDelete(wod_task);
	if (beacon_task != NULL)
		vTaskDelete(beacon_task);

	if (init_task != NULL)
		vTaskDelete(init_task);
	if (adcs_task != NULL)
		vTaskDelete(adcs_task);
	if (seuv_task != NULL)
		vTaskDelete(seuv_task);
	if (eop_task != NULL)
		vTaskDelete(eop_task);
	if (hk_task != NULL)
		vTaskDelete(hk_task);

	if (inms_error_handle != NULL)
		vTaskDelete(inms_error_handle);
	if (inms_current_moniter != NULL)
		vTaskDelete(inms_current_moniter);
	if (inms_task != NULL)
		vTaskDelete(inms_task);
	if (inms_task_receive != NULL)
		vTaskDelete(inms_task_receive);

	if (schedule_task != NULL)
		vTaskDelete(schedule_task);
	if (seuv_cm_task != NULL)
		vTaskDelete(seuv_cm_task);

	printf("Enter Ground Test mode, delete all task\r\n");
	return CMD_ERROR_NONE;
}
int seuvread(struct command_context * ctx) {

	uint8_t val[5];

	if (i2c_master_transaction_2(0, seuv_node, 0, 0, &val, 5, seuv_delay) == E_NO_ERR) {
		hex_dump(&val, 5);
	} else
		printf("ERROR!!  Get no reply from SEUV \r\n");

	return CMD_ERROR_NONE;
}
int seuvwrite(struct command_context * ctx) {
	unsigned int node;
	uint8_t txdata;
	uint8_t rxdata[4];
	float results;
	if (ctx->argc != 2) {
		return CMD_ERROR_SYNTAX;
	}
	if (sscanf(ctx->argv[1], "%u", &node) != 1) {
		return CMD_ERROR_SYNTAX;
	}
	txdata = node;

	if (i2c_master_transaction_2(0, seuv_node, &txdata, 1, 0, 0, seuv_delay) == E_NO_ERR) {};
	vTaskDelay(0.07 * delay_time_based);
	if (i2c_master_transaction_2(0, seuv_node, &txdata, 1, &rxdata, seuv_data_length, seuv_delay) == E_NO_ERR) {
		hex_dump(&rxdata, seuv_data_length);
		results = (rxdata[0] << 8) + rxdata[1];
		printf("node %x's Value = %.3f\n", node, results);
	}
	else
		printf("ERROR!! No reply from SEUV \r\n");

	return CMD_ERROR_NONE;
}

int comhk2(struct command_context * ctx) {

	uint8_t txdata = com_tx_hk;
	uint8_t val;

	if (i2c_master_transaction_2(0, com_tx_node, &txdata, 1, &val, com_tx_hk_length, com_delay) == E_NO_ERR) {
		hex_dump(&val, com_tx_hk_length);
	} else
		printf("ERROR!!  Get no reply from COM \r\n");

	return CMD_ERROR_NONE;
}
int comhk(struct command_context * ctx) {

	uint8_t txdata = com_rx_hk;
	uint8_t val[com_rx_hk_length];
	// i2c_master_transaction(0, com_rx_node, &txdata, 1, 0, 0, com_delay) ;
	if (i2c_master_transaction_2(0, com_rx_node, &txdata, 1, &val, com_rx_hk_length, com_delay) == E_NO_ERR) {
		hex_dump(&val, com_rx_hk_length);
	}
	// if (i2c_master_transaction(0, com_rx_node, &txdata, 1, &val, com_rx_hk_length, com_delay) == E_NO_ERR) {
	// 	hex_dump(&val, com_rx_hk_length);
	// }
	else
		printf("ERROR!!  Get no reply from COM \r\n");

	return CMD_ERROR_NONE;
}

command_t __root_command ph_commands[] = {
	{ .name = "epss", .help = "PHOENIX: epss []", .handler = eps_switch, },
	{ .name = "ff", .help = "PHOENIX: first flight switch", .handler = firstflight_switch, },
	{ .name = "mSFD", .help = "PHOENIX: mSFD", .handler = moveScriptFromSD, },
	{ .name = "ic", .help = "PHOENIX: ic", .handler = measure_INMS_current, },
	{ .name = "isn", .help = "PHOENIX: isn [buffer]", .handler = load_INMS_script, },
	{ .name = "mcs", .help = "PHOENIX: mcs [ON = 1 / OFF = 0]", .handler = mode_switch, },
	{ .name = "inmsR", .help = "PHOENIX: inms ", .handler = INMSreceive_handler, },
	{ .name = "inms", .help = "PHOENIX: inms <cmd1> <cmd2> <cmd3> <cmd4> ....", .usage = "<cmd1>", .handler = INMSsend_handler, },
	{ .name = "i2c", .help = "PHOENIX: i2c <node> <rx>  will have <para> *N byte ?",	.usage = "<node> <rx> <para *n>", .handler = I2Csend_handler, },
	{ .name = "inmss", .help = "PHOENIX: inmss [ON = 1 / OFF = 0]", .handler = INMS_switch, },
	{ .name = "cm", .help = "PHOENIX: cm", .handler = check_mode, },
	{ .name = "adcss", .help = "PHOENIX: adcss [ON = 1 / OFF = 0]", .handler = adcs_switch, },
	{ .name = "seuvs", .help = "PHOENIX: seuvs [ON = 1 / OFF = 0]", .handler = seuv_switch, },
	{ .name = "tele", .help = "PHOENIX: tele [ON = 1 / OFF = 0]", .handler = telecom, },
	{ .name = "jt", .help = "PHOENIX: jt [sec]", .handler = jumpTime, },
	{
		.name = "schd",
		.help = "PHOENIX: schedule_delete <range> <time_1> <time_2 (opt)>",
		.usage = "<range> <time_1> <time_2 (opt)>",
		.handler = scheduleDelete,
	},
	{
		.name = "schw",
		.help = "PHOENIX: schedule_write <Absolute_time> <Type> <Subtype> <para>",
		.usage = "<Absolute_time> <Type> <Subtype> <para>",
		.handler = scheduleWrite,
	},
	{
		.name = "sch",
		.help = "PHOENIX: schedule_command 1:reset, 2:dump, 3:shift, 4:delete" ,
		.usage = "sch <Command ID> <Para(optional)>",
		.handler = scheduleRelated,
	},
	{ .name = "ir", .help = "PHOENIX: inms script read", .usage = "<buffer>" , .handler = ir, },
	{ .name = "ct", .help = "PHOENIX: simulate receiving a uplink command and execute it", .usage = "<type> <subtype> <data*N> " , .handler = ct, },
	{ .name = "ccsds_send", .help = "PHOENIX: send a ccsds packet every 10 seconds", .usage = "<type> <subtype> <data> " , .handler = ccsds_send, },
	{ .name = "telecomtest", .help = "PHOENIX: telecomtest <packet_num> <packet_len>", .usage = "<packet_num> <packet_len>", .handler = telecomtest,	},
	{ .name = "T_test", .help = "PHOENIX: Activate/OFF Thermal Task,switch 1=on, 0 =off", .usage = "T_test <switch>", .handler = T_test, },
	{ .name = "shutdown_tm", .help = "PHOENIX: change transceiver standby mode", .usage = "shutdown_transmitter <switch>", .handler = shutdown_tm, },
	{ .name = "parawrite", .help = "PHOENIX: write para setting in FS", .handler = parawrite, },
	{ .name = "pararead", .help = "PHOENIX: read on board parameter setting in FS", .usage = "<partition(0, 1)>", .handler = pararead, },
	{ .name = "paradelete", .help = "PHOENIX: delete parameters.bin", .handler = paradelete, },
	{ .name = "T_data_del", .help = "PHOENIX: delete t_obc.bin t_inms.bin", .handler = T_data_del, },
	{ .name = "alldatadelete", .help = "PHOENIX: delete all on board data.bin", .handler = alldatadelete, },
	{ .name = "inmsdatadelete", .help = "PHOENIX: delete inmsdata.bin", .handler = inmsdatadelete, },
	{ .name = "woddelete", .help = "PHOENIX: delete wod.bin", .handler = woddelete, }	,
	{ .name = "seuvdelete", .help = "PHOENIX: delete seuv.bin", .handler = seuvdelete, }	,
	{ .name = "hkdelete", .help = "PHOENIX: delete hk.bin", .handler = hkdelete, }	,
	{ .name = "jump_mode", .help = "PHOENIX: jump_mode [mode] // 0=safe mode, 2=adcs mode,3=payload mode", .handler = jump_mode, },
	{ .name = "idleunlock", .help = "PHOENIX: skip idle 30m step", .handler = idleunlock, },
	{ .name = "testmode", .help = "PHOENIX: enter  testmode, please reboot satellite if want to leave this mode", .handler = testmode, },
	{ .name = "seuvwrite", .help = "PHOENIX: configure SEUV", .usage = "<node>", .handler = seuvwrite, },
	{ .name = "seuvread", .help = "PHOENIX: Take data from a configured Channel", .handler = seuvread, },
	{ .name = "comhk", .help = "PHOENIX: retrive com hk data ", .handler = comhk, },
	{ .name = "comhk2",	.help = "PHOENIX: retrive com transmitter state ", .handler = comhk2, },
};

void cmd_ph_setup(void) {
	command_register(ph_commands);
}
