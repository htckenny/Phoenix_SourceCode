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

int finalCheck (struct command_context * ctx)
{
	if (ctx->argc < 1) {
		return CMD_ERROR_SYNTAX;
	}
	printf("\n\n");
	printf("\tNCKU PHOENIX Final Parameter Check:\n");
	printf("_________________________________________________\n");
	printf("|                                               |\n");
	printf("| \E[0;32mSoftware Version: \t\t%s\t\t\E[0m|\n", software_version);
	printf("| \E[0;32mLast Update Time: \t\t%s\t\E[0m|\n", Last_Update_time);
	printf("|-----------------------------------------------|\n");
	if (ground_Test_Mode == 1)
		printf("| \E[1;31mGround Test Mode\t\t%d\E[0m\t\t|\n", ground_Test_Mode);
	else
		printf("| Ground Test Mode\t\t%d\t\t|\n", ground_Test_Mode);
	if (mode_task == NULL)
		printf("| \E[1;31mMode Task\t\t\tOFF\E[0m\t\t|\n");
	else
		printf("| Mode Task\t\t\tON\t\t|\n");
	printf("|-----------------------------------------------|\n");
	printf("| enable_stm_EPS_V\t\t%d\t\t|\n", enable_stm_EPS_V);
	printf("| enable_stm_EPS_I\t\t%d\t\t|\n", enable_stm_EPS_I);
	printf("| enable_stm_ADCS\t\t%d\t\t|\n", enable_stm_ADCS);
	printf("| enable_stm_IFB\t\t%d\t\t|\n", enable_stm_IFB);
	printf("| enable_stm_COM\t\t%d\t\t|\n", enable_stm_COM);
	printf("| enable_stm_ANT\t\t%d\t\t|\n", enable_stm_ANT);
	printf("| enable_stm_ADCS_T\t\t%d\t\t|\n", enable_stm_ADCS_T);
	printf("|-----------------------------------------------|\n");
	if (antenna_deploy == 0)
		printf("| \E[1;31mantenna_deploy\t\t%d\E[0m\t\t|\n", antenna_deploy);
	else
		printf("| antenna_deploy\t\t%d\t\t|\n", antenna_deploy);
	if (mag_meter_deploy == 0)
		printf("| \E[1;31mmagnetometer_deploy\t\t%d\E[0m\t\t|\n", mag_meter_deploy);
	else
		printf("| magnetometer_deploy\t\t%d\t\t|\n", mag_meter_deploy);

	printf("| initial_Time\t\t\t%d\t|\n", initial_Time);
	printf("| GS_threshold\t\t\t%"PRIu32"\t\t|\n", parameters.GS_threshold);
	printf("|_______________________________________________|\n");
	return CMD_ERROR_NONE;
}
int gpsTest (struct command_context * ctx) {
	uint8_t txbuf[6];

	if (ctx->argc < 1) {
		return CMD_ERROR_SYNTAX;
	}
	power_OFF_ALL();
	power_control(1, ON);
	vTaskDelay(20 * delay_time_based);
	txbuf[0] = 0x03;   //0d03 ADCS run mode
	txbuf[1] = 0x01;
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 2, 0, 0, adcs_delay) == E_NO_ERR)
		printf("ID:3\tSet ADCS run mode into enabled\n");
	vTaskDelay(1 * delay_time_based);
	/*---------------------------set the power control-------------------------------------*/
	txbuf[0] = 0x05;   //0d05 Set power control command
	txbuf[1] = 0x01;   //cubecontrol signal=auto(2)
	txbuf[2] = 0x00;   //cubecontrol motor=on(1)
	txbuf[3] = 0x01;   //cubesense =auto(2)
	txbuf[4] = 0x00;   //motor power=on(1)
	txbuf[5] = 0x01;   //GPS LAN Power=auto(2)
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 6, 0, 0, adcs_delay) == E_NO_ERR)
		printf("ID:05\tSet power control command\n");
	power_control(2, ON);
	printf("start to watch GPS status\n");

	return CMD_ERROR_NONE;
}
int changeHeater (struct command_context * ctx) {
	unsigned int buffer[1];
	uint8_t txdata[60];
	uint8_t rxdata[60];

	if (ctx->argc < 2) {
		return CMD_ERROR_SYNTAX;
	}
	if (sscanf(ctx->argv[1], "%u", &buffer[0]) != 1) {
		return CMD_ERROR_SYNTAX;
	}
	txdata[0] = 18;
	if (i2c_master_transaction_2(0, eps_node, &txdata, 1, &rxdata, 60, eps_delay) == E_NO_ERR) {};
	for (int i = 0; i < 58; i++)
	{
		txdata[i + 1] = rxdata[i + 2];
	}
	if (buffer[0] == 1) {
		txdata[2] = 1;
	}
	else if (buffer[0] == 0) {
		txdata[2] = 0;
	}
	else if (buffer[0] == 2) {
		txdata[3] = 0xFB;
		txdata[4] = 0;
	}
	else if (buffer[0] == 3) {
		txdata[3] = 25;
		txdata[4] = 30;
	}
	else if (buffer[0] == 4) {
		txdata[3] = 30;
		txdata[4] = 35;
	}
	else if (buffer[0] == 5) {
		txdata[3] = 0;
		txdata[4] = 3;
	}
	else
		return CMD_ERROR_SYNTAX;
	txdata[0] = 19;
	if (i2c_master_transaction_2(0, eps_node, &txdata, 59, 0, 0, eps_delay) == E_NO_ERR) {};
	return CMD_ERROR_NONE;
}
int powerControl(struct command_context * ctx) {
	unsigned int buffer[2];

	if (ctx->argc < 3) {
		return CMD_ERROR_SYNTAX;
	}
	if (sscanf(ctx->argv[1], "%u", &buffer[0]) != 1) {
		return CMD_ERROR_SYNTAX;
	}
	if (sscanf(ctx->argv[2], "%u", &buffer[1]) != 1) {
		return CMD_ERROR_SYNTAX;
	}
	if (buffer[1] == 1)
		power_control(buffer[0], ON);
	else if (buffer[1] == 0)
		power_control(buffer[0], OFF);
	else
		return CMD_ERROR_SYNTAX;
	return CMD_ERROR_NONE;
}
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
	if (buffer == 1 || buffer == 0)
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
	uint16_t current_3V3, current_5V0, temp_INMS, temp_IFB;

	current_5V0 = Interface_5V_current_get();
	vTaskDelay(0.01 * delay_time_based);
	current_3V3 = Interface_3V3_current_get();
	vTaskDelay(0.01 * delay_time_based);
	temp_INMS = Interface_inms_thermistor_get();
	vTaskDelay(0.01 * delay_time_based);
	temp_IFB = Interface_tmp_get();

	printf("5V Supply: %.3f mA\n", (double)current_5V0 / 4.7);
	printf("3.3V Supply: %.3f mA\n", (double)current_3V3 / 68);
	printf("temp_INMS: %.3f \n", (((double)temp_INMS / 3) - (double)273));
	printf("temp_IFB: %.3f \n", (double)(159 - 0.08569 * temp_IFB));

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
		if (inms_error_handle == NULL)
			xTaskCreate(vTaskInmsErrorHandle, (const signed char * ) "INMS_EH", 1024 * 4, NULL, 2, &inms_error_handle);
	} else if (buffer == 2) {
		if (inms_task == NULL)
			xTaskCreate(vTaskinms, (const signed char * ) "INMS", 1024 * 4, NULL, 2, &inms_task);
	} else if (buffer == 3) {
		if (inms_current_moniter == NULL)
			xTaskCreate(vTaskInmsCurrentMonitor, (const signed char * ) "INMS_CM", 1024 * 4, NULL, 2, &inms_current_moniter);
	} else if (buffer == 4) {
		if (inms_temp_moniter == NULL)
			xTaskCreate(vTaskInmsTemperatureMonitor, (const signed char * ) "InmsTM", 1024 * 4, NULL, 1, &inms_temp_moniter);
	} else if (buffer == 5) {
		if (inms_task_receive == NULL)
			xTaskCreate(vTaskInmsReceive, (const signed char*) "INMSR", 1024 * 4, NULL, 2, &inms_task_receive);
	}
	

	else if (buffer == 0) {
		if (inms_error_handle != NULL) {
			vTaskDelete(inms_error_handle);
			inms_error_handle = NULL;
		}
		if (inms_task != NULL) {
			vTaskDelete(inms_task);
			inms_task = NULL;
		}
		if (inms_current_moniter != NULL) {
			vTaskDelete(inms_current_moniter);
			inms_current_moniter = NULL;
		}
		if (inms_temp_moniter != NULL) {
			vTaskDelete(inms_temp_moniter);
			inms_temp_moniter = NULL;
		}
		if (inms_task_receive != NULL) {
			vTaskDelete(inms_task_receive);
			inms_task_receive = NULL;
		}
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
	extern void GPS_task(void * pvParameters);
	extern void Anomaly_Monitor_Task(void * pvParameters);
	xTaskHandle fs_task;
	extern xTaskHandle adcs_task;
	extern xTaskHandle eop_task;
	extern xTaskHandle Anom_mon_task;
	if (ctx->argc < 2) {
		return CMD_ERROR_SYNTAX;
	}
	if (sscanf(ctx->argv[1], "%u", &buffer) != 1) {
		return CMD_ERROR_SYNTAX;
	}
	if (buffer == 1) {
		xTaskCreate(ADCS_Task, (const signed char * ) "ADCS", 1024 * 4, NULL, 1, &adcs_task);
	}
	else if (buffer == 2) {
		xTaskCreate(vTaskfstest, (const signed char *) "FS_T", 1024 * 4, NULL, 2, &fs_task);
	}
	else if (buffer == 3) {
		xTaskCreate(EOP_Task, (const signed char * ) "EOP", 1024 * 4, NULL, 2, &eop_task);
	}
	else if (buffer == 4) {
		xTaskCreate(Anomaly_Monitor_Task, (const signed char *) "Anom", 1024 * 4, NULL, 3, &Anom_mon_task);
	}
	else if (buffer == 5) {
		xTaskCreate(GPS_task, (const signed char *) "GPS", 1024 * 4, NULL, 3, NULL);
	}
	else if (buffer == 10) {
		ADCS_DEBUG = 0;
	}
	else if (buffer == 11) {
		ADCS_DEBUG = 1;
	}
	else if (buffer == 0) {
		if (adcs_task != NULL)
			vTaskDelete(adcs_task);
		else if (eop_task != NULL)
			vTaskDelete(eop_task);
		else if (Anom_mon_task != NULL)
			vTaskDelete(Anom_mon_task);
		else if (fs_task != NULL)
			vTaskDelete(fs_task);
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
		if (com_task == NULL)
			xTaskCreate(Telecom_Task, (const signed char * ) "COM", 1024 * 4, NULL, 2, &com_task);
	}
	else if (buffer == 0) {
		if (com_task != NULL) {
			vTaskDelete(com_task);
			com_task = NULL;
		}
	}
	return CMD_ERROR_NONE;
}
int ir(struct command_context * ctx) {

	unsigned int buffer;
	int len;
	uint8_t *script;
	if (ctx->argc < 2) {
		return CMD_ERROR_SYNTAX;
	}
	if (sscanf(ctx->argv[1], "%u", &buffer) != 1) {
		return CMD_ERROR_SYNTAX;
	}
	len = inms_script_length_flash((int)buffer);
	script = malloc(len);
	printf("slot = %d  length = %d\n", buffer, len);
	if (inms_script_read_flash(buffer, len, script) == No_Error) {
		hex_dump(script, len);
	}
	else {
		return CMD_ERROR_FAIL;
	}
	free(script);
	return CMD_ERROR_NONE;
}
// Simulate receive telecommand and execute it
int ct(struct command_context * ctx) {

	unsigned int serviceType;
	unsigned int serviceSubType;
	unsigned int buffers[200];
	uint8_t para[50] = {0};

	if (ctx->argc < 3) {
		return CMD_ERROR_SYNTAX;
	}
	if (sscanf(ctx->argv[1], "%u", &serviceType) != 1) {
		return CMD_ERROR_SYNTAX;
	}
	if (sscanf(ctx->argv[2], "%u", &serviceSubType) != 1) {
		return CMD_ERROR_SYNTAX;
	}
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
	case T31_photo_processing:
		decodeService31(serviceSubType, para);
		break;
	case T32_image_upload:
		decodeService32(serviceSubType, para);
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
	lastCommandTime = t.tv_sec;
	printf("\n");
	return CMD_ERROR_NONE;
}

int T_test(struct command_context * ctx) {
	extern void Thermal_Task(void * pvParameters);
	xTaskHandle T_task;
	int mode;

	if (ctx->argc != 2)
		return CMD_ERROR_SYNTAX;

	if (sscanf(ctx->argv[1], "%u", &mode) != 1)
		return CMD_ERROR_SYNTAX;

	if (mode == 1)
		if (T_task == NULL) {
			xTaskCreate(Thermal_Task, (const signed char *) "T_Test", 1024 * 4, NULL, 2, &T_task);
		}

	if (mode == 0)
		if (T_task != NULL) {
			vTaskDelete(T_task);
			T_task = NULL;
		}

	return CMD_ERROR_NONE;
}

int shutdown_tm(struct command_context * ctx) {

	int off_on;
	if (ctx->argc != 2)
		return CMD_ERROR_SYNTAX;
	if (sscanf(ctx->argv[1], "%u", &off_on) != 1)
		return CMD_ERROR_SYNTAX;
	if (off_on == 1) {
		parameters.shutdown_flag = 1;
		para_w_flash();
		printf("Shutdown Command Detected!! \r\n");
	}
	else if (off_on == 0) {
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
	printf("crippled_Mode\t\t\t%d\n", (int) parameters.crippled_Mode);
	printf("use_IFB\t\t\t\t%d\n", (int) parameters.use_IFB);
	printf("GS_threshold\t\t\t%d\n", (int) parameters.GS_threshold);
	printf("last_part_number\t\t%d\n", (int) parameters.image_lastPartNum);
	printf("last_part_length\t\t%d\n", (int) parameters.image_lastPartLen);

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

int idleunlock(struct command_context * ctx) {
	idleunlocks = 1 ;
	return CMD_ERROR_NONE;
}

int testmode(struct command_context * ctx) {

	if (mode_task != NULL) {
		vTaskDelete(mode_task);
		mode_task = NULL;
	}
	if (bat_check_task != NULL) {
		vTaskDelete(bat_check_task);
		bat_check_task = NULL;
	}
	if (com_task != NULL) {
		vTaskDelete(com_task);
		com_task = NULL;
	}
	if (wod_task != NULL) {
		vTaskDelete(wod_task);
		wod_task = NULL;
	}
	if (beacon_task != NULL) {
		vTaskDelete(beacon_task);
		beacon_task = NULL;
	}

	if (init_task != NULL) {
		vTaskDelete(init_task);
		init_task = NULL;
	}
	if (adcs_task != NULL) {
		vTaskDelete(adcs_task);
		adcs_task = NULL;
	}
	if (seuv_task != NULL) {
		vTaskDelete(seuv_task);
		seuv_task = NULL;
	}
	if (eop_task != NULL) {
		vTaskDelete(eop_task);
		eop_task = NULL;
	}
	if (hk_task != NULL) {
		vTaskDelete(hk_task);
		hk_task = NULL;
	}

	if (inms_error_handle != NULL) {
		vTaskDelete(inms_error_handle);
		inms_error_handle = NULL;
	}
	if (inms_current_moniter != NULL) {
		vTaskDelete(inms_current_moniter);
		inms_current_moniter = NULL;
	}
	if (inms_task != NULL) {
		vTaskDelete(inms_task);
		inms_task = NULL;
	}
	if (inms_task_receive != NULL) {
		vTaskDelete(inms_task_receive);
		inms_task_receive = NULL;
	}

	if (schedule_task != NULL) {
		vTaskDelete(schedule_task);
		schedule_task = NULL;
	}
	if (seuv_cm_task != NULL) {
		vTaskDelete(seuv_cm_task);
		seuv_cm_task = NULL;
	}
	if (Anom_mon_task != NULL) {
		vTaskDelete(Anom_mon_task);
		Anom_mon_task = NULL;
	}

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
	int16_t rxdata_signed;
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
		rxdata_signed = (rxdata[0] << 8) + rxdata[1];
		results = rxdata_signed;
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

	uint8_t txdata;
	uint8_t val[com_rx_hk_length];
	uint16_t raw;
	txdata = 0x25;
	printf("transmitter: \n");
	if (i2c_master_transaction_2(0, com_tx_node, &txdata, 1, &val[0], 8, com_delay) == E_NO_ERR) {
		hex_dump(&val, 8);
		memcpy(&raw, &val[2], 2);
		printf("TX temp = %.3f\n", raw * (-0.0546) + 189.5522);
	}
	else
		printf("ERROR!!  Get no reply from COM TX \r\n");
	txdata = com_rx_hk;
	printf("Receiver: \n");
	if (i2c_master_transaction_2(0, com_rx_node, &txdata, 1, &val[0], com_rx_hk_length, com_delay) == E_NO_ERR) {
		hex_dump(&val, com_rx_hk_length);

		memcpy(&raw, &val[0], 2);
		printf("TX current\t%.3f mA\n", raw * (0.0897 ));
		memcpy(&raw, &val[4], 2);
		printf("RX current\t%.3f mA\n", raw * (0.0305));
		memcpy(&raw, &val[6], 2);
		printf("Voltage\t\t%.3f V\n", raw * (0.00488));
		memcpy(&raw, &val[8], 2);
		printf("Oscillator Temp\t%.3f\n", raw * (-0.0546) + 189.5522);
		memcpy(&raw, &val[10], 2);
		printf("Amplifier Temp\t%.3f\n", raw * (-0.0546) + 189.5522);
	}
	else
		printf("ERROR!!  Get no reply from COM RX\r\n");

	return CMD_ERROR_NONE;
}

command_t __root_command ph_commands[] = {

	{ .name = "finalCheck", .help = "PHOENIX: finalCheck", .handler = finalCheck, },
	{ .name = "gpstest", .help = "PHOENIX: gpstest", .handler = gpsTest, },
	{ .name = "ch", .help = "PHOENIX: ch [ON(1), OFF(0)]", .usage = "ch 1:AUTO 0:Manual", .handler = changeHeater, },
	{ .name = "pc", .help = "PHOENIX: pc [sub] [ON(1), OFF(0)]", .usage = "pc [sub] [ON(1), OFF(0)]", .handler = powerControl, },
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
	{ .name = "ir", .help = "PHOENIX: inms script read", .usage = "<buffer>" , .handler = ir, },
	{ .name = "ct", .help = "PHOENIX: simulate receiving a uplink command and execute it", .usage = "<type> <subtype> <data*N> " , .handler = ct, },
	{ .name = "T_test", .help = "PHOENIX: Activate/OFF Thermal Task,switch 1=on, 0 =off", .usage = "T_test <switch>", .handler = T_test, },
	{ .name = "shutdown_tm", .help = "PHOENIX: change transceiver standby mode", .usage = "shutdown_transmitter <switch>", .handler = shutdown_tm, },
	{ .name = "parawrite", .help = "PHOENIX: write para setting in FS", .handler = parawrite, },
	{ .name = "pararead", .help = "PHOENIX: read on board parameter setting in FS", .usage = "<partition(0, 1)>", .handler = pararead, },
	{ .name = "paradelete", .help = "PHOENIX: delete parameters.bin", .handler = paradelete, },
	{ .name = "T_data_del", .help = "PHOENIX: delete t_obc.bin t_inms.bin", .handler = T_data_del, },
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
