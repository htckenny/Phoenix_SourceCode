/**
 * Test code for Gomspace panels
 *
 * @author Johan De Claville Christiansen
 * Copyright 2010 GomSpace ApS. All rights reserved.
 */
#include <io/nanomind.h>
#include <util/hexdump.h>
#include <util/timestamp.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "parameter.h"
#include <util/console.h>
#include <dev/spi.h>
#include <dev/gyro.h>
#include <dev/lm70.h>
#include <dev/max6675.h>
#include <dev/usart.h>
//----------------------------------
#include <freertos/semphr.h>
#include <dev/i2c.h>
#include <io/nanopower2.h>
#include <fat_sd/ff.h>
#include "fs.h"
#include <csp/csp.h>
#include <csp/csp_endian.h>
#define E_NO_ERR -1

#include "parameter.h"
#include "subsystem.h"
#include "Tele_function.h"
//------------------------------------
extern spi_dev_t spi_dev;

/**
 * Chip selects:
 *
 * Ax	Low		High
 * -----------------
 * A1	5		6
 * A2	3		4
 * A3	1		2
 * A4	12		13
 * A5	10		11
 * A6	8		9
 *
 * Dataflash	0
 * SD-Card		15
 */

#define GYRO_MAP_SIZE	6
#define LM70_MAP_SIZE	6
#define NAME_MAP_SIZE	6

static spi_chip_t gyro_chip[GYRO_MAP_SIZE];
static spi_chip_t lm70_chip[LM70_MAP_SIZE];
//static spi_chip_t max6675_chip;

static const char * name_map[LM70_MAP_SIZE] = { "A1", "A2", "A3", "A4", "A5",
                                                "A6"
                                              };
static int gyro_map[GYRO_MAP_SIZE] = { 1, 3, 5, 8, 10, 12 };
static int lm70_map[LM70_MAP_SIZE] = { 2, 4, 6, 9, 11, 13 };
//int max6675_cs  = 1;


int telecom(struct command_context * ctx){
	unsigned int buffer;
	extern void Telecom_Task(void * pvParameters);
	if (ctx->argc < 2) {
		return CMD_ERROR_SYNTAX;
	}
	if (sscanf(ctx->argv[1], "%u", &buffer) != 1) {
		return CMD_ERROR_SYNTAX;
	}
	if (buffer == 1){
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
	len = inms_script_length((int)buffer);
	uint8_t script[len] ;
	// for (int i = 0 ; i < len ; i++){
	// 	script[i] = 0;
	// }
	
	// printf("%d\n",len );
	inms_script_read(buffer, len, &script);
	// printf("%d\n", script[304]);
	hex_dump(&script, len);
	
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
	para[5] = 5 + (ctx->argc) - 3;
	para[7] = serviceType;
	para[8] = serviceSubType;

	switch (serviceType) {

	case T3_SYS_CONF:
		decodeService3(serviceSubType, para);
		break;
	case T8_function_management:
		decodeService8(serviceSubType, para);
		break;
	case T11_OnBoard_Sche:
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
	if (range == 1){
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
		if (schedule_reset() == 1) {
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
		for (int i = 0; i < 4 ;i++){
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
	para_r();

	if (schedule_write(telecommand) == 1) {
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
		vTaskDelay(10000);
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
		para_w();
		printf("Shutdown Command Detected!! \r\n");

	}

	if (off_on == 0)
	{
		parameters.shutdown_flag = 0;
		para_w();
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
	para_w();
	return CMD_ERROR_NONE;
}

int pararead(struct command_context * ctx) {

	para_r();
	printf("First Flight \t\t\t%d\n", (int) parameters.first_flight);
	printf("shutdown_flag \t\t\t%d\n", (int) parameters.shutdown_flag);
	printf("ant_deploy_flag \t\t%d\n", (int) parameters.ant_deploy_flag);
	printf("wod_store_count \t\t%d\n", (int) parameters.wod_store_count);
	printf("inms_store_count \t\t%d\n", (int) parameters.inms_store_count);
	printf("seuv_store_count \t\t%d\n", (int) parameters.seuv_store_count);
	printf("hk_store_count \t\t\t%d\n", (int) parameters.hk_store_count);
	printf("obc_packet_sequence_count \t%d\n", (int) parameters.obc_packet_sequence_count);
	printf("vbat_recover_threshold \t\t%d\n", (int) parameters.vbat_recover_threshold);
	printf("vbat_safe_threshold \t\t%d\n", (int) parameters.vbat_safe_threshold);
	printf("schedule_series_number \t\t%d\n", (int) parameters.schedule_series_number);

	return CMD_ERROR_NONE;
}

int paradelete(struct command_context * ctx) {
	para_d();
	return CMD_ERROR_NONE;
}

int T_data_del(struct command_context * ctx) {
	T_data_d();
	return CMD_ERROR_NONE;
}
int data_DUMP(struct command_context * ctx) {
	int type;
	if (ctx->argc != 2)
		return CMD_ERROR_SYNTAX;


	if (sscanf(ctx->argv[1], "%u", &type) != 1)
		return CMD_ERROR_SYNTAX;


	if (type == 1)
		inms_data_dump();
	if (type == 2)
		wod_data_dump();
	if (type == 3)
		seuv_data_dump();
	if (type == 4)
		hk_data_dump();
	if (type == 5)
		thermal_data_dump();

	return CMD_ERROR_NONE;
}

int alldatadelete(struct command_context * ctx) {

	para_d();
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
	vTaskDelete(init_task);
	printf("Enter ground test mode, plz reboot the satellite if wants leave this mode \r\n");
	return CMD_ERROR_NONE;
}


int seuvread(struct command_context * ctx) {

	uint8_t val[5];

	if (i2c_master_transaction(0, seuv_node, 0, 0, &val, 5, seuv_delay) == E_NO_ERR) {
		hex_dump(&val, 5);
	} else
		printf("ERROR!!  Get no reply from SEUV \r\n");

	return CMD_ERROR_NONE;
}
int i2cread(struct command_context * ctx) {

	unsigned int node;
	unsigned int rx;

	if (ctx->argc != 3) {
		printf("i2c check point1 error\r\n");
		return CMD_ERROR_SYNTAX;
	}

	if (sscanf(ctx->argv[1], "%u", &node) != 1) {
		printf("i2c check point2 error\r\n");
		return CMD_ERROR_SYNTAX;
	}

	if (sscanf(ctx->argv[2], "%u", &rx) != 1) {
		printf("i2c check point2 error\r\n");
		return CMD_ERROR_SYNTAX;
	}
	i2c_frame_t * frame = csp_buffer_get(I2C_MTU);
	frame->dest = node;
	//	frame->len = txlen;
	frame->len_rx = rx;

	if (i2c_receive(0, &frame, 4) != E_NO_ERR) {
		printf("ERROR!!  Get no reply from COM \r\n");
		csp_buffer_free(frame);
		return CMD_ERROR_SYNTAX;
	}
	hex_dump(&frame, 10);
	csp_buffer_free(frame);
	return CMD_ERROR_NONE;
}
int seuvwrite(struct command_context * ctx) {
	unsigned int node;
	if (ctx->argc != 2) {
		return CMD_ERROR_SYNTAX;
	}

	if (sscanf(ctx->argv[1], "%u", &node) != 1) {
		return CMD_ERROR_SYNTAX;
	}

	uint8_t txdata = node;
	unsigned int addra = 110;

	if (i2c_master_transaction(0, addra, &txdata, 1, 0, 0, seuv_delay) == E_NO_ERR) {
		printf("configured SEUV: %x\r\n", node);
	} else
		printf("ERROR!!  Get no reply from COM \r\n");

	return CMD_ERROR_NONE;
}

int comhk2(struct command_context * ctx) {

	uint8_t txdata = com_tx_hk;
	uint8_t val;

	if (i2c_master_transaction(0, com_tx_node, &txdata, 1, &val, com_tx_hk_length, com_delay) == E_NO_ERR) {
		hex_dump(&val, com_tx_hk_length);
	} else
		printf("ERROR!!  Get no reply from COM \r\n");

	return CMD_ERROR_NONE;
}
int comhk(struct command_context * ctx) {

	uint8_t txdata = com_rx_hk;
	uint8_t val[com_rx_hk_length];

	if (i2c_master_transaction(0, com_rx_node, &txdata, 1, &val, com_rx_hk_length, com_delay) == E_NO_ERR) {
		hex_dump(&val, com_rx_hk_length);
	} 
	else
		printf("ERROR!!  Get no reply from COM \r\n");

	return CMD_ERROR_NONE;
}

//----------------------------------------------------------
int cmd_panels_init(struct command_context *ctx) {

	int i;

	for (i = 0; i < GYRO_MAP_SIZE; i++) {
		gyro_spi_setup_cs(&spi_dev, &gyro_chip[i], gyro_map[i]);
		gyro_setup(&gyro_chip[i]);
	}

	for (i = 0; i < LM70_MAP_SIZE; i++) {
		lm70_spi_setup_cs(&spi_dev, &lm70_chip[i], lm70_map[i]);
	}

	//max6675_spi_setup_cs(&spi_dev, &max6675_chip, max6675_cs);
	return CMD_ERROR_NONE;

}

int cmd_panels_test(struct command_context *ctx) {

	while (1) {
		int i;
		if (usart_messages_waiting(USART_CONSOLE) != 0)
			break;

		spi_chip_t * chip;
		for (i = 0; i < GYRO_MAP_SIZE; i++) {
			chip = &gyro_chip[i];
			printf("\E[2KGyro %d %s Angle\t\t\t %.3f [deg]\r\n", i, name_map[i],
			       gyro_read_angle(chip));
			printf("\E[2KGyro %d %s Rate\t\t\t %.3f [deg/sec]\r\n", i,
			       name_map[i], gyro_read_rate(chip));
		}

		printf("\E[2K\r\n");

		for (i = 0; i < LM70_MAP_SIZE; i++) {
			chip = &lm70_chip[i];
			printf("\E[2KLM70 %d %s Temp\t\t\t %.3f [C]\r\n", i, name_map[i],
			       lm70_read_temp(chip));
		}

		printf("\E[2K\r\n");

		//printf("\E[2KMax6675 Temp\t\t\t %.2f [C]\r\n", max6675_read_temp(&max6675_chip));

		vTaskDelay(0.30 * configTICK_RATE_HZ);

		printf("\E[20A\r");

	}
	return CMD_ERROR_NONE;

}

int cmd_panels_test_cont(struct command_context *ctx) {

	while (1) {
		int i;
		if (usart_messages_waiting(USART_CONSOLE) != 0)
			break;

		spi_chip_t * chip;
		for (i = 0; i < GYRO_MAP_SIZE; i++) {
			chip = &gyro_chip[i];
			printf(" %.4f, ", gyro_read_rate(chip));
		}

		vTaskDelay(0.10 * configTICK_RATE_HZ);

		printf("\n\r");
	}
	return CMD_ERROR_NONE;
}

int cmd_gyro_setup(struct command_context *ctx) {

	/* Setup the SPI device */
	extern spi_dev_t spi_dev;
	gyro_spi_setup_cs(&spi_dev, &gyro_chip[0], gyro_map[0]);

	/* Initialise gyro */
	gyro_setup(&gyro_chip[0]);
	return CMD_ERROR_NONE;

}

int cmd_gyro_test(struct command_context *ctx) {
	gyro_test(&gyro_chip[0]);
	return CMD_ERROR_NONE;
}

int cmd_gyro_autonull(struct command_context *ctx) {
	gyro_autonull(&gyro_chip[0]);
	return CMD_ERROR_NONE;
}

int cmd_gyro_write_offset(struct command_context *ctx) {

	char * args = command_args(ctx);
	float offset;

	if (args == NULL || sscanf(args, "%f", &offset) != 1) {
		printf("Gyro offset is %.4f [deg/sec]\r\n",
		       gyro_read_offset(&gyro_chip[0]));
	} else {
		printf("Setting offset %.4f [deg/sec]\r\n", offset);
		gyro_write_offset(&gyro_chip[0], offset);
	}

	return CMD_ERROR_NONE;

}

int cmd_lm70_init(struct command_context *ctx) {
	extern spi_dev_t spi_dev;
	lm70_spi_setup_cs(&spi_dev, &lm70_chip[0], lm70_map[0]);
	return CMD_ERROR_NONE;
}

int cmd_lm70_test(struct command_context *ctx) {

	while (1) {
		if (usart_messages_waiting(USART_CONSOLE) != 0)
			break;

		printf("Temp %f\r\n", lm70_read_temp(&lm70_chip[0]));
		vTaskDelay(100);
	}

	return CMD_ERROR_NONE;

}

struct command gyro_commands[] = { {
		.name = "setup", .help = "Gyro setup",
		.handler = cmd_gyro_setup,
	}, {
		.name = "test", .help = "Gyro test",
		.handler = cmd_gyro_test,
	}, {
		.name = "null", .help = "Gyro autonull",
		.handler = cmd_gyro_autonull,
	}, {
		.name = "offset", .help =
		"Gyro offset", .handler = cmd_gyro_write_offset,
	},
};

struct command lm70_commands[] = { {
		.name = "init", .help = "LM70 setup",
		.handler = cmd_lm70_init,
	}, {
		.name = "test", .help = "LM70 test",
		.handler = cmd_lm70_test,
	},
};

struct command panels_subcommands[] = { {
		.name = "init", .help = "Panels init",
		.handler = cmd_panels_init,
	}, {
		.name = "test", .help = "Panels test",
		.handler = cmd_panels_test,
	}, {
		.name = "cont", .help =
		"Panels test (cont)", .handler = cmd_panels_test_cont,
	},
};

struct command __root_command panels_commands[] = {
	{ .name = "tele", .help = "tele [ON = 1 / OFF = 0]", .handler = telecom, },
	{ .name = "jt", .help = "jt [sec]", .handler = jumpTime, },
	{
		.name = "schd",
		.help = "schedule_delete <range> <time_1> <time_2 (opt)>",
		.usage = "<range> <time_1> <time_2 (opt)>",
		.handler = scheduleDelete,
	},
	{
		.name = "schw",
		.help = "schedule_write <Absolute_time> <Type> <Subtype> <para>",
		.usage = "<Absolute_time> <Type> <Subtype> <para>",
		.handler = scheduleWrite,
	},
	{
		.name = "sch",
		.help = "schedule_command 1:reset, 2:dump, 3:shift, 4:delete" ,
		.usage = "sch <Command ID> <Para(optional)>",
		.handler = scheduleRelated,
	},
	{ .name = "ir", .help = " inms script read", .usage = "<buffer>" , .handler = ir, },
	{ .name = "ct", .help = " simulate receiving a uplink command and execute it", .usage = "<type> <subtype> <data*N> " , .handler = ct, },
	{ .name = "ccsds_send", .help = "send a ccsds packet every 10 seconds", .usage = "<type> <subtype> <data> " , .handler = ccsds_send, },
	{ .name = "telecomtest", .help = "telecomtest <packet_num> <packet_len>", .usage = "<packet_num> <packet_len>", .handler = telecomtest,	},
	{ .name = "T_test", .help = "Activate/OFF Thermal Task,switch 1=on, 0 =off", .usage = "T_test <switch>", .handler = T_test, },
	{ .name = "shutdown_tm", .help = "change transceiver standby mode", .usage = "shutdown_transmitter <switch>", .handler = shutdown_tm, },
	{ .name = "parawrite", .help = "write para setting in FS", .handler = parawrite, },
	{ .name = "pararead", .help = "read on board parameter setting in FS", .handler = pararead, },
	{ .name = "paradelete", .help = "delete parameters.bin", .handler = paradelete, },
	{ .name = "T_data_del", .help = "delete t_obc.bin t_inms.bin", .handler = T_data_del, },
	{ .name = "alldatadelete", .help = "delete all on board data.bin", .handler = alldatadelete, },
	{ .name = "inmsdatadelete", .help = "delete inmsdata.bin", .handler = inmsdatadelete, },
	{ .name = "woddelete", .help = "delete wod.bin", .handler = woddelete, }	,
	{ .name = "seuvdelete", .help = "delete seuv.bin", .handler = seuvdelete, }	,
	{ .name = "hkdelete", .help = "delete hk.bin", .handler = hkdelete, }	,
	{ .name = "jump_mode", .help = "jump_mode [mode] // 0=safe mode, 2=adcs mode,3=payload mode", .handler = jump_mode, },
	{ .name = "data_dump", .help = "onboard data_dump [type] , 1=inms 2=wod 3=seuv 4=hk 5=thermal ", .usage = "<type> 1=inms 2=wod 3=seuv 4=hk 5=thermal ", .handler = data_DUMP, },
	{ .name = "idleunlock", .help = "skip idle 30m step", .handler = idleunlock, },
	{ .name = "testmode", .help = "enter  testmode, please reboot satellite if want to leave this mode", .handler = testmode, },
	{ .name = "seuvwrite", .help = "configure SEUV", .usage = "<node>", .handler = seuvwrite, },
	{ .name = "seuvread", .help = "Take data from a configured Channel", .handler = seuvread, },
	{ .name = "i2cread", .help = "Take data from i2c Channel", .usage =	"<node> <rx>", .handler = i2cread, },
	{ .name = "comhk", .help = "retrive com hk data ", .handler = comhk, },
	{ .name = "comhk2",	.help = "retrive com transmitter state ", .handler = comhk2, },
	{ .name = "gyro", .help = "Gyro commands", .chain =	INIT_CHAIN(gyro_commands), },
	{ .name = "lm70", .help = "LM70 commands", .chain = INIT_CHAIN(lm70_commands), },
	{ .name = "panels",	.help = "Panels commands", .chain =	INIT_CHAIN(panels_subcommands), },
};

void cmd_panels_setup(void) {
	command_register(panels_commands);
}
