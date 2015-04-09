/**
 * Test code for Gomspace panels
 *
 * @author Johan De Claville Christiansen
 * Copyright 2010 GomSpace ApS. All rights reserved.
 */

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
#include "I2C_GPIO.h"
#include <io/nanomind.h>
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

static const char * name_map[LM70_MAP_SIZE] = { "A1", "A2", "A3", "A4", "A5", "A6" };
static int gyro_map[GYRO_MAP_SIZE] = { 1, 3, 5, 8, 10, 12 };
static int lm70_map[LM70_MAP_SIZE] = { 2, 4, 6, 9, 11, 13 };
//int max6675_cs  = 1;
int adcs_i2c(struct command_context * ctx){
	i2c_mpio_init();
	unsigned int rxnum = 0;
	unsigned int reg1;
	unsigned int reg2;
	if (sscanf(ctx->argv[1], "%u", &rxnum) != 1){
		printf("Should input some rx number");
	}
	if (sscanf(ctx->argv[2], "%u", &reg1) != 1){
		printf("Should input reg1");
	}
	if (sscanf(ctx->argv[3], "%u", &reg2 )!= 1){
		printf("Should input reg2");
	}
	printf("%d\n",rxnum );
	printf("%u\n", reg1);
	printf("%u\n", reg2);
	send_i2c_adcs(reg1,reg2,rxnum);
	return CMD_ERROR_NONE;
}
int jumpTime(struct command_context * ctx){

	timestamp_t t;
	t.tv_sec = 0;
	t.tv_nsec =0 ;
	obc_timesync(&t, 1000);
	unsigned int waitingTime = 0;
		/* Set time in lib-c */

	if (sscanf(ctx->argv[1], "%u", &waitingTime) != 1){
		printf("Should input some time");
	}
	t.tv_sec += waitingTime;
	obc_timesync(&t, 1000);
	printf("\n");
	return CMD_ERROR_NONE;
}

int jump_mode(struct command_context * ctx) {
	int mode;

	if (ctx->argc != 2)
		return CMD_ERROR_SYNTAX;


	if (sscanf(ctx->argv[1], "%u", &mode) != 1)
		return CMD_ERROR_SYNTAX;

	mode_status_flag = (uint8_t)mode;



	return CMD_ERROR_NONE;
}

int ccsds_send(struct command_context * ctx) {

	uint8_t hkdata[250];
	uint8_t txframe[246];
    uint8_t tx_length;
    tx_length=254;
    for(int a =1;a<233;a++)
    	hkdata[a]=a;

    hkdata[0]=wod_sid;
    uint8_t err;
    err= CCSDS_GenerateTelemetryPacket(&txframe[1],&tx_length,wod_apid, 15,1, hkdata, 229);
 printf("txlen was 254 from CCSDS = %d\n",tx_length);

    if(err==ERR_SUCCESS){
      txframe[0]= com_tx_send;
      tx_length= tx_length+1;
    i2c_master_transaction(0, com_tx_node, &txframe, tx_length, 0, 0, 0);
      printf("Send CCSDS packet success\n");
      hex_dump(&txframe, tx_length);
    }else
	printf("have error on generate CCSDS packet \n");

	return CMD_ERROR_NONE;
}

int parawrite(struct command_context * ctx) {

	para_w();
	return CMD_ERROR_NONE;
}

int pararead(struct command_context * ctx) {

	para_r();
	printf("First Flight %d\n", (int) parameters.first_flight);
	printf("shutdown_flag %d\n", (int) parameters.shutdown_flag);
	printf("adcs_function %d\n", (int) parameters.adcs_function);
	printf("inms_function %d\n", (int) parameters.inms_function);
	printf("com_function %d\n", (int) parameters.com_function);
	printf("seuv_function %d\n", (int) parameters.seuv_function);
	printf("gps_function %d\n", (int) parameters.gps_function);
	printf("wod_store_count %d\n", (int) parameters.wod_store_count);
	printf("inms_store_count %d\n", (int) parameters.inms_store_count);
	printf("seuv_store_count %d\n", (int) parameters.seuv_store_count);
	printf("hk_store_count %d\n", (int) parameters.hk_store_count);
	printf("obc_packet_sequence_count %d\n", (int) parameters.obc_packet_sequence_count);
	printf("inms_packet_sequence_count %d\n", (int) parameters.inms_packet_sequence_count);
	printf("seuv_sequence_count %d\n", (int) parameters.seuv_packet_sequence_count);
	printf("wod_sequence_count %d\n", (int) parameters.wod_packet_sequence_count);
	printf("phoenix_hk_sequence_count %d\n", (int) parameters. phoenix_hk_packet_sequence_count);

	printf("vbat_recover_threshold %d\n", (int) parameters.vbat_recover_threshold);
	printf("vbat_safe_threshold %d\n", (int) parameters.vbat_safe_threshold);

	return CMD_ERROR_NONE;
}

int paradelete(struct command_context * ctx) {
	para_d();
	return CMD_ERROR_NONE;
}
int data_DUMP(struct command_context * ctx) {
	int type;
	if (ctx->argc != 2)
		return CMD_ERROR_SYNTAX;


	if (sscanf(ctx->argv[1], "%u", &type) != 1)
		return CMD_ERROR_SYNTAX;


	if(type==1)
		inms_data_dump();
    if(type==2)
    	wod_data_dump();
    if(type==3)
    	 seuv_data_dump();
    if(type==4)
    	hk_data_dump();
	return CMD_ERROR_NONE;
}

int alldatadelete(struct command_context * ctx) {

	para_d();
	inms_data_delete();
	wod_delete();
	seuv_delete();
	hk_delete();
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
	parameters.first_flight = 0;
	return CMD_ERROR_NONE;
}

int seuvread(struct command_context * ctx) {

	uint8_t val[5];

	if (i2c_master_transaction(0,seuv_node, 0, 0, &val, 5, seuv_delay) == E_NO_ERR) {
		hex_dump(&val, 5);
	} else
		printf("ERROR!!  Get no reply from SEUV \r\n");

	return CMD_ERROR_NONE;
}
int i2cread(struct command_context * ctx) {

	unsigned int node;
	unsigned int rx;
	i2c_frame_t * frame = csp_buffer_get(I2C_MTU);
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

	frame->dest = node;
	//	frame->len = txlen;
	frame->len_rx = rx;

	if (i2c_receive(0, &frame, 4) != E_NO_ERR) {

		printf("ERROR!!  Get no reply from COM \r\n");
		return CMD_ERROR_SYNTAX;
	}
	hex_dump(&frame, 10);
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

	if (i2c_master_transaction(0, addra, &txdata, 1, 0, 0,seuv_delay) == E_NO_ERR) {
		printf("configured SEUV: %x\r\n", node);
	} else
		printf("ERROR!!  Get no reply from COM \r\n");

	return CMD_ERROR_NONE;
}

int comhk2(struct command_context * ctx) {

	uint8_t txdata= com_tx_hk;
	uint8_t val;

	if (i2c_master_transaction(0,com_tx_node, &txdata, 1, &val, 1,com_delay) == E_NO_ERR) {
		hex_dump(&val, 1);
	} else
		printf("ERROR!!  Get no reply from COM \r\n");

	return CMD_ERROR_NONE;
}
int comhk(struct command_context * ctx) {

	uint8_t txdata= com_rx_hk;
	uint8_t val[16];

	if (i2c_master_transaction(0,com_rx_node, &txdata, 1, &val, 16,com_delay) == E_NO_ERR) {
		hex_dump(&val, 16);
	} else
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

struct command gyro_commands[] = { { .name = "setup", .help = "Gyro setup",
		.handler = cmd_gyro_setup, }, { .name = "test", .help = "Gyro test",
		.handler = cmd_gyro_test, }, { .name = "null", .help = "Gyro autonull",
		.handler = cmd_gyro_autonull, }, { .name = "offset", .help =
		"Gyro offset", .handler = cmd_gyro_write_offset, }, };

struct command lm70_commands[] = { { .name = "init", .help = "LM70 setup",
		.handler = cmd_lm70_init, }, { .name = "test", .help = "LM70 test",
		.handler = cmd_lm70_test, }, };

struct command panels_subcommands[] = { { .name = "init", .help = "Panels init",
		.handler = cmd_panels_init, }, { .name = "test", .help = "Panels test",
		.handler = cmd_panels_test, }, { .name = "cont", .help =
		"Panels test (cont)", .handler = cmd_panels_test_cont, }, };

struct command __root_command panels_commands[] = {
		{ .name = "ccsds_send",.help = "send a ccsds packet with 1~49 content", .handler = ccsds_send, },
		{ .name = "parawrite",.help = "write para setting in FS", .handler = parawrite, },
		{ .name ="pararead", .help = "read on board parameter setting in FS", .handler =pararead, },
		{ .name = "paradelete", .help = "delete parameters.bin",.handler = paradelete, }
		,{ .name = "alldatadelete", .help = "delete all on board data.bin",.handler = alldatadelete, }
		,{ .name = "inmsdatadelete", .help = "delete inmsdata.bin",.handler = inmsdatadelete, }
		,{ .name = "woddelete", .help = "delete wod.bin",.handler = woddelete, }
		,{ .name = "seuvdelete", .help = "delete seuv.bin",.handler = seuvdelete, }
		,{ .name = "hkdelete", .help = "delete hk.bin",.handler = hkdelete, }
		,{ .name = "jump_mode", .help = "jump_mode [mode] // 0=safe mode, 2=adcs mode,3=payload mode",.handler = jump_mode, }
		,{ .name = "jt", .help = "jt [sec]",.handler = jumpTime, }
		,{ .name = "ai2c", .help = "ai2c [par] [par] [rxnum]",.handler = adcs_i2c, }
		,{ .name = "data_dump", .help = "onboard data_dump [type] // 1=inms 2=wod 3=seuv 4=hk",.handler = data_DUMP, }
		, { .name = "idleunlock", .help =
		"skip idle 30m step", .handler = idleunlock, }, { .name = "seuvwrite",
		.help = "configure SEUV", .usage = "<node>", .handler = seuvwrite, }, {
		.name = "seuvread", .help = "Take data from a configured Channel",
		.handler = seuvread, },
		{ .name = "i2cread", .help = "Take data from i2c Channel", .usage =
				"<node> <rx>", .handler = i2cread, }, { .name = "comhk", .help =
				"retrive com hk data ", .handler = comhk, }, { .name = "comhk2",
				.help = "retrive com transmitter state ", .handler = comhk2, },
		{ .name = "gyro", .help = "Gyro commands", .chain =
		INIT_CHAIN(gyro_commands), }, { .name = "lm70", .help = "LM70 commands",
				.chain = INIT_CHAIN(lm70_commands), }, { .name = "panels",
				.help = "Panels commands", .chain =
				INIT_CHAIN(panels_subcommands), }, };

void cmd_panels_setup(void) {
	command_register(panels_commands);
}

