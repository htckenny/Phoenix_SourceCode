/*
 * subsystem.c
 *
 *  Created on: 2015/3/18
 *      Author: rusei
 */

#include <io/nanopower2.h>
#include <dev/i2c.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <math.h>
#include <csp/csp_endian.h>
#include <csp/csp.h>
#include <dev/arm/ds1302.h>
#include <util/timestamp.h>
#include <nanomind.h>

#include "subsystem.h"
#include "parameter.h"
#include "fs.h"
#include "../lib/liba712/src/drivers/mpio.h"

extern int TS5();
extern int TS6();
extern int TS7();
extern int TS9();
int status_update() {
	if (mode_task != NULL)
		status_frame.mode_task = 1;
	if (bat_check_task != NULL)
		status_frame.bat_check_task = 1;
	if (com_task != NULL)
		status_frame.com_task = 1;
	if (wod_task != NULL)
		status_frame.wod_task = 1;
	if (beacon_task != NULL)
		status_frame.beacon_task = 1;

	if (init_task != NULL)
		status_frame.init_task = 1;
	if (adcs_task != NULL)
		status_frame.adcs_task = 1;
	if (seuv_task != NULL)
		status_frame.seuv_task = 1;
	if (eop_task != NULL)
		status_frame.eop_task = 1;
	if (hk_task != NULL)
		status_frame.hk_task = 1;

	if (inms_error_handle != NULL)
		status_frame.inms_error_handle = 1;
	if (inms_current_moniter != NULL)
		status_frame.inms_current_moniter = 1;
	if (inms_task != NULL)
		status_frame.inms_task = 1;
	if (inms_task_receive != NULL)
		status_frame.inms_task_receive = 1;

	if (schedule_task != NULL)
		status_frame.schedule_task = 1;
	if (seuv_cm_task != NULL)
		status_frame.seuv_cm_task = 1;

	return E_NO_ERR;
}

uint32_t get_time() {

	unsigned long time;
	struct ds1302_clock clock;
	if (ds1302_clock_read_burst(&clock) < 0)
		return 0;

	if (ds1302_clock_to_time((time_t *) &time, &clock) < 0)
		return 0;

//	printf("Time is: %s", ctime((time_t *) &time));
//	printf("Time is: %lu",time);

	return (uint32_t)time;

}

int parameter_init() {
	abort_transfer_flag 				= 0;
	magnetometer_deploy					= 0;
	HK_frame.sun_light_flag				= 0;
	SD_lock_flag						= 0;
	use_GPS_header						= 0;
	inms_tm_status						= 1;
	

	/*--File System store count--*/
	parameters.wod_store_count			= 0;
	parameters.inms_store_count			= 0;
	parameters.seuv_store_count			= 0;
	parameters.hk_store_count			= 0;
	parameters.eop_store_count			= 0;

	/* Protocol sequence count */
	parameters.obc_packet_sequence_count = 0;
	parameters.ax25_sequence_count       = 0;
	parameters.tc_count                  = 0;

	/* System Configuration */
	parameters.first_flight				= 1;		/* indicates if this is the first flight */
	parameters.shutdown_flag			= 0;
	parameters.ant_deploy_flag			= 0;		/* indicates that antenna already deployed or not */
	parameters.hk_collect_period		= 60;
	parameters.beacon_period			= 30;
	parameters.reboot_count				= 0;
	parameters.com_bit_rates			= 0x01;

	/*  seuv related  */
	parameters.seuv_period				= 8;
	parameters.seuv_sample_rate			= 50;

	parameters.seuv_ch1_G1_conf			= 0x98;		/* Gain 1 configuration */
	parameters.seuv_ch2_G1_conf			= 0xB8;
	parameters.seuv_ch3_G1_conf			= 0xD8;
	parameters.seuv_ch4_G1_conf			= 0xF8;

	parameters.seuv_ch1_G8_conf			= 0x9B;		/* Gain 8 configuration */
	parameters.seuv_ch2_G8_conf			= 0xBB;
	parameters.seuv_ch3_G8_conf			= 0xDB;
	parameters.seuv_ch4_G8_conf			= 0xFB;
	parameters.seuv_mode				= 0x03;

	/* battery*/
	parameters.vbat_recover_threshold	= 7500;
	parameters.vbat_safe_threshold		= 7000;

	parameters.INMS_timeout 			= 400;
	parameters.inms_status				= 1;
	parameters.SD_partition_flag		= 0;

	parameters.ErrSerialNumber			= 0;

	seuvFrame.samples = parameters.seuv_sample_rate << 1 ;		/* samples */
	/* 0 1 2 3 4 5 6 |  7    */
	/*  sample rate  | Gain  */

	/* Read last parameter from FLASH */
	if (para_r_flash() == No_Error) {
		parameters.reboot_count ++;
		para_w_flash();
		return No_Error;
	}
	else
		para_w_flash();
	return Error;
}

void deploy_antenna() {
	uint8_t txdata[2];

	txdata[0] = ant_arm;    // arm ant board
	i2c_master_transaction_2(0, ant_node, &txdata, 1, 0, 0, 0);
	vTaskDelay(0.1 * delay_time_based);
	txdata[0] = ant_deploy;    // deploy ant board one by one
	txdata[1] = ant_deploy_timeout;
	i2c_master_transaction_2(0, ant_node, &txdata, 2, 0, 0, 0);
	printf("Antenna Deployed!!\n");
}


void power_control(int device, int stats) {
	/**
	 * Device code:
	 * 1: ADCS
	 * 2: GPS
	 * 3: SEUV
	 * 4: INMS
	 * 5: Interface Board
	 *
	 * Status =>
	 * 1: ON
	 * 0: OFF
	 */

	//   unsigned int mode = stats;
	printf("power %d with device %d\n", stats, device);
	eps_output_set_single_req eps_switch;
	eps_switch.mode = (uint8_t)stats;
	eps_switch.delay = 0;

	uint8_t txdata[100];
	txdata[0] = EPS_PORT_SET_SINGLE_OUTPUT;
	/* ADCS  */
	if (device == 1) {
		/* channel 0 = ADCS 5V */
		eps_switch.channel = 0;
		memcpy(&txdata[1], &eps_switch, sizeof(eps_output_set_single_req));
		i2c_master_transaction_2(0, eps_node, &txdata, 1 + sizeof(eps_output_set_single_req), 0, 0, eps_delay);

		/* channel 3 = ADCS 3.3V */
		eps_switch.channel = 3;
		memcpy(&txdata[1], &eps_switch, sizeof(eps_output_set_single_req));
		i2c_master_transaction_2(0, eps_node, &txdata, 1 + sizeof(eps_output_set_single_req), 0, 0, eps_delay);
	}
	/* GPS  */
	else if (device == 2) {
		/* channel 4 = GPS 3.3V */
		eps_switch.channel = 4;

		memcpy(&txdata[1], &eps_switch, sizeof(eps_output_set_single_req));
		i2c_master_transaction_2(0, eps_node, &txdata, 1 + sizeof(eps_output_set_single_req), 0, 0, eps_delay);
	}
	/* SEUV */
	else if (device == 3) {
		if (stats == ON) {
			/* channel 2 = SEUV 5V */
			eps_switch.channel = 2;
			memcpy(&txdata[1], &eps_switch, sizeof(eps_output_set_single_req));
			i2c_master_transaction_2(0, eps_node, &txdata, 1 + sizeof(eps_output_set_single_req), 0, 0, eps_delay);

			/* channel 5 = SEUV 3.3V */
			eps_switch.channel = 5;
			memcpy(&txdata[1], &eps_switch, sizeof(eps_output_set_single_req));
			i2c_master_transaction_2(0, eps_node, &txdata, 1 + sizeof(eps_output_set_single_req), 0, 0, eps_delay);
		}
		else if (stats == OFF) {
			/* channel 5 = SEUV 3.3V */
			eps_switch.channel = 5;
			memcpy(&txdata[1], &eps_switch, sizeof(eps_output_set_single_req));
			i2c_master_transaction_2(0, eps_node, &txdata, 1 + sizeof(eps_output_set_single_req), 0, 0, eps_delay);

			/* channel 2 = SEUV 5V */
			eps_switch.channel = 2;
			memcpy(&txdata[1], &eps_switch, sizeof(eps_output_set_single_req));
			i2c_master_transaction_2(0, eps_node, &txdata, 1 + sizeof(eps_output_set_single_req), 0, 0, eps_delay);
		}
	}
	/* INMS */
	else if (device == 4) {
		/*      INMS Power GPIO Control        */
		if (stats == ON) {
			io_set(5);
			vTaskDelay(0.3 * delay_time_based);
			io_set(1);
			vTaskDelay(1 * delay_time_based);
			io_clear(5);
			io_clear(1);
		}
		else if (stats == OFF) {
			io_set(6);
			vTaskDelay(0.3 * delay_time_based);
			io_set(0);
			vTaskDelay(1 * delay_time_based);
			io_clear(6);
			io_clear(0);
		}
	}
	/* Interface Board */
	else if (device == 5) {
		if (stats == ON)
			io_set(2);
		else if (stats == OFF)
			io_clear(2);
	}
}
/** Power off all the subsystems */

void power_OFF_ALL() {
	power_control(1, OFF);
	power_control(2, OFF);
	power_control(3, OFF);
	power_control(4, OFF);
}

/* Helper function */
uint16_t Interface_tmp_get() {
	uint8_t rx[5];
	uint8_t tx[2];
	tx[0] = 0xF0;	//0d240

	i2c_master_transaction_2(0, interface_node, &tx, 1, 0, 0, interface_delay) ;
	vTaskDelay(0.01 * delay_time_based);
	if (i2c_master_transaction_2(0, interface_node, &tx , 1, &rx, 4, interface_delay) == E_NO_ERR) {
		return (rx[0] << 8) + rx[1];
	}
	else
		return 0;
}
uint16_t Interface_inms_thermistor_get() {
	uint8_t rx[5];
	uint8_t tx[2];
	tx[0] = 0xD0;	//0d208

	i2c_master_transaction_2(0, interface_node, &tx, 1, 0, 0, interface_delay) ;
	vTaskDelay(0.01 * delay_time_based);
	if (i2c_master_transaction_2(0, interface_node, &tx, 1, &rx, 4, interface_delay) == E_NO_ERR) {
		return (rx[0] << 8) + rx[1];
	}
	else
		return 0;
}
uint16_t Interface_3V3_current_get() {
	uint8_t rx[5];
	uint8_t tx[2];
	tx[0] = 0x90;	//0d144
	i2c_master_transaction_2(0, interface_node, &tx, 1, 0, 0, interface_delay) ;
	vTaskDelay(0.01 * delay_time_based);
	if (i2c_master_transaction_2(0, interface_node, &tx, 1, &rx, 4, interface_delay) == E_NO_ERR) {
		return (rx[0] << 8) + rx[1];
	}
	else
		return 0;
}
uint16_t Interface_5V_current_get() {
	uint8_t rx[5];
	uint8_t tx[2];
	tx[0] = 0xB0;	//0x176
	i2c_master_transaction_2(0, interface_node, &tx, 1, 0, 0, interface_delay) ;
	vTaskDelay(0.01 * delay_time_based);
	if (i2c_master_transaction_2(0, interface_node, &tx , 1, &rx, 4, interface_delay) == E_NO_ERR) {
		return (rx[0] << 8) + rx[1];
	}
	else
		return 0;
}

void generate_Error_Report(int type) {
	uint8_t errPacket [10];
	uint8_t txbuf[2];
	uint8_t rxbuf[133];
	uint16_t sub_current, sub_temperature;
	timestamp_t t;

	t.tv_sec = 0;
	t.tv_nsec = 0;
	obc_timesync(&t, 6000);

	if (HK_frame.mode_status_flag == 2 && parameters.first_flight == 1)
		HK_frame.mode_status_flag = 4;

	memcpy(&errPacket[0], &t.tv_sec, 4);
	memcpy(&errPacket[4], &parameters.ErrSerialNumber, 2);
	memcpy(&errPacket[6], &type, 1);
	memcpy(&errPacket[7], &HK_frame.mode_status_flag, 1);

	switch (type) {
	/* Low Battery Condition */
	case 1 :
		txbuf[0] = 0x08;
		if (i2c_master_transaction_2(0, eps_node, &txbuf, 1, &rxbuf, 43 + 2, eps_delay) == E_NO_ERR) {
			memcpy(&errPacket[8], &rxbuf[10], 2);
		}
		break;
	/* ADCS 3.3V current too high */
	case 2 :
		txbuf[0] = 0x08;
		txbuf[1] = 0x00;
		if (i2c_master_transaction_2(0, eps_node, &txbuf, 2, &rxbuf, 133, eps_delay) == E_NO_ERR) {
			memcpy(&errPacket[8], &rxbuf[28], 2); // curout[3] ADCS 3.3V
		}
		break;
	/* ADCS 5V current too high */
	case 3 :
		txbuf[0] = 0x08;
		txbuf[1] = 0x00;
		if (i2c_master_transaction_2(0, eps_node, &txbuf, 2, &rxbuf, 133, eps_delay) == E_NO_ERR) {
			memcpy(&errPacket[8], &rxbuf[22], 2); // curout[0] ADCS 5V
		}
		break;
	/* GPS current too high */
	case 4 :
		txbuf[0] = 0x08;
		txbuf[1] = 0x00;
		if (i2c_master_transaction_2(0, eps_node, &txbuf, 2, &rxbuf, 133, eps_delay) == E_NO_ERR) {
			memcpy(&errPacket[8], &rxbuf[30], 2); // curout[4] GPS
		}
		break;
	/* SEUV 3.3V current too high */
	case 5 :
		txbuf[0] = 0x08;
		txbuf[1] = 0x00;
		if (i2c_master_transaction_2(0, eps_node, &txbuf, 2, &rxbuf, 133, eps_delay) == E_NO_ERR) {
			memcpy(&errPacket[8], &rxbuf[32], 2); // curout[5] SEUV 3.3V
		}
		break;
	/* SEUV 5V current too high */
	case 6 :
		txbuf[0] = 0x08;
		txbuf[1] = 0x00;
		if (i2c_master_transaction_2(0, eps_node, &txbuf, 2, &rxbuf, 133, eps_delay) == E_NO_ERR) {
			memcpy(&errPacket[8], &rxbuf[26], 2); // curout[2] SEUV 5V
		}
		break;
	/* INMS 3.3V current too high */
	case 7 :
		sub_current = Interface_3V3_current_get();
		memcpy(&errPacket[8], &sub_current, 2);
		break;
	/* INMS 5V current too high */
	case 8 :
		sub_current = Interface_5V_current_get();
		memcpy(&errPacket[8], &sub_current, 2);
		break;
	/* OBC temperature out of range */
	case 9 :
		TS9();
		memcpy(&errPacket[8], &ThermalFrame.T9, 2);
		break;
	/* COM temperature out of range */
	case 10 :
		TS5();
		memcpy(&errPacket[8], &ThermalFrame.T5, 2);
		break;
	/* Antenna temperature out of range */
	case 11 :
		TS6();
		memcpy(&errPacket[8], &ThermalFrame.T6, 2);
		break;
	/* IFB temperature out of range */
	case 12 :
		sub_temperature = Interface_tmp_get();
		memcpy(&errPacket[8], &sub_temperature, 2);
		break;
	/* ADCS temperature out of range */
	case 13 :
		TS7();
		memcpy(&errPacket[8], &ThermalFrame.T7, 2);
		break;
	/* INMS temperature out of range */
	case 14 :
		sub_temperature = Interface_inms_thermistor_get();
		memcpy(&errPacket[8], &sub_temperature, 2);
		break;
	}
	if (errPacket_write(errPacket) == No_Error) {
		parameters.ErrSerialNumber ++;
		para_w_flash();
	}
	else {
		printf("error write into errpacket\n");
	}
}
