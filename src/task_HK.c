/*
 * HK_Task.c
 *
 *  Created on: 	2015/3/28
 *  Last update: 	2015/10/12
 *      Author: rusei, Kenny
 */
#include <util/timestamp.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <dev/cpu.h>
#include <util/delay.h>
 #include <nanomind.h>
#include <csp/csp_endian.h>
#include <dev/i2c.h>
#include <string.h>
#include <util/hexdump.h>
#include <time.h>
#include <dev/adc.h>
#include <dev/usart.h>
 #include <io/nanopower2.h>
#include <csp/csp.h>
 
#include "parameter.h"
#include "tele_function.h"
#include "subsystem.h"
#include "fs.h"


int num;
uint8_t hk_buffer[200];

/* Thermistor Sensor data */
void TS12_16() {

	int nums = 0;
	char uchar[174 * 10];
	nums = usart_messages_waiting(2);
	if (nums != 0) {
		for (int f = 0; f < nums; f++)
			uchar[f] = usart_getc(2);

		if ((uint8_t)uchar[0] == 0x0A) {
			memcpy(&Tdata[0], &num, 4);
			memcpy(&Tdata[4], &uchar[0], 174);
			thurmal_2_w();
			printf("Get a Side panel thermal sensor packet # %d \n", num);
			hex_dump(&Tdata[0], 178);
		}
	}
}

/* interface board  INMS temperature */
int TS11() {
	uint8_t rx[10];
	uint8_t tx[2];
	tx[0] = 0xD0;
	tx[1] = 0xD0;

	i2c_master_transaction(0, interface_node, &tx, 2, 0, 0, 0);
	i2c_master_transaction(0, interface_node, 0 , 0, &rx, 4, seuv_delay);

	if (i2c_master_transaction(0, interface_node, 0 , 0, &rx, 4, seuv_delay) != E_NO_ERR)
		return Error;

	// memcpy(&ThermalFrame.T10,&rx[0],3);
	ThermalFrame.T11 = (rx[0] << 8) + rx[1];


	return No_Error;
}

/* interface temperature */
int TS10() {
	uint8_t rx[10];

	uint8_t tx[2];
	tx[0] = 0xF0;
	tx[1] = 0xF0;


	i2c_master_transaction(0, interface_node, &tx, 2, 0, 0, 0);
	i2c_master_transaction(0, interface_node, 0 , 0, &rx, 4, seuv_delay);


	if (i2c_master_transaction(0, interface_node, 0 , 0, &rx, 4, seuv_delay) != E_NO_ERR)
		return Error;


	// memcpy(&ThermalFrame.T10,&rx[0],3);
	ThermalFrame.T10 = (rx[0] << 8) + rx[1];
	//ThermalFrame.T10=0x0A;

	return No_Error;
}

/* OBC temp */
int TS9() {
	uint16_t * adc_buffer;

	adc_buffer = adc_start_blocking(1);

	ThermalFrame.T9 = (uint16_t)((((adc_buffer[0] * 2493.0) / 1023) - 424) / 6.25);

	return No_Error;
}
/* GPS temp */
int TS8() {
	ThermalFrame.T8 = 0;
	return No_Error;
}
/* ADCS temp */
int TS7() {
	uint8_t txbuffer = 175;
	uint8_t rxbuffer[6];

	if (i2c_master_transaction(0, adcs_node, &txbuffer, 1, &rxbuffer, 6, adcs_delay) != E_NO_ERR)
		return Error;

	// ThermalFrame.T7 = (uint16_t)rxbuffer[5];
    memcpy(&ThermalFrame.T7, &rxbuffer[4], 2);
	return No_Error;
}

/* Antenna Board temp */ 
int TS6() {
	uint8_t txbuffer = 0xC0;
	uint16_t rxbuffer[2];

	if (i2c_master_transaction(0, ant_node, &txbuffer, 1, &rxbuffer, 2, com_delay) != E_NO_ERR)
		return Error;
	memcpy(&ThermalFrame.T6, &rxbuffer, 2);
	// ThermalFrame.T6 = rxbuffer;

	return No_Error;
}
/* COM board temp */
int TS5() {
	uint8_t txbuffer = com_rx_hk;
	uint16_t rxbuffer[7];

	if (i2c_master_transaction(0, com_rx_node, &txbuffer, 1, &rxbuffer, 14, com_delay) != E_NO_ERR)
		return Error;

	//rxbuffer=csp_ntoh16(rxbuffer);
	ThermalFrame.T5 = rxbuffer[5];
	//ThermalFrame.T5=0x05;

	return No_Error;
}

/* EPS temp */
int TS1_4() {
	eps_hk_t * chkparam;

	i2c_frame_t * frame = csp_buffer_get(I2C_MTU);
	frame->dest = eps_node;
	frame->data[0] = EPS_PORT_HK;
	frame->data[1] = 0;
	frame->len = 2;
	frame->len_rx = 2 + (uint8_t) sizeof(eps_hk_t);
	frame->retries = 0;

	if (i2c_send(0, frame, 0) != E_NO_ERR) {
		csp_buffer_free(frame);
		return Error;
	}
	if (i2c_receive(0, &frame, 20) != E_NO_ERR)
		return Error;

	chkparam = (eps_hk_t *)&frame->data[2];
	eps_hk_unpack(chkparam);
	csp_buffer_free(frame);

	ThermalFrame.T1 = chkparam->temp[0];
	ThermalFrame.T2 = chkparam->temp[1];
	ThermalFrame.T3 = chkparam->temp[2];
	ThermalFrame.T4 = chkparam->temp[3];


	return No_Error;

}
void clean_all() {
	ThermalFrame.T1 = 0;
	ThermalFrame.T2 = 0;
	ThermalFrame.T3 = 0;
	ThermalFrame.T4 = 0;
	ThermalFrame.T5 = 0;
	ThermalFrame.T6 = 0;
	ThermalFrame.T7 = 0;
	ThermalFrame.T8 = 0;
	ThermalFrame.T9 = 0;
	ThermalFrame.T10 = 0;
	ThermalFrame.T11 = 0;
}
void thermal_test(void * pvParameters) {
	int nums = 0;
	char uchar[174 * 10];
	num = 0;
	power_OFF_ALL();
	vTaskDelay(2 * delay_time_based);


	power_control(4, ON);
	power_control(1, ON);

	while (1) {

		num++;
		clean_all();
		ThermalFrame.packet_number = num;
		TS1_4();
		TS5();
		TS6();
		TS7();
		TS9();
		TS10();
		TS11();
		printf("------------------------------------- \n");
		printf("NUM = %d ,T1= %04X ,T2 %04X ,T3= %04X ,T4= %04X ,T5= %04X \n", (int)ThermalFrame.packet_number, ThermalFrame.T1, ThermalFrame.T2, ThermalFrame.T3, ThermalFrame.T4, ThermalFrame.T5);
		printf("T6= %04X ,T7 %04X ,T8= %04X ,T9= %04X ,T10= %04X \n", ThermalFrame.T6, ThermalFrame.T7, ThermalFrame.T8, ThermalFrame.T9, ThermalFrame.T10);

		thurmal_1_w();
		TS12_16();


		nums = usart_messages_waiting(2);
		if (nums != 0) {
			for (int f = 0; f < nums; f++)
				uchar[f] = usart_getc(2);
			hex_dump(&uchar, 174);

			timestamp_t t;
			t.tv_sec = 0;
			t.tv_nsec = 0;
			obc_timesync(&t, 6000);
			time_t tt = t.tv_sec + 946684800;

			printf("OBC time is: %s\r\n", ctime(&tt));
		}
		vTaskDelay(1 * delay_time_based);
	}
}

uint8_t hk_get() {
	uint8_t txbuf;
	timestamp_t t;
	t.tv_sec = 0;
	t.tv_nsec = 0;
	obc_timesync(&t, 6000);
	// t.tv_sec = csp_hton32(t.tv_sec);
	memcpy(&hk_buffer[0], &t.tv_sec, 4); // get Time LSB


	/* get adcs hk1			ID = 136, Current ADCS state */
	txbuf = 0x88;  
	if (i2c_master_transaction(0, adcs_node, &txbuf, 1, &hk_buffer[4], 48, adcs_delay) != E_NO_ERR)
		return Error;
	/* get adcs hk2:		ID = 137, Calibrated sensor measurements */
	txbuf = 0x89;
	if (i2c_master_transaction(0, adcs_node, &txbuf, 1, &hk_buffer[52], 36, adcs_delay) != E_NO_ERR)
		return Error;
	/* get adcs hk3:		ID = 138, Actuator commands */
	txbuf = 0x8A;
	if (i2c_master_transaction(0, adcs_node, &txbuf, 1, &hk_buffer[88], 12, adcs_delay) != E_NO_ERR)
		return Error;

	/* get System Status */
	HK_frame.reboot_count			= parameters.reboot_count;
	HK_frame.interface_3V3_current	= Interface_3V3_current_get();
	HK_frame.interface_5V_current	= Interface_5V_current_get();
	HK_frame.interface_temp			= Interface_tmp_get();
	HK_frame.interface_thermistor	= Interface_inms_thermistor_get();

	memcpy(&hk_buffer[100], &HK_frame.mode_status_flag, (int)sizeof(hk_frame_t));


	/* get Thermal Data from subsystems */
	// TS1_4();			/* No need to measure EPS temperature, it's in WOD */
	// TS5();			/* No need to measure COM temperature, it's in WOD */
	TS6();			/* Measure Antenna Temperature */
	TS7();			/* Measure ADCS Temperature */
	TS8();			/* Measure GPS Temperature */ /* No function now */
	TS9();			/* Measure OBC Temperature */
	// TS10();			/* No need to measure interface board temperature, it's in HK */
	// TS11();			/* No need to measure INMS temperature, it's in HK */
	memcpy(&hk_buffer[100 + sizeof(hk_frame_t)], &ThermalFrame.T6, 8);

	/**
	 * Total HK length :
	 * 4		48			36			12			14		8
	 * Time		ADCS_HK1	ADCS_HK2	ADCS_HK3	HK 		Thermal
	 * 
	 * => 122 Bytes
	 */
	
	// Finish collecting, dump the complete frame.
	hex_dump(&hk_buffer[0], hk_length);

	return No_Error;

}
void clean_hk_buffer() {
	for (int f = 0; f < 200; f++)
		hk_buffer[f] = 0;
}

void HK_Task(void * pvParameters) {

	while (1) {

		if (parameters.first_flight == 1)
			vTaskDelay(3 * delay_time_based);  //30000
		else
			vTaskDelay(6 * delay_time_based);  //60000

		clean_hk_buffer();
		clean_all();

		if (hk_get() != No_Error)
			printf("HK get fail\n");
		else {
			hk_write_dup(&hk_buffer[0]);
			printf("HK Get and Saved\n");
		}
	}
	/** End of HK  ,should never reach this */
	vTaskDelete(NULL);
}
