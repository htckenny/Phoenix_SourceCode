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
			thermal_2_w();
			printf("Get a Side panel thermal sensor packet # %d \n", num);
			hex_dump(&Tdata[0], 178);
		}
	}
}

/* interface board  INMS temperature */
int TS11() {
	ThermalFrame.T11 = Interface_inms_thermistor_get();
	return No_Error;
}

/* interface temperature */
int TS10() {
	ThermalFrame.T10 = Interface_tmp_get();
	return No_Error;
}

/* OBC temp */
int TS9() {
	uint16_t * adc_buffer;

	adc_buffer = adc_start_blocking(1);

	ThermalFrame.T9 = adc_buffer[0];

	// printf("%f\n", ((((adc_buffer[0] * 2493.0) / 1023) - 424) / 6.25));

	return No_Error;
}
/* ADCS Rate Sensor & Magnetometer temp */
int TS8() {
	uint8_t txbuffer = 175;		// check 135
	uint8_t rxbuffer[6];

	if (i2c_master_transaction_2(0, adcs_node, &txbuffer, 1, &rxbuffer, 6, adcs_delay) != E_NO_ERR)
		return Error;

	memcpy(&ThermalFrame.T8, &rxbuffer[4], 2);
	return No_Error;
}
/* ADCS ARM CPU temp */
int TS7()
{
	uint8_t txbuffer = 173;		// check 135
	uint8_t rxbuffer[6];

	if (i2c_master_transaction_2(0, adcs_node, &txbuffer, 1, &rxbuffer, 6, adcs_delay) != E_NO_ERR)
		return Error;

	memcpy(&ThermalFrame.T7, &rxbuffer[4], 2);
	return No_Error;
}

/* Antenna Board temp */
int TS6() {
	uint8_t txbuffer = 0xC0;
	uint8_t rxbuffer[2];

	if (i2c_master_transaction_2(0, ant_node, &txbuffer, 1, &rxbuffer, 2, com_delay) != E_NO_ERR)
		return Error;
	memcpy(&ThermalFrame.T6, &rxbuffer, 2);
	return No_Error;
}
/* COM board temp */
int TS5() {
	uint8_t txbuffer = com_rx_hk;
	uint16_t rxbuffer[7];

	if (i2c_master_transaction_2(0, com_rx_node, &txbuffer, 1, &rxbuffer, 14, com_delay) != E_NO_ERR)
		return Error;

	ThermalFrame.T5 = rxbuffer[5];
	return No_Error;
}

/* EPS temp */
int TS1_4() {
	uint8_t txbuf[2];
	uint8_t rxbuf[23];
	txbuf[0] = 0x08;
	txbuf[1] = 0x04;
	if (i2c_master_transaction_2(0, eps_node, &txbuf, 2, &rxbuf, 23, eps_delay) == E_NO_ERR) {
		ThermalFrame.T1 = (rxbuf[6] << 8) + rxbuf[7];
		ThermalFrame.T2 = (rxbuf[8] << 8) + rxbuf[9];
		ThermalFrame.T3 = (rxbuf[10] << 8) + rxbuf[11];
		ThermalFrame.T4 = (rxbuf[12] << 8) + rxbuf[13];
		return No_Error;
	}
	else
		return Error;
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
/**
 * This task is used only when conducting the thermal vacuum test
 * @param pvParameters [description]
 */
void Thermal_Task(void * pvParameters) {
	// int uart_nums = 0;
	// char uchar[174 * 10];
	num = 0;
	power_OFF_ALL();
	vTaskDelay(2 * delay_time_based);

	// power_control(4, ON);
	power_control(1, ON);

	while (1) {

		num++;
		clean_all();
		ThermalFrame.packet_number = num;
		TS1_4();
		TS5();
		TS6();
		TS7();
		TS8();
		TS9();
		TS10();
		TS11();
		printf("------------------------------------- \n");
		printf("NUM = %d\n", (int)ThermalFrame.packet_number);
		printf("EPS1 %03d\tESP2 %03d\tEPS3 %03d\tBatt %03d\n",  ThermalFrame.T1, ThermalFrame.T2, ThermalFrame.T3, ThermalFrame.T4);
		printf("COM %.2f\tAnt %.2f mV\tOBC %.2f\n", (ThermalFrame.T5 * (-0.0546) + 189.5522), (ThermalFrame.T6 * 3.3 * 1000 / 1023), ((((ThermalFrame.T9 * 2493.0) / 1023) - 424) / 6.25));
		printf("A_ARM %03d\tA_Rate %03d\tA_Mag %03d\n", ThermalFrame.T7, (ThermalFrame.T8 % 256), (ThermalFrame.T8 >> 8));
		printf("IFB %03d\tINMS %03d\n", (int)(159 - 0.08569 * ThermalFrame.T10), (int)((ThermalFrame.T11 / 3) - 273));

		thermal_1_w();
		// TS12_16();

		// uart_nums = usart_messages_waiting(2);
		// if (uart_nums != 0) {
		// 	for (int f = 0; f < uart_nums; f++)
		// 		uchar[f] = usart_getc(2);
		// 	hex_dump(&uchar, 174);

		// 	timestamp_t t;
		// 	t.tv_sec = 0;
		// 	t.tv_nsec = 0;
		// 	obc_timesync(&t, 6000);
		// 	time_t tt = t.tv_sec + 946684800;

		// 	printf("OBC time is: %s\r\n", ctime(&tt));
		// }
		vTaskDelay(1 * delay_time_based);
	}
}

uint8_t hk_get() {
	uint8_t txbuf;
	timestamp_t t;
	t.tv_sec = 0;
	t.tv_nsec = 0;
	obc_timesync(&t, 6000);
	t.tv_sec -= 946684800;
	memcpy(&hk_buffer[0], &t.tv_sec, 4); // get Time LSB


	/* get adcs hk1			ID = 136, Current ADCS state */
	txbuf = 0x88;
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 1, &hk_buffer[4], 48, adcs_delay) != E_NO_ERR)
		return Error;
	/* get adcs hk2:		ID = 137, Calibrated sensor measurements */
	txbuf = 0x89;
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 1, &hk_buffer[52], 36, adcs_delay) != E_NO_ERR)
		return Error;
	/* get adcs hk3:		ID = 138, Actuator commands */
	txbuf = 0x8A;
	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 1, &hk_buffer[88], 12, adcs_delay) != E_NO_ERR)
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
	TS7();			/* Measure ADCS CPU Temperature */
	TS8();			/* Measure ADCS Rate Sensor & Magnetometer Temperature */
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
	// hex_dump(&hk_buffer[0], hk_length);

	return No_Error;

}
void clean_hk_buffer() {
	for (int f = 0; f < 200; f++)
		hk_buffer[f] = 0;
}

void HK_Task(void * pvParameters) {

	while (1) {

		if (parameters.first_flight == 1)
			vTaskDelay(30 * delay_time_based);  //30000
		else
			vTaskDelay(60 * delay_time_based);  //60000

		clean_hk_buffer();
		clean_all();

		if (hk_get() != No_Error)
			printf("HK get fail\n");
		else {
			if (parameters.crippled_Mode == 0)
				hk_write_dup(&hk_buffer[0]);
			else
				hk_write_crippled(&hk_buffer[0]);

			printf("HK Get and Saved\n");
		}
	}
	/** End of HK  ,should never reach this */
	vTaskDelete(NULL);
}
