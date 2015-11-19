/**
 * NanoMind3
 *
 * @author Johan De Claville Christiansen
 * Copyright 2011 GomSpace ApS. All rights reserved.
 *
 * USING SRAM MEMORY:
 * To store a variable in internal SRAM use:
 * static __attribute__((section(".sram.data")))
 *
 * To place a function in internal SRAM use:
 * __attribute__((section(".sram.text")))
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#include <conf_nanomind.h>
#include <conf_io.h>
#include "mpio.h"
#include <dev/usart.h>
#include <dev/arm/cpu_pm.h>
#include <util/console.h>
#include <util/delay.h>

#include <supervisor/supervisor.h>

#include <csp/csp.h>
#include <csp/interfaces/csp_if_i2c.h>
#include <csp/interfaces/csp_if_kiss.h>
#if ENABLE_CAN
#include <csp/interfaces/csp_if_can.h>
#endif
#include <csp_extra/csp_console.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#if ENABLE_RTC
#include <util/clock.h>
#include <dev/arm/ds1302.h>
#endif

#define F_CPU				40000000
#define F_OSC				8000000
#define F_USART				500000		//wire:500000; BT:57600
#define F_USART_INMS		9600
#include "parameter.h"


int main(void) {

	/* Initialise USART */
	usart_init(USART_CONSOLE, cpu_core_clk, F_USART);
	usart_init(1, cpu_core_clk, F_USART);
	usart_init(2, cpu_core_clk, F_USART_INMS); //for inms
	/* Initialize delay */
	delay_init(cpu_core_clk);

#if ENABLE_RTC
	struct ds1302_clock clock;
	timestamp_t timestamp;

	/* Get time from RTC */
	ds1302_init();
	ds1302_clock_read_burst(&clock);
	ds1302_clock_to_time((time_t *) &timestamp.tv_sec, &clock);
	timestamp.tv_nsec = 0 ; 
	timestamp.tv_sec = 946684800 ;
	/* Set time in lib-c */
	clock_set_time(&timestamp);
#endif

	/* Initialize command */
	command_init();

	/** ROUTING TABLE */

#if ENABLE_CAN
	/* CAN nodes */
	struct csp_can_config conf = {.bitrate = 500000, .clock_speed = cpu_core_clk};
	csp_can_init(CSP_CAN_PROMISC, &conf);
	csp_route_set(9, &csp_if_can, CSP_NODE_MAC);
#endif

	/* Initialise CSP */
	csp_buffer_init(200, 500);
	csp_init(1);

	/* KISS USART1 (for csp-term) */
	void usart1_putc(char c) {
		usart_putc(USART_CONSOLE, c);
	}
	void usart1_insert(char c, void * pxTaskWoken) {
		usart_insert(USART_CONSOLE, c, pxTaskWoken);
	}
	static csp_iface_t csp_if_kiss1;
	static csp_kiss_handle_t csp_kiss1_driver;
	static const char * kiss1_name = "KISS1";
	csp_kiss_init(&csp_if_kiss1, &csp_kiss1_driver, usart1_putc, usart1_insert, kiss1_name);
	void my_usart1_rx(uint8_t * buf, int len, void * pxTaskWoken) {
		csp_kiss_rx(&csp_if_kiss1, buf, len, pxTaskWoken);
	}
	usart_set_callback(USART_CONSOLE, &my_usart1_rx);
	csp_route_set(8, &csp_if_kiss1, CSP_NODE_MAC);


	/* I2C nodes */
	csp_i2c_init(1, 0, 400);
	csp_route_set(2, &csp_if_i2c, CSP_NODE_MAC);
	csp_route_set(3, &csp_if_i2c, CSP_NODE_MAC);
	csp_route_set(4, &csp_if_i2c, CSP_NODE_MAC);
	csp_route_set(5, &csp_if_i2c, CSP_NODE_MAC);
	csp_route_set(6, &csp_if_i2c, CSP_NODE_MAC);
	csp_route_set(7, &csp_if_i2c, CSP_NODE_MAC);

	/* Default route */
	csp_route_set(CSP_DEFAULT_ROUTE, &csp_if_i2c, 5);

	/* Start router */
	csp_route_start_task(1024*4, 3);

	/* Initialise Console */
	console_init();
	console_set_hostname(CONFIG_HOSTNAME);

#ifdef ENABLE_CSP_CLIENT
	csp_console_init();
	csp_set_hostname(CONFIG_HOSTNAME);
	csp_set_model(CONFIG_MODEL);
#endif

#if ENABLE_SUPERVISOR
	/* Start supervisor */
	sv_init(1000);
#endif

#if ENABLE_CPP
	/* C++ static constructors */
	extern void (*__init_array_start []) (void) __attribute__((weak));
	extern void (*__init_array_end []) (void) __attribute__((weak));
	int count = __init_array_end - __init_array_start, i;
	for (i = 0; i < count; i++)
		__init_array_start[i]();
#endif

	/* GPIO initialization */
	for (int i = 0; i < 7; i++){	// Set all gpio as output
		io_init(i, 1);
	}
	io_set(2);						// set interface relay to ON as default

	/* Start tasks */
	xTaskCreate(debug_console, (const signed char *) "CONSOLE", 1024*4, NULL, 0, NULL);
	extern void vTaskInit(void *pvParameters);
	xTaskCreate(vTaskInit, (const signed char *) "INIT", 1024*4, NULL, 3, NULL);
	extern void vTaskServer(void * pvParameters);
	xTaskCreate(vTaskServer, (const signed char *) "SRV", 1024*4, NULL, 2, NULL);
	extern void BatteryCheckTask(void * pvParameters);
	xTaskCreate(BatteryCheckTask, (const signed char *) "BatCk", 1024 * 4, NULL, 2, NULL);
	
	// extern void EOP_Task(void * pvParameters);
	// xTaskCreate(EOP_Task, (const signed char * ) "EOP", 1024 * 4, NULL, 2, NULL);

	// extern void HK_Task(void * pvParameters) ;
	// xTaskCreate(HK_Task, (const signed char * ) "HK", 1024 * 4, NULL, 2, NULL);

	// extern void Telecom_Task(void * pvParameters);
	// xTaskCreate(Telecom_Task, (const signed char * ) "COM", 1024 * 4, NULL, 2, NULL);
	
	// extern void vTaskwod(void * pvParameters);
	// xTaskCreate(vTaskwod, (const signed char * ) "WOD", 1024 * 4, NULL, 2, &wod_task);
	
	// extern void vTaskinms(void * pvParameters);
	// xTaskCreate(vTaskinms, (const signed char *) "INMS", 1024*4, NULL, 2, NULL);
	
	// extern void SolarEUV_Task(void * pvParameters);
	// xTaskCreate(SolarEUV_Task, (const signed char *) "SEUV", 1024*4, NULL, 2, NULL);
	
	// extern void vTaskfstest(void * pvParameters);
	// xTaskCreate(vTaskfstest, (const signed char *) "FS_T", 1024*4, NULL, 2, NULL);
	
	// extern void Mode_Control(void * pvParameters);
	// xTaskCreate(Mode_Control, (const signed char *) "MC", 1024*4, NULL, 2, NULL);
	
	// extern void vTaskSchedule(void * pvParameters);
	// xTaskCreate(vTaskSchedule, (const signed char * ) "SCHE", 1024 * 4, NULL,2,NULL);
	
	// extern void thermal_test(void * pvParameters);
	// xTaskCreate(thermal_test, (const signed char *) "T_Test", 1024*4, NULL, 2, NULL);
	
	// extern void ADCS_Tasks(void * pvParameters);
	// xTaskCreate(ADCS_Tasks, (const signed char * ) "ADCS", 1024 * 4, NULL,2,NULL);
	
	// extern void vTaskInmsErrorHandle(void * pvParameters);
	// extern void vTaskInmsCurrentMonitor(void * pvParameters);
	// xTaskCreate(vTaskInmsErrorHandle, (const signed char * ) "INMS_EH", 1024 * 4, NULL, 2, &inms_error_handle);
 	// xTaskCreate(vTaskInmsCurrentMonitor, (const signed char * ) "INMS_CM", 1024 * 4, NULL, 2, &inms_current_moniter);

	/* Timer uses LFCLOCK = F_OSC/2 */
	vTaskStartScheduler(F_OSC/2, 1024*4);
	/* Should never reach here */
	while(1) exit(0);

}
