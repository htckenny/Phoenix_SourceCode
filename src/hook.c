#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "../lib/liba712/src/drivers/mpio.h"
#include "parameter.h"
#include "subsystem.h"
/**
 * hook_main:
 * This function is called by the board initialization routine in the file
 * main.c after setting up watchdog and csp, but before going into task context
 *
 * Important:
 * Do not use printf's here
 * Do not use FreeRTOS API calls here, since the scheduler is not running
 *
 */
void hook_main(void) {
	;
}

/**
 * hook_init_early:
 * This function is called by the task_init.c just at the beginning of the task
 * This happens before initialization of filesystems and the console task.
 * It is now safe to use FreeRTOS API calls, malloc and printf's.
 */
void hook_init_early(void) {
	;
}

/**
 * hook_init:
 * This function is called when the system is finally fully initialized.
 * Here you can expect the filesystem to be operative.
 *
 * Use this function to start new FreeRTOS tasks, that will run the mission
 * software.
 */
void hook_init(void) {
	printf("Welcome to nanomind...\r\n");

	/* GPIO initialization */
	for (int i = 0; i < 7; i++){	// Set all gpio as output
		io_init(i, 1);
	}
	io_set(2);						// set interface board relay to ON as default

	/* Start Self-defined Tasks */
	// parameter_init();

	// extern void ModeControl_Task(void * pvParameters);
	// xTaskCreate(ModeControl_Task, (const signed char *) "MC", 1024*4, NULL, 2, NULL);
	
	// extern void BatteryCheck_Task(void * pvParameters);
	// xTaskCreate(BatteryCheck_Task, (const signed char *) "BatCk", 1024 * 4, NULL, 2, NULL);
	
	// extern void EOP_Task(void * pvParameters);
	// xTaskCreate(EOP_Task, (const signed char * ) "EOP", 1024 * 4, NULL, 2, NULL);

	// extern void HK_Task(void * pvParameters) ;
	// xTaskCreate(HK_Task, (const signed char * ) "HK", 1024 * 4, NULL, 2, NULL);

	// extern void Telecom_Task(void * pvParameters);
	// xTaskCreate(Telecom_Task, (const signed char * ) "COM", 1024 * 4, NULL, 2, &com_task);
	
	// extern void WOD_Task(void * pvParameters);
	// xTaskCreate(WOD_Task, (const signed char * ) "WOD", 1024 * 4, NULL, 2, NULL);
	
	// extern void SolarEUV_Task(void * pvParameters);
	// xTaskCreate(SolarEUV_Task, (const signed char *) "SEUV", 1024*4, NULL, 2, NULL);
	
	// extern void vTaskfstest(void * pvParameters);
	// xTaskCreate(vTaskfstest, (const signed char *) "FS_T", 1024*4, NULL, 2, NULL);
	
	// extern void Schedule_Task(void * pvParameters);
	// xTaskCreate(Schedule_Task, (const signed char * ) "SCHE", 1024 * 4, NULL,2,NULL);
	
	// extern void thermal_test(void * pvParameters);
	// xTaskCreate(thermal_test, (const signed char *) "T_Test", 1024*4, NULL, 2, NULL);
	
	// extern void ADCS_Task(void * pvParameters);
	// xTaskCreate(ADCS_Task, (const signed char * ) "ADCS", 1024 * 4, NULL,2,NULL);
	
	// extern void vTaskinms(void * pvParameters);
	// xTaskCreate(vTaskinms, (const signed char *) "INMS", 1024*4, NULL, 2, &inms_task);
	// extern void vTaskInmsReceive(void * pvParameters);
	// xTaskCreate(vTaskInmsReceive, (const signed char*) "INMSR", 1024 * 4, NULL, 2, &inms_task_receive);
	// extern void vTaskInmsErrorHandle(void * pvParameters);
	// xTaskCreate(vTaskInmsErrorHandle, (const signed char * ) "INMS_EH", 1024 * 4, NULL, 2, &inms_error_handle);
	// extern void vTaskInmsCurrentMonitor(void * pvParameters);
 	// xTaskCreate(vTaskInmsCurrentMonitor, (const signed char * ) "INMS_CM", 1024 * 4, NULL, 2, &inms_current_moniter);
	
}
