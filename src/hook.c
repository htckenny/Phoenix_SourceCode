#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "../lib/liba712/src/drivers/mpio.h"
// #include "drivers/mpio.h"
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

	// extern void BatteryCheckTask(void * pvParameters);
	// xTaskCreate(BatteryCheckTask, (const signed char *) "BatCk", 1024 * 4, NULL, 2, NULL);
	
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
	
	// Implement your code here

}
