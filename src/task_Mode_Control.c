#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "parameter.h"
#include "subsystem.h"
#include "fs.h"

extern void EOP_Task(void * pvParameters);
extern void Init_Task(void * pvParameters);
extern void ADCS_Task(void * pvParameters);
extern void HK_Task(void * pvParameters);
extern void SolarEUV_Task(void * pvParameters);
extern void vTaskInmsErrorHandle(void * pvParameters);
extern void vTaskInmsCurrentMonitor(void * pvParameters);
extern void vTaskInmsTemperatureMonitor(void * pvParameters);

extern void Enter_Safe_Mode(int last_mode) ;

void ModeControl_Task(void * pvParameters) {
	/* Mode 1 = INIT Mode
	 * Mode 2 = ADCS MODE
	 * MODE 3 = PAYLOAD MODE
	 * MODE 0 = SAFE MODE
	 */

	uint8_t lastmode ;
	/* power off all configurable device for insurance. */
	power_OFF_ALL();
	vTaskDelay(2 * delay_time_based);  // waiting for power off being applied

	/* Set satellite to enter INIT mode */
	HK_frame.mode_status_flag = 1;
	printf("-----------------------Enter INIT Mode----------------------------\n");
	xTaskCreate(Init_Task, (const signed char *) "Init", 1024 * 4, NULL, 2, &init_task);

	/* Satellite  enter init mode done. */
	lastmode = 1;

	while (1) {
		/* Mode change detected!!! */
		if (HK_frame.mode_status_flag != lastmode) {
			/* desire to Enter the Initial mode */
			if (HK_frame.mode_status_flag == 1 /*&& parameters.first_flight != 1*/) {
				printf("---------------------Enter Init Mode----------------------\n");

				if (init_task == NULL)
					xTaskCreate(Init_Task, (const signed char *) "Init", 1024 * 4, NULL, 2, &init_task);

				lastmode = HK_frame.mode_status_flag; /* ENTER INIT MODE */
			}
			/* desire to Enter the ADCS mode */
			else if (HK_frame.mode_status_flag == 2) {
				printf("---------------------Enter ADCS Mode----------------------\n");
				power_control(1, ON);
				if (parameters.first_flight == 1) {
					if (eop_task == NULL)
						xTaskCreate(EOP_Task, (const signed char * ) "EOP", 1024 * 4, NULL, 2, &eop_task);

					/* TODO: implement GPS task and activate this line */
					// if (gps_task == NULL)
					//     xTaskCreate(GPS_Task, (const signed char * ) "GPS", 1024 * 4, NULL, 1, &gps_task);
				}
				if (adcs_task == NULL)
					xTaskCreate(ADCS_Task, (const signed char * ) "ADCS", 1024 * 4, NULL, 2, &adcs_task);

				lastmode = HK_frame.mode_status_flag; /* ENTER ADCS MODE */
			}
			/* desire to Enter the Payload Mode. */
			else if (HK_frame.mode_status_flag == 3) {
				vTaskDelay(3 * delay_time_based);

				printf("-------------------Enter Payload Mode----------------------\n");
				if (parameters.first_flight == 1) {
					if (eop_task != NULL) {
						printf("delete eop task\n");
						vTaskDelete(eop_task);
						eop_task = NULL;
					}
					parameters.first_flight = 0;
					para_w_flash();
				}
				printf("Creating tasks of HK, INMS, SEUV ~~~\n");
				if (hk_task == NULL)
					xTaskCreate(HK_Task, (const signed char * ) "HK", 1024 * 4, NULL, 1, &hk_task);
				if (seuv_task == NULL)
					xTaskCreate(SolarEUV_Task, (const signed char * ) "SEUV", 1024 * 4, NULL, 2, &seuv_task);
				if (inms_error_handle == NULL)
					xTaskCreate(vTaskInmsErrorHandle, (const signed char * ) "InmsEH", 1024 * 4, NULL, 2, &inms_error_handle);
				if (inms_current_moniter == NULL)
					xTaskCreate(vTaskInmsCurrentMonitor, (const signed char * ) "InmsCM", 1024 * 4, NULL, 1, &inms_current_moniter);
				if (inms_temp_moniter == NULL)
					xTaskCreate(vTaskInmsTemperatureMonitor, (const signed char * ) "InmsTM", 1024 * 4, NULL, 1, &inms_temp_moniter);
				lastmode = HK_frame.mode_status_flag;   /* ENTER PAYLOAD MODE */
			}
			/* desire to Enter the Safe Mode. */
			else if (HK_frame.mode_status_flag == 0) {
				printf("-------------------Enter Safe Mode----------------------\n");
				Enter_Safe_Mode(lastmode);
				lastmode = HK_frame.mode_status_flag;	/* ENTER SAFE Mode */
			}
		}
		/* Check if the mode is changed or not every second */
		vTaskDelay(1 * delay_time_based);
	}
	/* End of this Task, Should Never Reach This Line */
	vTaskDelete(NULL);
}
