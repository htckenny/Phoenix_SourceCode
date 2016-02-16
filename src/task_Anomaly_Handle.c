#include <util/timestamp.h>
#include <util/delay.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <nanomind.h>
#include <csp/csp_endian.h>
#include <dev/i2c.h>
#include <string.h>
#include <time.h>
#include <fat_sd/ff.h>

#include "fs.h"
#include "tele_function.h"
#include "subsystem.h"
#include "parameter.h"
FATFS fs[2];
void Anomaly_Handler_Task(void * pvParameters)
{

	while (1) {
		if (SD_lock_flag == 10) {
			for (int i = 0 ; i < 2 ; i++){
				if (f_mount(i, NULL) == FR_OK)
					printf("unmount %d\n", i);

				if (f_mount(i, &fs[i]) == FR_OK)
					printf("mount %d\n", i);
			}
		}
		vTaskDelay(10 * delay_time_based);
		printf("%d\n", SD_lock_flag);
	}
	/** End of Task, Should Never Reach This */
	vTaskDelete(NULL);
}
