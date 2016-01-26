#include <util/timestamp.h>
#include <util/delay.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <nanomind.h>
#include <csp/csp_endian.h>
#include <dev/i2c.h>
#include <string.h>
#include <time.h>

#include "tele_function.h"
#include "subsystem.h"
#include "parameter.h"

void Anomaly_Handler_Task(void * pvParameters) {

	while (1) {
		
	}
	/** End of Task, Should Never Reach This */
	vTaskDelete(NULL);
}
