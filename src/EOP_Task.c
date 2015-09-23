/*
 * EOP_Task.c
 *
 *  Created on: 2015/8/21
 *      Author: Kenny Huang
 */

#include "fs.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <dev/cpu.h>
#include <dev/i2c.h>
#include <util/hexdump.h>
#include "parameter.h"
#include <csp/csp.h>
#include "subsystem.h"
#include <util/timestamp.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <math.h>
#include "Tele_function.h"

void EOP_Task(void * pvParameters) {

	while(1) {
		vTaskDelay(10000);
		printf("collecting EOP packet\n");
	}

	/** End of init */
	vTaskDelete(NULL);
}

/*--------------------------------------------------------------------*/



