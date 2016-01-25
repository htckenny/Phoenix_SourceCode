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


void gps_timesyn() {
	uint8_t rxbuf[6];
	uint8_t txbuf = Raw_GPS_Time;
	timestamp_t t;
	time_t tt;

	if (i2c_master_transaction_2(0, adcs_node, &txbuf, 1, &rxbuf, 6, adcs_delay) == E_NO_ERR) {

		t.tv_sec = 0;
		t.tv_nsec = 0;
		memcpy(&t.tv_sec, &rxbuf[2], 4);
		t.tv_sec = csp_ntoh32(t.tv_sec);
		obc_timesync(&t, 1000);

		tt = t.tv_sec + 946684800;
		printf("OBC time Sync with GPS to : %s\r\n", ctime(&tt));
	}
	else
		printf("Sync time with GPS Fail");
}

void init_time() {
	time_t tt;
	timestamp_t t;

	t.tv_sec  = 473385600;			// 473385600 = to 2015/1/1
	t.tv_nsec = 0;

	obc_timesync(&t, 6000);

	/*-------------------------------------*/
	tt = t.tv_sec + 946684800;
	printf("Set OBC time to: %s\r\n", ctime(&tt));
}

void Clock_Task(void * pvParameters) {

	if (parameters.first_flight == 1)
		init_time();

	while (1) {
		vTaskDelay(600 * delay_time_based);
		gps_timesyn();
	}
	/** End of Task, Should Never Reach This */
	vTaskDelete(NULL);
}
