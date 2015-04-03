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


#define E_NO_ERR -1



int parameter_init() {

	mode_status_flag = 1;
	beacon_period=30;
	sun_light_flag=0;
	adcs_done_flag=0;

	parameters.wod_store_count = 0;
	parameters.inms_store_count = 0;
	parameters.seuv_store_count = 0;
	parameters.hk_store_count = 0;

	parameters.obc_packet_sequence_count = 0;
	parameters.inms_packet_sequence_count = 0;
	parameters.seuv_packet_sequence_count = 0;
	parameters.wod_packet_sequence_count = 0;
	parameters.phoenix_hk_packet_sequence_count = 0;
	parameters.ax25_sequence_count = 0;
	parameters.tc_count=0;

	parameters.first_flight = 1;
	parameters.shutdown_flag = 0;
	parameters.adcs_function = 1;
	parameters.inms_function = 0;
	parameters.com_function = 1;
	parameters.seuv_function = 1;
	parameters.gps_function = 1;

	parameters.vbat_recover_threshold = 8000;
	parameters.vbat_safe_threshold = 6800;

	seuvFrame.samples=50;

	if (para_r() != 0)
		return 1;
	else
		para_w();
		return 0;  //success

}


void Init_Task(void * pvParameters) {

	if (parameter_init() != 0)
		printf("Can't read parameter from fs\n");
	else
		printf("Loaded parameter from fs\n");

    set_Call_Sign(0);
	/*In order to Sync time before WOD Task start to work,
	 * ADCS & GPS must be power on before first time sync
	 * vTaskDelay(10000);  since ADCS needs 10s to initialize
	 */
     power_control(1,ON);
     power_control(2,ON);
	 vTaskDelay(10000);

	extern void BatteryCheckTask(void * pvParameters);
	xTaskCreate(BatteryCheckTask, (const signed char *) "Bat_Check", 1024*4, NULL, 2, NULL);
	extern void Clock_Task(void * pvParameters);
	xTaskCreate(Clock_Task, (const signed char * ) "ClockTask", 1024 * 4, NULL,2, NULL);





	/*   Idle 30M in first flight  */
	if (parameters.first_flight != 0)
		printf("Idle 30M   use GOSH 'idleunlock' to cancel \r\n");
	while (parameters.first_flight != 0) {
		static int count = 0;
		vTaskDelay(3000);
		count = count + 3;
		printf("Can use GOSH 'idleunlock' to cancel idle \r\n");
		printf("Left %d second to deploy devices \r\n", (1800 - count));
		if (count == 1800) {
			parameters.first_flight = 1;
			para_w();           //update to parameter system
		}
	}



	/*   Deploy Device  */
	// deploy_antenna();
    printf("Antenna was set not to deploy\n");


    printf("Active Telecom Task\n");
	extern void Telecom_Task(void * pvParameters);
	xTaskCreate(Telecom_Task, (const signed char * ) "COM_Task", 1024 * 4, NULL,2,NULL);

	printf("Active WOD Task\n");
	extern void vTaskwod(void * pvParameters);
	xTaskCreate(vTaskwod, (const signed char * ) "WOD_Task", 1024 * 4, NULL,2, &wod_task);

	mode_status_flag = 2;


	/** End of init */
	vTaskDelete(NULL);
}

/*--------------------------------------------------------------------*/


