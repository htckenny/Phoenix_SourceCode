
#include <io/nanopower2.h>
#include <csp/csp.h>
#include <csp/csp_endian.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <dev/i2c.h>
#include <util/hexdump.h>
#include "parameter.h"
#include <util/delay.h>
#include "subsystem.h"
#include <dev/cpu.h>

#define E_NO_ERR -1
uint16_t battery_read(){
	   eps_hk_t * chkparam;

		i2c_frame_t * frame;
		frame = csp_buffer_get(I2C_MTU);
		frame->dest = 2;
		frame->data[0] = EPS_PORT_HK;
		frame->data[1] = 0;
		frame->len = 2;
		frame->len_rx = 2 + (uint8_t) sizeof(eps_hk_t);
		frame->retries = 0;

		if (i2c_send(0, frame, 0) != E_NO_ERR) {
			csp_buffer_free(frame);
			return 0;
		}

		if (i2c_receive(0, &frame, 20) != E_NO_ERR)
			return 0;

		chkparam = (eps_hk_t *)&frame->data[2];
		eps_hk_unpack(chkparam);
		csp_buffer_free(frame);
		return chkparam->vbatt;
}

void enter_safe_mode(uint8_t stats){
	printf("Enter Safe Mode \n");
	printf("Enter Safe Mode \n");
	printf("Enter Safe Mode \n");
	vTaskDelete(wod_task);
	vTaskDelete(seuv_task);
	vTaskDelete(adcs_task);




}

void Leave_safe_mode(){
	printf("Recover from Safe Mode \n");
	printf("Recover from Safe Mode \n");
	printf("Recover from Safe Mode \n");


	mode_status_flag=2;
//	if (cpu_set_reset_cause)
//		cpu_set_reset_cause(CPU_RESET_USER);
//	cpu_reset();

}


void BatteryCheckTask(void * pvParameters) {
	printf("Battery Check Task activated \r\n");
	uint16_t vbat;
	while(1){
         vbat = battery_read();
		printf("vbat = %05u mV \r\n",vbat);

		if( ((int)vbat < (int)parameters.vbat_safe_threshold) && (vbat!=0))
		mode_status_flag=0;



		if(mode_status_flag==0)
		while(1){
			vTaskDelay(30000);

			vbat = battery_read();
			printf("vbat = %05u mV \r\n",vbat);

			if( ((int)vbat > (int)parameters.vbat_recover_threshold) && (vbat!=0)){
				Leave_safe_mode();           /* Leave safemode  */
			   break;
		    }

		}

		vTaskDelay(30000);
	}
	/** End of Task, Should Never Reach This */
	vTaskDelete(NULL);
	}

