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

uint16_t battery_read() {
	uint8_t txbuf[2];
	uint8_t rxbuf[43];
	uint16_t Vbat = 0;
	txbuf[0] = 0x08;   //0d03 ADCS run mode
	txbuf[1] = 0x00;

	if (i2c_master_transaction(0, eps_node, &txbuf, 1, 0, 0, eps_delay) == E_NO_ERR){
		if (i2c_master_transaction(0, eps_node, 0, 0, &rxbuf, 43, eps_delay) == E_NO_ERR){
			memcpy(&Vbat, &rxbuf[8],2);	
		}
	}

	return Vbat;

// 	eps_hk_t * chkparam;

// 	i2c_frame_t * frame = csp_buffer_get(I2C_MTU);
// 	frame->dest = eps_node;				//2
// 	frame->data[0] = EPS_PORT_HK;		//8
// 	frame->data[1] = 0;
// 	frame->len = 2;
// 	frame->len_rx = 2 + (uint8_t) sizeof(eps_hk_t);
// 	frame->retries = 0;

// 	if (i2c_send(0, frame, 0) != E_NO_ERR) {
// 		csp_buffer_free(frame);
// 		return 0;
// 	}

// 	if (i2c_receive(0, &frame, 20) != E_NO_ERR)
// 		return 0;

// 	if (frame->data[0] != 8)
// 		i2c_receive(0, &frame, 20);

// 	if (frame->data[0] != 8)
// 		return 0;

// 	chkparam = (eps_hk_t *)&frame->data[2];
// 	eps_hk_unpack(chkparam);
// 	csp_buffer_free(frame);
// 	return chkparam->vbatt;
}

void Leave_safe_mode()
{
	printf("Recover from Safe Mode \n");
	printf("Recover from Safe Mode \n");
	printf("Recover from Safe Mode \n");

	i2c_frame_t * frame;
	frame = csp_buffer_get(I2C_MTU);
	frame->dest = 2;
	frame->data[0] = EPS_PORT_HARDRESET;
	frame->len = 1;
	frame->len_rx = 0;
	frame->retries = 0;
	if (i2c_send(0, frame, 0) != E_NO_ERR) {
		csp_buffer_free(frame);
	}
}

void BatteryCheckTask(void * pvParameters) {
	uint16_t vbat;
	vTaskDelay(3000);
	printf("Battery Check Task activated \r\n");

	while (1) {
		vbat = battery_read();
		printf("vbat = %04u mV \r\n", vbat);

		if ( (int) vbat < (int) parameters.vbat_safe_threshold) {
			vbat = battery_read();
			if ( (int) vbat < (int) parameters.vbat_safe_threshold) {
				if (vbat != 0) {
					if (parameters.vbat_safe_threshold != 0) {
						HK_frame.mode_status_flag = safe_mode;
					}
				}
			}
		}

		if (HK_frame.mode_status_flag == safe_mode) {
			while (1) {
				vTaskDelay(3000);

				vbat = battery_read();
				printf("vbat = %05u mV \r\n", vbat);

				if ( (int)vbat > (int)parameters.vbat_recover_threshold) {
					if (parameters.vbat_recover_threshold != 0) {
						if (vbat != 0) {
							Leave_safe_mode();           /* Leave safemode  */
							break;
						}
					}
				}
			}
		}
		vTaskDelay(3000);
	}
	/** End of Task, Should Never Reach This */
	vTaskDelete(NULL);
}