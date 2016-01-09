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

uint16_t battery_read() {
	uint8_t txbuf[1];
	uint8_t rxbuf[43+2];
	uint16_t Vbat = 0;
	txbuf[0] = 0x08;   // 0x08 EPS HK
	
	if (i2c_master_transaction_2(0, eps_node, &txbuf, 1, &rxbuf, 43+2, eps_delay) == E_NO_ERR){
		memcpy(&Vbat, &rxbuf[10],2);	
	}
	return csp_ntoh16(Vbat);
}

void Leave_safe_mode()
{
	printf("Recover from Safe Mode \n");
	printf("Recover from Safe Mode \n");
	printf("Recover from Safe Mode \n");

	// i2c_frame_t * frame;
	// frame = csp_buffer_get(I2C_MTU);
	// frame->dest = 2;
	// frame->data[0] = EPS_PORT_HARDRESET;
	// frame->len = 1;
	// frame->len_rx = 0;
	// frame->retries = 0;
	// if (i2c_send(0, frame, 0) != E_NO_ERR) {
	// 	csp_buffer_free(frame);
	// }
	HK_frame.mode_status_flag = init_mode;
	// last_mode = HK_frame.mode_status_flag;
}

void BatteryCheck_Task(void * pvParameters) {
	uint16_t vbat;
	vTaskDelay(3 * delay_time_based);
	printf("Battery Check Task activated \r\n");

	while (1) {
		vbat = battery_read();
		printf("vbat = %" PRIu16 " mV\n", vbat);

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
				vTaskDelay(30 * delay_time_based);

				vbat = battery_read();
				printf("(safe)vbat = %04u mV \r\n", csp_ntoh16(vbat));

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
		vTaskDelay(30 * delay_time_based);
	}
	/** End of Task, Should Never Reach This */
	vTaskDelete(NULL);
}