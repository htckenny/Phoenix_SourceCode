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
	uint8_t rxbuf[43 + 2];
	uint16_t Vbat = 0;
	txbuf[0] = 0x08;

	if (i2c_master_transaction_2(0, stm_eps_node_v, &txbuf, 1, &rxbuf, 43 + 2, eps_delay) == E_NO_ERR) {
		memcpy(&Vbat, &rxbuf[10], 2);
		i2c_lock_flag = 0;
	}
	else {
		i2c_lock_flag ++;
		if (i2c_lock_flag >= 3) {
			power_control(5, OFF);
			i2c_lock_flag = 0;
		}
	}
	return csp_ntoh16(Vbat);
}
void Enter_Recovery_Mode() {
	if (bat_check_task != NULL) {
		printf("Shutting Down battery Task\n");
		vTaskDelete(bat_check_task);
		bat_check_task = NULL;
	}
	if (Anom_mon_task != NULL) {
		printf("Shutting Down Anomaly Task\n");
		vTaskDelete(Anom_mon_task);
		Anom_mon_task = NULL;
	}
	if (schedule_task != NULL) {
		printf("Shutting Down schedule Task\n");
		vTaskDelete(schedule_task);
		schedule_task = NULL;
	}
	if (eop_task != NULL) {
		printf("Shutting Down EOP Task\n");
		vTaskDelete(eop_task);
		eop_task = NULL;
	}
	if (hk_task != NULL) {
		printf("Shutting Down HK_Task\n");
		vTaskDelete(hk_task);
		hk_task = NULL;
	}
	if (seuv_task != NULL) {
		printf("Shutting Down SEUV_Task\n");
		vTaskDelete(seuv_task);
		seuv_task = NULL;
	}
	if (seuv_cm_task != NULL) {
		printf("Shutting Down SEUV Current Monitor Task\n");
		vTaskDelete(seuv_cm_task);
		seuv_cm_task = NULL;
	}
	if (inms_error_handle != NULL) {
		printf("Shutting Down INMS Error task \n");
		vTaskDelete(inms_error_handle);
		inms_error_handle = NULL;
	}
	if (inms_current_moniter != NULL) {
		printf("Shutting Down INMS current task \n");
		vTaskDelete(inms_current_moniter);
		inms_current_moniter = NULL;
	}
	if (inms_task != NULL) {
		printf("Shutting Down INMS task \n");
		vTaskDelete(inms_task);
		inms_task = NULL;
	}
	if (inms_task_receive != NULL) {
		printf("Shutting Down INMS receive task \n");
		vTaskDelete(inms_task_receive);
		inms_task_receive = NULL;
	}
	if (inms_temp_moniter != NULL) {
		printf("Shutting Down INMS temperature monitor task \n");
		vTaskDelete(inms_temp_moniter);
		inms_temp_moniter = NULL;
	}

	power_control(2, OFF);      // Power OFF GPS
	power_control(3, OFF);      // Power OFF SEUV
	power_control(4, OFF);      // Power OFF INMS
}
void Enter_Safe_Mode(int last_mode) {

	/* last mode = Init Mode */
	if (last_mode == 1) {
		if (init_task != NULL) {
			vTaskDelete(init_task);
			init_task = NULL;
		}
	}
	/* last mode = ADCS Mode */
	if (last_mode == 2 || last_mode == 4) {
		if (parameters.first_flight == 1) {
			if (eop_task != NULL) {
				vTaskDelete(eop_task);
				eop_task = NULL;
			}
		}
		if (adcs_task != NULL) {
			vTaskDelete(adcs_task);
			adcs_task = NULL;
		}
	}
	power_control(1, OFF);      // Power OFF ADCS
	power_control(2, OFF);      // Power OFF GPS

	/* last mode = Payload Mode */
	if (last_mode == 3) {

		if (adcs_task != NULL) {
			printf("Shutting Down ADCS_Task\n");
			vTaskDelete(adcs_task);
			adcs_task = NULL;
		}
		if (hk_task != NULL) {
			printf("Shutting Down HK_Task\n");
			vTaskDelete(hk_task);
			hk_task = NULL;
		}
		if (seuv_task != NULL) {
			printf("Shutting Down SEUV_Task\n");
			vTaskDelete(seuv_task);
			seuv_task = NULL;
		}
		if (seuv_cm_task != NULL) {
			printf("Shutting Down SEUV current monitor Task\n");
			vTaskDelete(seuv_cm_task);
			seuv_cm_task = NULL;
		}
		if (inms_error_handle != NULL) {
			printf("Shutting Down INMS Error task \n");
			vTaskDelete(inms_error_handle);
			inms_error_handle = NULL;
		}
		if (inms_current_moniter != NULL) {
			printf("Shutting Down INMS current task \n");
			vTaskDelete(inms_current_moniter);
			inms_current_moniter = NULL;
		}
		if (inms_task != NULL) {
			printf("Shutting Down INMS task \n");
			vTaskDelete(inms_task);
			inms_task = NULL;
		}
		if (inms_task_receive != NULL) {
			printf("Shutting Down INMS receive task \n");
			vTaskDelete(inms_task_receive);
			inms_task_receive = NULL;
		}
		if (inms_temp_moniter != NULL) {
			printf("Shutting Down INMS temperature monitor task \n");
			vTaskDelete(inms_temp_moniter);
			inms_temp_moniter = NULL;
		}
		power_OFF_ALL();
	}
	if (schedule_task != NULL) {
		printf("Shutting Down schedule task \n");
		vTaskDelete(schedule_task);
		schedule_task = NULL;
	}
	if (gps_task != NULL) {
		printf("Shutting Down GPS task \n");
		vTaskDelete(gps_task);
		gps_task = NULL;
	}
	if (Anom_mon_task != NULL) {
		printf("Shutting Down Anomaly task \n");
		vTaskDelete(Anom_mon_task);
		Anom_mon_task = NULL;
	}


}
void Leave_safe_mode()
{
	printf("Recover from Safe Mode \n");
	HK_frame.mode_status_flag = init_mode;
}

void BatteryCheck_Task(void * pvParameters) {
	uint16_t vbat;
	uint8_t safe_counter = 0;
	uint8_t normal_counter = 0;

	vTaskDelay(3 * delay_time_based);
	printf("Battery Check Task activated \r\n");

	while (1) {
		vbat = battery_read();
		printf("vbat = %" PRIu16 " mV\t", vbat);

		if (HK_frame.mode_status_flag == safe_mode) {
			while (1) {
				vTaskDelay(30 * delay_time_based);
				vbat = battery_read();
				printf("(safe)vbat = %04u mV \t", vbat);
				if ( (int)vbat > (int)parameters.vbat_recover_threshold) {
					safe_counter ++;
					printf("Batt outcounter = %d\n", safe_counter);
					if (safe_counter >= 6) {
						safe_counter = 0;
						Leave_safe_mode();
						break;
					}
				}
				else {
					printf("\n");
					safe_counter = 0;
				}
			}
		}
		else {
			if ( (int) vbat < (int) parameters.vbat_safe_threshold && vbat != 0) {
				normal_counter ++;
				printf("Batt outcounter = %d\n", normal_counter);
				if (normal_counter >= 6) {
					printf("safe mode detected\n");
					generate_Error_Report(1, vbat);
					HK_frame.mode_status_flag = safe_mode;
					normal_counter = 0;
				}
			}
			else {
				printf("\n");
				normal_counter = 0;
			}
		}
		vTaskDelay(30 * delay_time_based);
	}
	/** End of Task, Should Never Reach This */
	vTaskDelete(NULL);
}