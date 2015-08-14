#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "parameter.h"
#include "subsystem.h"
extern void Init_Task(void * pvParameters);
extern void ADCS_Task(void * pvParameters);
extern void HK_Task(void * pvParameters);
extern void SolarEUV_Task(void * pvParameters);
extern void vTaskInmsErrorHandle(void * pvParameters);
extern void vTaskInmsCurrentMonitor(void * pvParameters);


void Enter_Safe_Mode(int last_mode) {

    if (last_mode == 1) {
        vTaskDelete(init_task);

    }
    if (last_mode == 2)
        vTaskDelete(adcs_task);
//  if(parameters.first_flight==1)
//     vTaskDelete(eop_task);

    power_control(1, OFF);
    power_control(2, OFF);

    if (last_mode == 3) {
        vTaskDelete(adcs_task);
        vTaskDelete(hk_task);
        vTaskDelete(seuv_task);
        vTaskDelete(inms_error_handle);
        vTaskDelete(inms_current_moniter);

        if (inms_task_flag == 1)
            vTaskDelete(inms_task);

        if (inms_task_receive_flag == 1)
            vTaskDelete(inms_task_receive);

        inms_task_flag = 0;
        inms_task_receive_flag = 0;


        power_OFF_ALL();
    }
}


void Mode_Control(void * pvParameters) {
    /* Mode 1 = INIT Mode
     * Mode 2 = ADCS MODE
     * MODE 3 = PAYLOAD MODE
     * MODE 0 = SAFE MODE
     */

    power_OFF_ALL();   // command EPS power off all configurable device for insurance.
    vTaskDelay(2000);  // waiting for power off being applied

    HK_frame.mode_status_flag = 1; // set satellite to enter INIT mode
    printf("-----------------------Enter INIT Mode----------------------------\n");
    xTaskCreate(Init_Task, (const signed char *) "Init", 1024 * 4, NULL, 2, &init_task);

    uint8_t lastmode = 1; // Satellite  enter init mode done.

    while (1) {
        if (HK_frame.mode_status_flag != lastmode) { // Mode change detected!!!

            if (HK_frame.mode_status_flag == 2) { // if wants to Enter ADCS mode
                printf("---------------------Enter ADCS Mode----------------------\n");
              // if(parameters.first_flight==1)
              // xTaskCreate(EOP_Task, (const signed char * ) "EOP", 1024 * 4, NULL,1, &eop_task);
                // xTaskCreate(ADCS_Task, (const signed char * ) "ADCS", 1024 * 4, NULL, 1, &adcs_task);


                lastmode = HK_frame.mode_status_flag; // ENTER ADCS MODE DONE!

            }
            else if (HK_frame.mode_status_flag == 3) { //if wants to enter Payload Mode.

                printf("-------------------Enter Payload Mode----------------------\n");
                // xTaskCreate(HK_Task, (const signed char * ) "HK", 1024 * 4, NULL, 2, &hk_task);
                // xTaskCreate(SolarEUV_Task, (const signed char * ) "SEUV", 1024 * 4, NULL, 3, &seuv_task);
                // xTaskCreate(vTaskInmsErrorHandle, (const signed char * ) "InmsEH", 1024 * 4, NULL, 2, &inms_error_handle);
                // xTaskC/reate(vTaskInmsCurrentMonitor, (const signed char * ) "inms_CM", 1024 * 4, NULL, 2, &inms_current_moniter);
                lastmode = HK_frame.mode_status_flag;
            }
            else if (HK_frame.mode_status_flag == 0) { //if wants to enter  SafeMode.
                printf("-------------------Enter Safe Mode----------------------\n");
                
                Enter_Safe_Mode(lastmode);

                lastmode = HK_frame.mode_status_flag; // ENTER SAFE Mode DONE!
            }
        }
        vTaskDelay(1000); // check whether mode changes every second
    }
    /** End of this Task  ,Should Never Reach This Line*/
    vTaskDelete(NULL);
}
