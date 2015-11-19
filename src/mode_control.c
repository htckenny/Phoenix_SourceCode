#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "parameter.h"
#include "subsystem.h"
#include "fs.h"
extern void EOP_Task(void * pvParameters);
extern void Init_Task(void * pvParameters);
extern void ADCS_Tasks(void * pvParameters);
extern void HK_Task(void * pvParameters);
extern void SolarEUV_Task(void * pvParameters);
extern void vTaskInmsErrorHandle(void * pvParameters);
extern void vTaskInmsCurrentMonitor(void * pvParameters);


void Enter_Safe_Mode(int last_mode) {

    /* last mode = Init Mode */
    if (last_mode == 1) {                                       
        vTaskDelete(init_task);
    }
    /* last mode = ADCS Mode */
    if (last_mode == 2){
        vTaskDelete(adcs_task);
        if(parameters.first_flight==1){
            vTaskDelete(eop_task);                
        }
    }                                

    power_control(1, OFF);      // Power OFF    ADCS
    power_control(2, OFF);      // Power OF     GPS

    /* last mode = Payload Mode */
    if (last_mode == 3) {
        printf("Shutting Down ADCS_Task\n");
        vTaskDelete(adcs_task);
        printf("Shutting Down HK_Task\n");
        vTaskDelete(hk_task);
        printf("Shutting Down SEUV_Task\n");
        vTaskDelete(seuv_task);
        printf("Shutting Down INMS related \n");
        vTaskDelete(inms_error_handle);
        vTaskDelete(inms_current_moniter);

        if (inms_task_flag == 1){
            vTaskDelete(inms_task);
            inms_task_flag = 0;
        }

        if (inms_task_receive_flag == 1){
            vTaskDelete(inms_task_receive);
            inms_task_receive_flag = 0;
        }
        power_OFF_ALL();
    }
}

void Mode_Control(void * pvParameters) {
    /* Mode 1 = INIT Mode
     * Mode 2 = ADCS MODE
     * MODE 3 = PAYLOAD MODE
     * MODE 0 = SAFE MODE
     */

    uint8_t lastmode ;
    /* power off all configurable device for insurance. */
    power_OFF_ALL();   
    vTaskDelay(2000);  // waiting for power off being applied

    /* Set satellite to enter INIT mode */
    HK_frame.mode_status_flag = 1; 
    printf("-----------------------Enter INIT Mode----------------------------\n");
    xTaskCreate(Init_Task, (const signed char *) "Init", 1024 * 4, NULL, 2, &init_task);

    /* Satellite  enter init mode done. */
    lastmode = 1;  

    while (1) {
        /* Mode change detected!!! */
        if (HK_frame.mode_status_flag != lastmode) { 
            /* desire to Enter the ADCS mode */
            if (HK_frame.mode_status_flag == 2) { 
                printf("---------------------Enter ADCS Mode----------------------\n");
                if(parameters.first_flight==1){
                    xTaskCreate(EOP_Task, (const signed char * ) "EOP", 1024 * 4, NULL,1, &eop_task);
                    // xTaskCreate(GPS_Task, (const signed char * ) "GPS", 1024 * 4, NULL, 1, &gps_task);
                    // TODO: implement GPS task and activate this line
                }
                xTaskCreate(ADCS_Tasks, (const signed char * ) "ADCS", 1024 * 4, NULL, 1, &adcs_task);

                lastmode = HK_frame.mode_status_flag; // ENTER ADCS MODE DONE!
            }
            /* desire to Enter the Payload Mode. */
            else if (HK_frame.mode_status_flag == 3) {
                vTaskDelay(3000);

                printf("-------------------Enter Payload Mode----------------------\n");
                if (parameters.first_flight == 1){
                    printf("delete eop\n");
                    // vTaskDelete(eop_task);
                    parameters.first_flight = 0;
                    para_w();
                }
                printf("Creating tasks of HK, INMS, SEUV ~~~\n");
                xTaskCreate(HK_Task, (const signed char * ) "HK", 1024 * 4, NULL, 2, &hk_task);
                xTaskCreate(SolarEUV_Task, (const signed char * ) "SEUV", 1024 * 4, NULL, 3, &seuv_task);
                // xTaskCreate(vTaskInmsErrorHandle, (const signed char * ) "InmsEH", 1024 * 4, NULL, 2, &inms_error_handle);
                // xTaskCreate(vTaskInmsCurrentMonitor, (const signed char * ) "inms_CM", 1024 * 4, NULL, 2, &inms_current_moniter);
                lastmode = HK_frame.mode_status_flag;   // ENTER PAYLOAD MODE DONE!
            }
            /* desire to Enter the Safe Mode. */
            else if (HK_frame.mode_status_flag == 0) { 
                printf("-------------------Enter Safe Mode----------------------\n");
                Enter_Safe_Mode(lastmode);


                lastmode = HK_frame.mode_status_flag; // ENTER SAFE Mode DONE!
            }
        }
        /* Check if the mode is changed or not every second */
        vTaskDelay(1000); 
    }
    /* End of this Task, Should Never Reach This Line */
    vTaskDelete(NULL);
}
