#include <util/delay.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "parameter.h"
#include "subsystem.h"
#include <dev/i2c.h>
#include "fs.h"

void ADCS_Task(void * pvParameters) {
    while (1) {
        printf(" ADCS Operation TBD, keep going to payload mode in 10s \n");
        /*
         * ADCS & GPS must be power on before first time sync
         * vTaskDelay(10000);   ADCS needs 10s to initialize
         */
        power_control(1, ON);
        power_control(2, ON);
        vTaskDelay(10000);
        //  extern void Clock_Task(void * pvParameters);
        //  xTaskCreate(Clock_Task, (const signed char * ) "ClockTask", 1024 * 4, NULL,3, NULL);
        printf("Magnetoeter deployed (NOT YET!!)\n");
        /***********************/
        printf(" ADCS Operation Done \n");
        // HK_frame.adcs_done_flag=1;   // ADCS Done
        // HK_frame.mode_status_flag=3; // enter Payload

        while (1) {
            vTaskDelay(1000);
            // if (parameters.adcs_function == 0){
                // vTaskDelete(NULL);
            // }
        }
        vTaskDelete(NULL);
    }
}
