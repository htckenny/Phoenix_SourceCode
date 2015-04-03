#include <util/delay.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "parameter.h"
#include "subsystem.h"
#include <dev/i2c.h>
#include "fs.h"

void ADCS_Task(void * pvParameters){



while(1){

    printf(" ADCS Operation TBD, keep going to payload mode \n");


    vTaskDelay(10000);
    printf(" ADCS Operation Done, Enter Payload Mode \n");




    adcs_done_flag=1;
    mode_status_flag=3;
    parameters.first_flight=0;
    para_w();


    while(1){
    vTaskDelay(5000);
    }

    vTaskDelete(NULL);

}



}
