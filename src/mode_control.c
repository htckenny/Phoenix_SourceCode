#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "parameter.h"
#include "subsystem.h"
  extern void Init_Task(void * pvParameters);
  extern void ADCS_Task(void * pvParameters);
  extern void HK_Task(void * pvParameters);
  extern void SolarEUV_Task(void * pvParameters);
  extern void vTaskInmsReceive(void * pvParameters);
  extern void vTaskinms(void * pvParameters);

void Enter_Safe_Mode(int last_mode){

	if(last_mode==1){
	vTaskDelete(init_task);
    power_control(1,OFF);
    power_control(2,OFF);
	}
if(last_mode==2)
	vTaskDelete(adcs_task);
    vTaskDelete(hk_task);
    power_control(1,OFF);
    power_control(2,OFF);

if(last_mode==3){
	vTaskDelete(adcs_task);
    vTaskDelete(hk_task);
	if(parameters.seuv_function==1)
    vTaskDelete(seuv_task);


	if(parameters.inms_function==1){
	vTaskDelete(inms_task);
	vTaskDelete(inms_task_receive);
	}

	power_OFF_ALL();
}
}


void mode_control(void * pvParameters) {
  uint8_t lastmode=1;
  power_OFF_ALL();
  vTaskDelay(2000);
  mode_status_flag=lastmode;
  printf("-----------------------Enter INIT Mode----------------------------\n");
  xTaskCreate(Init_Task, (const signed char *) "Init_Task", 1024*4, NULL, 2,&init_task);

	while(1){

		if(mode_status_flag!=lastmode){  // Mode change detected!!!

           if(mode_status_flag==2){   // if wants to Enter ADCS mode,should only from init mode.
        	   lastmode=mode_status_flag;
        	   printf("---------------------Enter ADCS Mode----------------------\n");
        		printf(" Powering On ADCS & GPS, wait for 10s \n");
        	    power_control(1,ON);
        	    power_control(2,ON);
        		vTaskDelay(10000);
        	   xTaskCreate(ADCS_Task, (const signed char * ) "ADCS_Task", 1024 * 4, NULL,2, &adcs_task);
        	   xTaskCreate(HK_Task, (const signed char * ) "HK_Task", 1024 * 4, NULL,2, &hk_task);
           }else if(mode_status_flag==3){ //if wants to enter Payload Mode.
        	   lastmode=mode_status_flag;
        	   printf("-------------------Enter Payload Mode----------------------\n");

        	   if(parameters.seuv_function==1)
        	   xTaskCreate(SolarEUV_Task, (const signed char * ) "SEUV_Task", 1024 * 4, NULL,3, &seuv_task);

    //    	   if(parameters.inms_function==1){
    //           xTaskCreate(vTaskInmsReceive, (const signed char * ) "InmsReceive_Task", 1024 * 4, NULL,2, &inms_task_receive);
    //           xTaskCreate(vTaskinms, (const signed char * ) "inms_Task", 1024 * 4, NULL,2, &inms_task);
    //           }
           }
           else if(mode_status_flag==0){ //if wants to enter  SafeMode.
        	   printf("Enter Safe Mode\n");
        	   Enter_Safe_Mode(lastmode);
        	   lastmode=mode_status_flag;
           }

		}
		vTaskDelay(1000);
	}
	/** End this Task  ,Should Never Reach This Line*/
	vTaskDelete(NULL);
}
