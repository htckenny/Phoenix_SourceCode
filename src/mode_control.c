#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
typedef xQueueHandle xSemaphoreHandle;
#include "mode_control.h"

xQueueHandle wod_flag;
xQueueHandle hk_flag;
xQueueHandle inms_flag;
static int on=1;
static int off =0;

int task_on(int type){
	if(type ==1){
		if(wod_flag != 0){
		 if(! xQueueSend( wod_flag, &on ,500 ))
		   printf("Fail to send queue to WOD\n");
		 else return 1;
		}
		else
			wod_flag = xQueueCreate( 1, sizeof( int ) );

	}else if(type==2){
		if(hk_flag != 0){
		 if(! xQueueSend( hk_flag, &on ,500 ))
		   printf("Fail to send queue to hk\n");
		 else return 1;
		}
		else
			hk_flag = xQueueCreate( 1, sizeof( int ) );

	}else if(type==3){
		if(inms_flag != 0){
		 if(! xQueueSend( inms_flag, &on ,500 ))
		   printf("Fail to send queue to INMS\n");
		 else return 1;
		}
		else
			inms_flag = xQueueCreate( 1, sizeof( int ) );

	}else return -1;

	return -1;

}
int task_off(int type){
	if(type ==1){
		if(wod_flag != 0){
		 if(! xQueueSend( wod_flag, &off ,500 ))
		   printf("Fail to send queue to WOD\n");
		 else return 1;
		}
		else
			wod_flag = xQueueCreate( 1, sizeof( int ) );

	}else if(type==2){
		if(hk_flag != 0){
		 if(! xQueueSend( hk_flag, &off ,500 ))
		   printf("Fail to send queue to hk\n");
		 else return 1;
		}
		else
			hk_flag = xQueueCreate( 1, sizeof( int ) );

	}else if(type==3){
		if(inms_flag != 0){
		 if(! xQueueSend( inms_flag, &off ,500 ))
		   printf("Fail to send queue to INMS\n");
		 else return 1;
		}
		else
			inms_flag = xQueueCreate( 1, sizeof( int ) );

	}else return -1;

	return -1;

}


void mode_control(void * pvParameters) {

	wod_flag = xQueueCreate( 1, sizeof( int ) );
	hk_flag = xQueueCreate( 1, sizeof( int ) );
	inms_flag = xQueueCreate( 1, sizeof( int ) );

	while(1){
		vTaskDelay(10000);
		task_on(1);
		vTaskDelay(10000);
		task_off(1);


	}



}
