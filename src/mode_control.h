#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
typedef xQueueHandle xSemaphoreHandle;


/**
 * Put name_flag into while(name_flag){} to control on and off of a task
 *
 * @param wod_flag for wod task
 * @param hk_flag for phoenix hk task
 * @param inms_flag for INMS script handle task           ...etc
 */
xQueueHandle wod_flag;
xQueueHandle hk_flag;
xQueueHandle inms_flag;
