#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
typedef xQueueHandle xSemaphoreHandle;
#include <freertos/semphr.h>

xSemaphoreHandle flag;
xSemaphoreHandle flagx;
 xQueueHandle xQueue;
 xTaskHandle test1;
 xTaskHandle test2;
