#include <dev/i2c.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


static int slave_node = 6;
#define E_NO_ERR -1

void vTaskI2C(void * pvParameters) {

        int hh=0;
	i2c_frame_t * frame;
	frame = csp_buffer_get(I2C_MTU);
	frame->dest = slave_node;
	frame->data[0] = 3221;
	//frame->data[1] = 0x55;
	frame->len = 1;
	frame->len_rx = 3;
	frame->retries = 0;

	while (hh<5) {
                hh++;
		vTaskDelay(5000);

		if (i2c_send(0, frame, 0) != E_NO_ERR) {
			csp_buffer_free(frame);
		    printf("i2c_send error\r\n");

		}

		if (i2c_receive(0, &frame, 200) == E_NO_ERR) {
			printf("Received a reply from OOO!\r\n");
		} else {
			printf("No reply from OOO hh=%d \r\n",hh);
		}
		


	}
}





/*
#include <dev/i2c.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
static int slave_node = 6;
#define E_NO_ERR -1


	i2c_frame_t * frame;
	frame = csp_buffer_get(I2C_MTU);
	frame->dest = slave_node;
	frame->data[0] = 0x11;
	frame->data[1] = 0x55;
	frame->len = 2;
	frame->len_rx = 3;
	frame->retries = 0;

if (i2c_send(0, frame, 0) != E_NO_ERR) {
			csp_buffer_free(frame);
		    printf("i2c_send error\r\n");

		}

		if (i2c_receive(0, &frame, 200) == E_NO_ERR) {
			printf("Received a reply from OOO!\r\n");
		} else {
			printf("No reply from OOO\r\n");
		}

*/




