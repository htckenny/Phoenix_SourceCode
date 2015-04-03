/*
 * subsystem.c
 *
 *  Created on: 2015/3/18
 *      Author: rusei
 */

#include <io/nanopower2.h>
#include "subsystem.h"
#include <dev/i2c.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <math.h>
#include "parameter.h"
#include "mpio.h"
#include <csp/csp_endian.h>
#include <csp/csp.h>
void power_OFF_ALL(){

	uint8_t txdata[2];
	txdata[0]=9; //eps outputmask
	txdata[1]=0; /* power on all sub systems  */
	i2c_master_transaction(0, eps_node, &txdata,2, 0, 0,eps_delay);

	io_set(6);
    vTaskDelay(300);
    io_set(0);
    vTaskDelay(1000);
    io_clear(6);
    io_clear(0);

}

void deploy_antenna(){
  uint8_t txdata[2];

  txdata[0] = ant_arm;    // arm ant board
  i2c_master_transaction(0,ant_node,&txdata,1,0,0,0);
  vTaskDelay(100);
  txdata[0] = ant_deploy;    // deploy ant board one by one
  txdata[1] = ant_deploy_timeout;
  i2c_master_transaction(0,ant_node,&txdata,2,0,0,0);
  printf("Antenna Deployed!!\n");
}


void power_control(int device,int stats){
	/* device code:
	 * ADCS = 1
	 * GPS = 2
	 * SEUV = 3
	 * INMS = 4
	 *           */
	unsigned int mode = stats;
	int delay=1;

        eps_output_set_single_req eps_switch;
		eps_switch.mode = (uint8_t)mode;
		eps_switch.delay = csp_hton16((int16_t)delay);

		i2c_frame_t * frame;


		if(device==1){

			/* channel 0 = ADCS 5V */

		eps_switch.channel = 0;

		frame = csp_buffer_get(I2C_MTU);
		frame->dest = eps_node;
		frame->data[0] = EPS_PORT_SET_SINGLE_OUTPUT; // Ping port
		memcpy(&frame->data[1], &eps_switch, sizeof(eps_switch));
		frame->len = 1 + sizeof(eps_switch);
		frame->len_rx = 0;
		frame->retries = 0;
        i2c_send(0, frame, 0);

		/* channel 3 = ADCS 3.3V */

		eps_switch.channel = 3;

		frame = csp_buffer_get(I2C_MTU);
		frame->dest = eps_node;
		frame->data[0] = EPS_PORT_SET_SINGLE_OUTPUT; // Ping port
		memcpy(&frame->data[1], &eps_switch, sizeof(eps_switch));
		frame->len = 1 + sizeof(eps_switch);
		frame->len_rx = 0;
		frame->retries = 0;
        i2c_send(0, frame, 0);

		csp_buffer_free(frame);
		}

		if(device==2){

			/* channel 4 = GPS 3.3V */
		eps_switch.channel = 4;

		frame = csp_buffer_get(I2C_MTU);
		frame->dest = eps_node;
		frame->data[0] = EPS_PORT_SET_SINGLE_OUTPUT; // Ping port
		memcpy(&frame->data[1], &eps_switch, sizeof(eps_switch));
		frame->len = 1 + sizeof(eps_switch);
		frame->len_rx = 0;
		frame->retries = 0;
        i2c_send(0, frame, 0);


		csp_buffer_free(frame);
		}


		if(device==3){

				/* channel 2 = SEUV 5V */

			eps_switch.channel =2;

			frame = csp_buffer_get(I2C_MTU);
			frame->dest = eps_node;
			frame->data[0] = EPS_PORT_SET_SINGLE_OUTPUT; // Ping port
			memcpy(&frame->data[1], &eps_switch, sizeof(eps_switch));
			frame->len = 1 + sizeof(eps_switch);
			frame->len_rx = 0;
			frame->retries = 0;
	        i2c_send(0, frame, 0);

			/* channel 5 = SEUV 3.3V */

			eps_switch.channel = 5;

			frame = csp_buffer_get(I2C_MTU);
			frame->dest = eps_node;
			frame->data[0] = EPS_PORT_SET_SINGLE_OUTPUT; // Ping port
			memcpy(&frame->data[1], &eps_switch, sizeof(eps_switch));
			frame->len = 1 + sizeof(eps_switch);
			frame->len_rx = 0;
			frame->retries = 0;
	        i2c_send(0, frame, 0);

			csp_buffer_free(frame);
			}

         if(device==4){
            /*      INMS Power GPIO Control        */
               if(stats==ON){
        			io_set(5);
        			vTaskDelay(300);
        			io_set(1);
        			vTaskDelay(1000);
        			io_clear(5);
        			io_clear(1);
               }

               if(stats==OFF){

           		io_set(6);
           		vTaskDelay(300);
           		io_set(0);
           		vTaskDelay(1000);
           		io_clear(6);
           		io_clear(0);
               }
         }
}




