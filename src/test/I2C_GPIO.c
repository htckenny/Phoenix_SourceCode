/*
 * Bit-Banging I2C using GPIO
 *
 *  Created on: 	2015/04/04
 *  Last updated: 	2015/04/04
 *      Author: Kenny Huang
 */
#include <stdio.h>
#include <dev/arm/at91sam7.h>
#include <command/command.h>
#include "mpio.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <util/hexdump.h>	
 #include  "I2C_GPIO.h"
/**
 * GPIO 0 1 5 6 hava already be used as INMS power relay
 */
#define SCL_PORT  5
#define SDA_PORT  0

#define ADDR_ADCS  0x24 /* The Address of ADCS */
#define I2C_DELAY_TIME  0.1 /* us */

#define ACK   1
#define NO_ACK   0
/**
 * This function initialize the GPIO pin, and set the level to high, complete the initialization of the I2C
 */
void i2c_mpio_init(void) {

	/* Enable SDA & SCL gpio port */
	io_init(SCL_PORT, 1);
	io_init(SDA_PORT, 1);
	/* Set SDA & SCL to High */
	io_set(SDA_PORT);
	io_set(SCL_PORT);
}

void i2c_mpio_start(void) {

 /* I2C start sequence is defined as 
  * a High to Low Transition on the data
  * line as the CLK pin is high */
  
	io_set(SDA_PORT);  /* SDA: High */
	io_set(SCL_PORT);  /* SCL: High */
	vTaskDelay(I2C_DELAY_TIME);
	 
	io_clear(SDA_PORT);  /* SDA: Low */
	io_clear(SCL_PORT);  /* SCL: Low */
	vTaskDelay(I2C_DELAY_TIME);
}
void i2c_mpio_stop(void) {
  
 /* I2C stop sequence is defined as 
  * data pin is low, then CLK pin is high,
  * finally data pin is high. */

	 io_clear(SDA_PORT);  	/* SDA: Low */
	 io_set(SCL_PORT);  	/* SCL: High */
	 io_set(SDA_PORT);  	/* SDA: High */
 
}

void i2c_mpio_write(unsigned char data) {
 /* An I2C output byte is bits 7-0
   * (MSB to LSB).  Shift one bit at a time
  * to the MDO output, and then clock the
  * data to the I2C Slave */

	unsigned char i;

	/* Write to slave */
	for(i = 0; i < 8; i++) {
		/* Send data bit */
		if ((data & 0x80) == 128){
			io_set(SDA_PORT);
		}
		else if((data & 0x80) == 0){
			io_clear(SDA_PORT);
		}

		data <<= 1;   /* Shift one bit */
		// hex_dump(data,2);
		io_set(SCL_PORT); /* SCL: High */
		// vTaskDelay(0.01);		
		io_clear(SCL_PORT); /* SCL: Low */
		// vTaskDelay(0.01);
	}

	/* Read ACK bit from slave */
	// printf("%d \n",io_get(SDA_PORT));x
	// if(io_get(SDA_PORT)){
	io_get(SDA_PORT);
	io_set(SCL_PORT);  /* SCL: High */
	// vTaskDelay(I2C_DELAY_TIME);
	io_clear(SCL_PORT);  /* SCL: Low */
	// vTaskDelay(0.01);
	// }
	
	 
}
unsigned char i2c_mpio_read(unsigned char send_ack) {
	unsigned char i, data;
	data = 0x00;
	/* Read from slave */
	for(i = 0; i < 8; i++) {
		data <<= 1;   				/* Shift one bit */
		// printf("sda = %d\n", io_get(SDA_PORT));
		data |= io_get(SDA_PORT); 		/* Read data bit */
		io_set(SCL_PORT); 			/* SCL: High */
		// vTaskDelay(I2C_DELAY_TIME);
		io_clear(SCL_PORT); 			/* SCL: Low */
		// vTaskDelay(I2C_DELAY_TIME);
 	}

 /* Send ACK bit to slave */
 	// printf("send_ack = %x\n",send_ack);
	if(send_ack)
		io_clear(SDA_PORT); 			/* SDA: Low */
	else
		io_set(SDA_PORT); 			/* SDA: High */

	// vTaskDelay(I2C_DELAY_TIME);	
	// io_get(SDA_PORT);	
	io_set(SCL_PORT); 				/* SCL: High */
	io_clear(SCL_PORT);  				/* SCL: Low */
	// vTaskDelay(I2C_DELAY_TIME);
	// printf("sda= %d\n",io_get(SDA_PORT));
	// io_init(SDA_PORT, 0);	
	
 return data;
 
}
/**
 * [i2c_adcs description]
 * @param  txReg [The register number being writen, eg. 0x80]unsigned char txReg,
 * @param  rxNum [the number of recieved byte]
 * @return       [description]
 */
int send_i2c_adcs(unsigned char reg1,unsigned char reg2, int rxNum) {

	unsigned char receivedByte[rxNum];
	// for(int i=0;i<rxNum;i++)
	// 	receivedByte[i]=0x00;	
	i2c_mpio_start();
	i2c_mpio_write(ADDR_ADCS);  /* Ask ADCS write */
	// printf("Ask ADCS Write\n") ;
	i2c_mpio_write(reg1);		/* Ask ADCS's Identification information for this node*/
	if (reg2!=0){
		i2c_mpio_write(reg2);
	}
	// printf("Write 0x80 to register\n") ;
	i2c_mpio_stop();
	vTaskDelay(0.05);
	i2c_mpio_start();

	i2c_mpio_write(ADDR_ADCS+1); /* Ask ADCS read */
	// printf("Ask ADCS Read\n") ;
	for (int i=0;i<rxNum-1;i++){
		receivedByte[i] = i2c_mpio_read(ACK);
		io_set(SDA_PORT);
		vTaskDelay(0.05);
	}
	receivedByte[rxNum-1]=i2c_mpio_read(NO_ACK);
	io_set(SDA_PORT);
	vTaskDelay(0.05);
	i2c_mpio_stop();
	hex_dump(receivedByte,rxNum);
	
 	return 1;
 
}
void ADCS_I2C(void * pvParameters) {
	i2c_mpio_init();
	while(1){
		// send_i2c_adcs(0x03,0x01,0);
		// printf("\n");
		// vTaskDelay(5000);
		send_i2c_adcs(0x80,0x00,8);
		vTaskDelay(5000);
	}
}
