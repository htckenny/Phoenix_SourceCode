#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <fat_sd/ff.h>
#include "fs.h"



void vTaskfstest(void * pvParameters) {
		
  uint8_t con[]={0x9D,0x00,0x1C,0x96,0x8D,0xCE,0x7E,0x99,0x8D,0xCE,0x41,0x01,0x00,0x0F,0x10,0x41,0x00
		        ,0x14,0x10,0x42,0x00,0x28,0x10,0x43,0x00,0x00,0x11,0x42,0x00,0x14,0x11,0x43,0x00,0x28
		        ,0x11,0x42,0x00,0x00,0x12,0x43,0x00,0x14,0x12,0x42,0x00,0x28,0x12,0x43,0x55,0x05,0x00
		        ,0xFB,0x01,0x01,0x0A,0x00,0x04,0x02,0x02,0x04,0x05,0x01,0x0B,0x01,0x03,0x0A,0x00,0xFC
		        ,0x01,0x04,0x02,0x00,0xFD,0x01,0x05,0x05,0x00,0xFB,0x01,0x06,0x05,0x00,0x53,0x01,0x07
		        ,0x0A,0x00,0xC9,0x01,0x08,0x08,0x00,0x08,0x06,0x09,0x34,0x80,0x64,0x32,0x64,0x1E,0x12
		        ,0x0B,0x01,0x0A,0x0A,0x00,0xFC,0x01,0x0B,0x0A,0x00,0xFD,0x01,0x0C,0x05,0x00,0xFB,0x01
		        ,0x0D,0x05,0x00,0x53,0x01,0x0E,0x0A,0x00,0xC9,0x01,0x0F,0x08,0x00,0x08,0x06,0x10,0x34
		        ,0x80,0x64,0x32,0x64,0x1E,0x12,0x0B,0x01,0x11,0x0A,0x00,0xFC,0x01,0x12,0x0A,0x00,0xFD
		        ,0x01,0x13,0xEF,0x31};
//		uint8_t *conr;
//		int len;
uint8_t cons[]= {0x7D,0x00,0xAD,0x5C,0x07,0xCF,0xAD,0x5C,0x07,0xCF,0x23,0x12,0x00,0x05,0x00,0x41,0x00
		        ,0x06,0x00,0x42,0x00,0x07,0x00,0x43,0x55,0x02,0x00,0xF1,0x01,0x01,0x02,0x00,0x04,0x02
		        ,0x02,0x40,0x02,0x00,0x0B,0x01,0x03,0x02,0x00,0xF2,0x01,0x04,0x04,0x00,0xFE,0x01,0x05
		        ,0x02,0x00,0xF1,0x01,0x06,0x02,0x00,0x04,0x02,0x07,0x15,0x02,0x00,0x53,0x01,0x08,0x14
		        ,0x00,0xC9,0x01,0x09,0x14,0x00,0x08,0x06,0x0A,0xC8,0x8E,0x02,0x00,0x05,0x02,0x00,0x0B
		        ,0x01,0x0B,0x02,0x00,0xF2,0x01,0x0C,0x04,0x00,0xFE,0x01,0x0D,0x02,0x00,0xF1,0x01,0x0E
		        ,0x02,0x00,0x04,0x02,0x0F,0x04,0x02,0x00,0x0B,0x01,0x10,0x02,0x00,0xF2,0x01,0x11,0x04
		        ,0x00,0xFE,0x01,0x12,0x93,0xFF};
uint8_t con6[]= {0x2D,0x00 ,0xF6 ,0xDF ,0x02 ,0xCF ,0xF5 ,0xDF ,0x02 ,0xCF ,0x23 ,0x12 ,0x12 ,0x0F ,0x0E ,0x41 ,0x12,0x0F,0x0E,0x42,0x55 ,0x0A ,0x00 ,0x04 ,0x02 ,0x01 ,0x19 ,0x0A ,0x00 ,0xFE ,0x01 ,0x02 ,0x0A ,0x00 ,0x04 ,0x02 ,0x03 ,0x23 ,0x0A ,0x00 ,0xFE ,0x01 ,0x04 ,0xBE ,0xDD};

uint8_t con7[]={0x48,0x00,0x94,0xD8,0x02,0xCF,0x94,0xD8,0x02,0xCF,0x23,0x12,0x30,0x2B,0x0D,0x41,0x55,0x0A,0x00,0x04,0x02,0x01,0x32,0x0A,0x00,0x04,0x02,0x02,0x32,0x0A,0x00,0x04,0x02,0x03,0x32,0x0A,0x00,0x04,0x02,0x04,0x32,0x0A,0x00,0x04,0x02,0x05,0x32,0x0A,0x00,0x04,0x02,0x06,0x21,0x0A,0x00,0x04,0x02,0x07,0x21,0x0A,0x00,0x04,0x02,0x08,0x21,0x00,0x01,0xFE,0x01,0x09,0xCB,0x2C};

		//inms_script_write(2,"Scheiße");
		inms_script_write(6,con6);
		inms_script_write(7,con7);
		inms_script_write(7,cons);
		inms_script_write(7,con);
		//vTaskDelay(5000);
//		len = inms_script_length(4);
//		conr = inms_script_read(3,len);
//		printf("len = %d\n",len);
		//printf("con = %d\n",sizeof(con));
//		for(int i =0;i<20;i++){
//			printf("0x%02X\n",conr[i]);
//		}
		//printf("0x%02X\n",*conr);
}
