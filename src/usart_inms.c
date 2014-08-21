#include <dev/usart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <fat_sd/ff.h>
#include "fs.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

char uchar[];
uint8_t con[]={0xF1,0x01,0x01};
int a = 10000;
char inmscmd[]= {0xF1,0x01,0x01};
char cmd1[] = {0x04};
char cmd2[] = {0x02};
char cmd3[] = {0x02};
char cmd4[] = {0x40};
char inmscmd1[]= {0x04,0x02,0x02,0x40};
char s[255];


int nums;
void usart_inms(void * pvParameters) {    /// choose the name you want

	vTaskDelay(5000);
	printf("send uart to port 2\n\r");

	int i = 0;
	//uint8_t con[]={0xF1};



//	usart_putc(2,cmd1);
//	usart_putc(2,cmd2);
//	usart_putc(2,cmd3);
//	usart_putc(2,cmd4);
	//vTaskDelay(1000);
	//printf("%d",strlen(&inmscmd1));

	usart_putstr(2,&inmscmd1,4);

	while(1){
		vTaskDelay(2000);
		nums = usart_messages_waiting(2);
		printf(" %d \n\r ",nums);
		if (nums != 0){
			printf("seems get something!\n\r");



	        for(int f=0;f < nums;f++){
	        	uchar[f] = usart_getc(2);
	        	//hex_dump(uchar,f);
//	        	printf("%x",uchar[f]);
	        	printf("0x%02x", uchar[f]);
	        }
				printf("\n");
		}
		hex_dump(uchar,nums);
		i = i+1;
	}
}
void i2s(int i,char *s)	// Convert Integer to String
{
	char sign;
	short len;
	char *p;
	sign='+';
	len=0;
	p=s;
	if(i<0){sign='-';i=-i;}
	do{*s=(i%10)+'0';s++;len++;i/=10;}while(i!=0);
	if(sign=='-'){*s='-';s++;len++;}
	for(i=0;i<len/2;i++){p[len]=p[i];p[i]=p[len-1-i];p[len-1-i]=p[len];}
	p[len]=0;
}

