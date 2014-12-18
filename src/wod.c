#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <time.h>
#include <math.h>

#include <fat_sd/ff.h>
#include "fs.h"
#include <dev/i2c.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <util/error.h>
#include <util/hexdump.h>
#include <util/csp_buffer.h>
#include <util/log.h>
#include <util/driver_debug.h>
#include <util/delay.h>
#include <dev/cpu.h>

#include <io/nanopower2.h>
#include <io/nanomind.h>
#include <csp/csp.h>
#include <csp/csp_endian.h>
#define __max(a,b) \
   ({__typeof__ (a) _a = (a); \
     __typeof__ (b) _b = (b); \
     _a > _b ? _b : _a; })
#define __min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _b : _a; })
#define E_NO_ERR -1

#define dtime 2000 //1 minute for the real case


uint8_t wod[232];
uint8_t beaconWod [8];
uint8_t frame[3][102];


void sa(int byte_no, int bit_no){ //calculate the position of the memory and then store in.
	int num=0;
	switch(bit_no)
	{
		case 0:
			num=128;	break;
		case 1:
			num=64;		break;
		case 2:
			num=32;		break;
		case 3:
			num=16;		break;
		case 4:
			num=8;		break;
		case 5:
			num=4;		break;
		case 6:
			num=2;		break;
		case 7:
			num=1;		break;
		default:
			num = 0;
			printf("error, do not have this bit number");
			break;
	}
	wod[byte_no-1]=wod[byte_no-1]+num;


}
void calbit(int dataSet){  //normally for mode(first bit of 57bits)
	int curbit;
	//32 data set
	if (dataSet>32)
		printf("error, data set no more than 32");

	curbit = 57*(dataSet-1);
	sa(curbit/8+5,curbit%8);//+1(next)+4(32bit time)
	printf("%d\n",curbit/8+5);
	printf("%d\n",curbit%8);

}
void calmulbit(int dataSet, int data, int value){ //for the wod choose which dataset and data and the value
	//data=1:batvol ,data=2:batcurr.............data=7:tempbat
	int curbit;
	int i ,j;
	int rec[8];
	//32 data set
	if (dataSet>32)
		printf("error, data set no more than 32");
	if(value>255)
		printf("value out of range!!\n");
	for(i=0;i<8;i++){
		rec[i]=0;
	}
	for(i=0;i<8;i++){
	if(value==0)
			break;
		if(value>= (1<<(7-i))){
			rec[i]=1;
			value = value - (1<<(7-i));
		}

	}

	for(j=0;j<8;j++){
		if(rec[j]==1){
			curbit = 57*(dataSet-1)+8*(data-1)+j+1;
			sa(curbit/8+5,curbit%8);//+1(next)+4(32bit time)
			//printf("%d\n",curbit/8+5);
			//printf("%d\n",curbit%8);
		}
	}
}
void getWodFrame(int fnum){

	unsigned int mode = 0;
	unsigned int batVoltage = 0;
	unsigned int batCurrent = 0;
	unsigned int bus3v3Current=0;
	unsigned int bus5v0Current=0;
	signed int tempComm=0 ;
	signed int tempEps=0 ;
	signed int tempBat=0 ;


	unsigned int reg;
	uint8_t txdata[1];
	//txdata[0] = reg;
	int rx_length = 43;
	//uint8_t rxdata[rx_length];

//	if(i2c_master_transaction(0,2, txdata,1,&rxdata,rx_length,2) == E_NO_ERR) { //eps node = 2
//		batVoltage = rxdata[8] << 8 | rxdata[9];
//		tempEps = ((rxdata[12]<<8|rxdata[13])+(rxdata[14]<<8|rxdata[15])+(rxdata[16]<<8|rxdata[17]))/3 ;
//		tempBat = ((rxdata[18]<<8|rxdata[19])+(rxdata[20]<<8|rxdata[21])+(rxdata[22]<<8|rxdata[23]))/3 ;
//	}
	reg=22;
	txdata[0] = reg;
	rx_length = 2;
	uint8_t val[rx_length];
	if(i2c_master_transaction(0,80, txdata,1,&val,rx_length,2) == E_NO_ERR) {
		tempComm  =((((val[1]<<8)+val[0])*330)/1023)-50;
	}

	eps_hk_t hk = {};
//	if (eps_hk_get(&hk) >= 0)
//		eps_hk_print(&hk);
//	else
//		printf("CMD_ERROR_FAIL");

	//following the conversion equation comes from the QB50 Whole Orbit Data-Iss2.pdf
	batVoltage = __min(__max(floor(20*(hk.vbatt/1000)-60),255),0);
	batCurrent = __min(__max(floor(127*((hk.cursys-hk.cursun)/1000)+127),255),0);

	bus3v3Current = __min(__max(floor(40*((hk.curout[0]+hk.curout[1]+hk.curout[2])/1000)),255),0);
	bus5v0Current = __min(__max(floor(40*((hk.curout[3]+hk.curout[4]+hk.curout[5])/1000)),255),0);
	tempEps = __min(__max(floor(4*(((hk.temp[0]+hk.temp[1]+hk.temp[2]+hk.temp[3])/4)/1000)+60),255),0);
	tempBat = __min(__max(floor(4*(((hk.temp[4]+hk.temp[5])/2)/1000)+60),255),0);;


	if (mode)	calbit(fnum);		// wodd.dataSet[fnum].mode = 0 or 1;
	calmulbit(fnum,1,batVoltage);	// wodd.dataSet[fnum].bat_voltage = batVoltage;
	printf("bV = %d\n",batVoltage );
	calmulbit(fnum,2,batCurrent);	// wodd.dataSet[fnum].bat_current=0;
	printf("BC = %d\n",batCurrent );
	calmulbit(fnum,3,bus3v3Current);// wodd.dataSet[fnum].bus3v3_current=0;
	calmulbit(fnum,4,bus5v0Current);// wodd.dataSet[fnum].bus5v0_current=0;
	calmulbit(fnum,5,tempComm);		// wodd.dataSet[fnum].comm_temp= tempComm;
	calmulbit(fnum,6,tempEps);		// wodd.dataSet[fnum].eps_temp = tempEps;
	calmulbit(fnum,7,tempBat);		// wodd.dataSet[fnum].battery_temp = tempBat;
	//for beacon

	beaconWod[0]= mode;
	beaconWod[1]= batVoltage;
	beaconWod[2]= batCurrent;
	beaconWod[3]= bus3v3Current;
	beaconWod[4]= bus5v0Current;
	beaconWod[5]= tempComm;
	beaconWod[6]= tempEps;
	beaconWod[7]= tempBat;
//	int re = wodBeaconWrite(beaconWod);
//	if(re!=-1)
//	{
//		printf("write beacon wod");
//	}
}
void vTaskBeacon(void * pvParameters) {
	uint8_t txdataW[8];
	uint8_t txdata[19];
	for(int i =0;i<8;i++)
	{
		txdataW[i]=0;
	}
	vTaskDelay(2000);
	while(1){
		/*                                               1D = 30m      */
		/*beacon message Node Time[1] Time[2] Q	B		5			0			T			W		0			1*/
		uint8_t txdataH[]={0x14, 0x1D, 0x00, 0x51, 0x42, 0x35, 0x30, 0x54, 0x57, 0x30, 0x31};

		for(int i=0;i<8;i++){
			txdataW[i] = beaconWod[i];
		}
		for (int i=0;i<11;i++)	{
			txdata[i]=txdataH[i];
		}
		for (int i=0;i<8;i++){
			txdata[i+11]=txdataW[i];
		}
		if(i2c_master_transaction(0,81, txdata,23,0,0,0) == E_NO_ERR) {
			printf("Set AX.25 beacon with default callsigns");
		}
		else{
			printf("set beacon fail \r\n");
		}
		/*beacon message*/
	}
}
void vTaskwod(void * pvParameters) {
	int dataSetNum= 0;
	int i;
//	int snum=0; //series number
//	int day=0;
	int re = 0;
	for(int i = 0;i<232;i++){
		wod[i]=0;
	}

	vTaskDelay(dtime);

	while(dataSetNum<2){
		/*obc get time------------------------------------------*/
		timestamp_t t;
		t.tv_sec = 0;
		t.tv_nsec = 0;
		obc_timesync(&t, 6000);
		time_t tt = t.tv_sec;
		printf("OBC time is: %s\r\n", ctime(&tt));

		//wodd.time = ctime(&tt) ;
		//wodd.time = t.tv_sec;
		//printf("time : %d\n",wodd.time);

		//first four byte for time(little endian)
		//wod[232]  is the whole package
		wod[0]=t.tv_sec;
		wod[1]=t.tv_sec>>8;
		wod[2]=t.tv_sec>>16;
		wod[3]=t.tv_sec>>24;



//		getWodFrame(dataSetNum);
//		printf("--------Frame %d get--------\n",i);
//		vTaskDelay(dtime);

		//get the 32 dataSet . dtime must be modify to 1 minute, i ->32
		for(i=1;i<=5;i++)
		{
			getWodFrame(i);
			printf("--------Frame %d get--------\n",i);
			vTaskDelay(dtime);
		}

		re = wod_write(wod);
		if(re!=-1){
			printf("write WOD into FS ");
		}
//		for (int j=0;j<3;j++)
//		{
//			snum++;
//			//printf("%d", snum);
//			frame[j][0]=0;// the first two byte header is the day
//			frame[j][1]=0;
//			frame[j][2]=snum>>8;
//			frame[j][3]=snum;
//			for(i = 0;i<98;i++){
//				frame[j][i+4]=wod[i+j*98];
//			}
//		}
//		char cday[10];
//		int re = 0;
//		sprintf(cday,"%d", day); //change the day from integer to char
//		//printf(cday);
//		re = wod_write(cday,frame[0],0); //write the frame[0] into the file system
//		if (re) printf("Error with frame1\n");
//		else printf("Good with frame1\n");
//		vTaskDelay(dtime);
//		re = wod_write(cday,frame[1],1);
//		if (re) printf("Error with frame2\n");
//		else printf("Good with frame2\n");
//		vTaskDelay(dtime);
//		re = wod_write(cday,frame[2],1);
//		if (re) printf("Error with frame3\n");
//		else printf("Good with frame3\n");
//
//
//		uint8_t rewod[102];
//		for(i=0;i<102;i++)
//		{
//			rewod[i]=0;
//		}
//		for (int i =1;i<=6;i++){
//			wod_down(0,"0",i,rewod);  //read the data from the File System, second parameter is the day.
//			hex_dump(rewod,102);
//		}

		dataSetNum++;
	}
}
