#include <time.h>
#include <fat_sd/ff.h>
#include "fs.h"
#include <dev/i2c.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <util/hexdump.h>
#include <util/csp_buffer.h>
#include <util/log.h>
#include <util/delay.h>
#include <io/nanopower2.h>
#include <io/nanomind.h>
#include <csp/csp.h>
#include <csp/csp_endian.h>
#include "parameter.h"
#include "math.h"
#define __max(a,b) \
   ({__typeof__ (a) _a = (a); \
     __typeof__ (b) _b = (b); \
     _a > _b ? _b : _a; })
#define __min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _b : _a; })
#define E_NO_ERR -1

#define dtime 5000 //1 minute for the real case
uint8_t wod[232];
uint8_t beaconWod [8];

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
	//uint8_t txdataW[8]={};
	uint8_t txdata[19]={};

	unsigned int reg;
    reg=8;
	txdata[0] = reg;
	int rx_length = 43;
	uint8_t rxdata[rx_length];

	if(i2c_master_transaction(0,2, txdata,1,&rxdata,rx_length,2) == E_NO_ERR) { //eps node = 2
		batVoltage = rxdata[8] << 8 | rxdata[9];
		tempEps = ((rxdata[12]<<8|rxdata[13])+(rxdata[14]<<8|rxdata[15])+(rxdata[16]<<8|rxdata[17]))/3 ;
		tempBat = ((rxdata[18]<<8|rxdata[19])+(rxdata[20]<<8|rxdata[21])+(rxdata[22]<<8|rxdata[23]))/3 ;
	}
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



	/*beacon message Node Time[1] Time[2] Q	B5	0	T		W	0	1*/
	uint8_t txdataH[]={0x14, 0x00, 0x00, 0x51, 0x42, 0x35, 0x30, 0x54, 0x57, 0x30, 0x31};
    if(parameters.first_flight>0)
	txdataH[1]=10;
    else
    txdataH[1]=30;

	for (int i=0;i<11;i++)	{
		txdata[i]=txdataH[i];
	}
	for (int i=0;i<8;i++){
		txdata[i+11]=beaconWod[i];
	}
	if(i2c_master_transaction(0,81, &txdata,19,0,0,0) != E_NO_ERR) {
		printf("set beacon fail \r\n");
	}



}


void vTaskwod(void * pvParameters) {

	vTaskDelay(dtime);
	timestamp_t t;
	while(1){
		/*obc get time------------------------------------------*/

		t.tv_sec = 0;
		t.tv_nsec = 0;
		obc_timesync(&t, 6000);
		time_t tt = t.tv_sec;
		printf("OBC time is: %s\r\n", ctime(&tt));

		wod[0]=t.tv_sec;
		wod[1]=t.tv_sec>>8;
		wod[2]=t.tv_sec>>16;
		wod[3]=t.tv_sec>>24;

		//get the 32 dataSet . dtime must be modify to 1 minute, i ->32
		for(int i=1;i<=32;i++)
		{
			getWodFrame(i);
			printf("--------Frame %d get--------\n",i);
			vTaskDelay(dtime);
		}

		if(wod_write(wod)!=0){
			printf("write WOD into FS error");
		}
		hex_dump(&wod,232);
		for(int i = 0;i<232;i++){
			wod[i]=0;
		}

	}
}
