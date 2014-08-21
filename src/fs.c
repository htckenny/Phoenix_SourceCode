#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <fat_sd/ff.h>
#include "fs.h"
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <util/hexdump.h>

//FRESULT scan_files (char* path);
//#define F_PUTS			0	//write string into the file
//#define F_READ			1	//read the string out of the file
//#define F_UNLINK		0	//delete the file
//#define SCAN_FILES		1	//scan the directory
#define No_Error		0

FATFS fs;
FRESULT res;
FIL file;
UINT br,bw;
uint8_t buffer[4096];
FILINFO *fno;
int wod_write(char day[], uint8_t frameCont[], int first)
{
	f_mount(0, &fs);
	char fileName[20];
	char s[]="0:/wod/wod_";
	strcpy(fileName,s);
	strcat(fileName,day);
	strcat(fileName,".bin");
	printf("%s\n",fileName);
//	for (int i =0;i<20;i++)
//		printf("%c\n",fileName[i]);
//	
	if(first ==0 ){
		res = f_unlink(fileName);	  //先刪除
		if(res!=FR_OK)
		{
			printf("\r\n f_unlink() fail .. \r\n");
		}else{
			printf("\r\n f_unlink() success .. \r\n");
		}
	}



//	res = f_mkdir("wod");
//	if(res!=FR_OK)
//	{
//		printf("\r\n f_mkdir() fail .. \r\n");
//	}else{
//		printf("\r\n f_mkdir() success .. \r\n");
//	}


	res = f_open(&file,fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
	}else{
		printf("\r\n f_open() success .. \r\n");
	}

	//將pointer指向文件最後面
	res = f_lseek(&file,file.fsize);
	
	//hex_dump(frameCont,102);
	res = f_write(&file,frameCont,102,&bw);
	
	if(res!=FR_OK){
		printf("\r\n f_write() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return -1;
	}
	else{
		printf("\r\n f_write() success .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return No_Error;
	}		

//	return 0;
}
void wod_down(int type,char day[],int serNum,void * txbuf){ //secNum still not work yet.
	//type=0 :WOD  type=1 :INMS type=2 :EUV type=3 :HK
	f_mount(0, &fs);
	int snum;
	char fileName[100];
	char s[]="0:/wod/wod_";
	strcpy(fileName,s);
	strcat(fileName,day);
	strcat(fileName,".bin");
	printf("%s\n",fileName);
	res = f_open(&file,fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
	}else{
		printf("\r\n f_open() success .. \r\n");
	}
	
	while(1){
		res = f_read(&file,buffer,102,&br);
	   
		if(res == FR_OK)
		{	
			printf("\r\n f_read() success .. \r\n");
		}else{
			printf("\r\n f_read() fail .. \r\n");
		}
		snum = buffer[3] | (buffer[2]<<8);
		//printf("snum = %d, serNum = %d\n", snum, serNum);
		if (snum==serNum)
		{
			memcpy(txbuf,&buffer,102);
		}
		if(f_eof(&file)){break;}
	}

//	for(int i=0;i<102;i++)
//	{
//		printf("%02x\n",buffer[i]);
//	}
	//printf("ee11\n");



	f_close(&file);
	f_mount(0, NULL);


}


//buffNum = the number which buffer should write in. 
//scriptCon = the whole script contents.

int inms_data_write(char day[], uint8_t frameCont[])
{
	f_mount(0, &fs);
	char fileName[100];
	char s[]="0:/inms/inms_";
	strcpy(fileName,s);
	strcat(fileName,day);
	strcat(fileName,".bin");
	printf("%s\n",fileName);
//
//	res = f_unlink(fileName);	  //先刪除


	res = f_open(&file,fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
	}else{
		printf("\r\n f_open() success .. \r\n");
	}

	//將pointer指向文件最後面
	res = f_lseek(&file,file.fsize);

	hex_dump(frameCont,174);
	res = f_write(&file,frameCont,174+22,&bw);

	if(res!=FR_OK){
		printf("\r\n f_write() fail .. \r\n");
		//return -1;
	}
	else{
		printf("\r\n f_write() success .. \r\n");
		//return No_Error;
	}
	f_close(&file);
	f_mount(0, NULL);
	return 0;
}
int inms_script_write(int buffNum,uint8_t scriptCont[]){
	//int n;
	f_mount(0, &fs);
	char fileName[100];
	
	//char *fileName;
	if(buffNum==7){
		strcpy(fileName,"0:/Running.bin");}
	else if(buffNum==0){
		strcpy(fileName,"0:/Idle0.bin");}
	else if(buffNum==1){
		strcpy(fileName,"0:/Idle1.bin");}
	else if(buffNum==2){
		strcpy(fileName,"0:/Idle2.bin");}
	else if(buffNum==3){
		strcpy(fileName,"0:/Idle3.bin");}
	else if(buffNum==4){
		strcpy(fileName,"0:/Idle4.bin");}
	else if(buffNum==5){
		strcpy(fileName,"0:/Idle5.bin");}
	else if(buffNum==6){
		strcpy(fileName,"0:/Idle6.bin");}
	
	//printf("%s\n",des);
	printf("%s\n",fileName);
	
	res = f_unlink(fileName);	  //先刪除

	if(res!=FR_OK)
	{
		printf("\r\n f_unlink() fail .. \r\n");
	}else{
		printf("\r\n f_unlink() success .. \r\n");
	}

	res = f_open(&file,fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
	
	//res = f_open(&file,"0:/data.txt",FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
	}else{
		printf("\r\n f_open() success .. \r\n");
	}

	//將pointer指向文件最後面
	res = f_lseek(&file,file.fsize);

	//n = f_puts(scriptCont, &file) ;//寫入字串	
	//if(n<1)  //判斷是否成功
	//{
	//	printf("\r\n f_puts() fail .. \r\n");
	//}else{
	//	printf("\r\n f_puts() success .. \r\n");
	//}
	hex_dump(scriptCont,157);
	res = f_write(&file,scriptCont,(int)scriptCont[0],&bw);
	if(res!=FR_OK)
	{
		printf("\r\n f_write() fail .. \r\n");
		//return -1;
	}else{
		printf("\r\n f_write() success .. \r\n");
		//return
	}
	f_close(&file);
	f_mount(0, NULL);
	return No_Error;
}
void inms_script_read(int buffNum,int packlength,void * txbuf){
	//int n;
	f_mount(0, &fs);

	char fileName[100];
	if(buffNum==7){
		strcpy(fileName,"0:/Running.bin");}
	else if(buffNum==0){
		strcpy(fileName,"0:/Idle0.bin");}
	else if(buffNum==1){
		strcpy(fileName,"0:/Idle1.bin");}
	else if(buffNum==2){
		strcpy(fileName,"0:/Idle2.bin");}
	else if(buffNum==3){
		strcpy(fileName,"0:/Idle3.bin");}
	else if(buffNum==4){
		strcpy(fileName,"0:/Idle4.bin");}
	else if(buffNum==5){
		strcpy(fileName,"0:/Idle5.bin");}
	else if(buffNum==6){
		strcpy(fileName,"0:/Idle6.bin");}
	
	res = f_open(&file, fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
	//printf("test2");
	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
	}else{
		printf("\r\n f_open() success .. \r\n");
	}
	
	//int i =0; 
	
	
	
	while(1){
		res = f_read(&file,buffer,packlength,&br); //20->packlength
	   
		if(res == FR_OK)
		{	
			printf("\r\n f_read() success .. \r\n");

		}else{
			printf("\r\n f_read() fail .. \r\n");
		}
		
		if(f_eof(&file)){break;}
	}

	memcpy(txbuf,&buffer,packlength);

	
	f_close(&file);
	f_mount(0, NULL);
	//printf("buff = %d\n",sizeof(buffer));

}
//int inms_script_run(int buffNum){
//	
//	
//}
int inms_script_length(int buffNum){
	
	f_mount(0, &fs);
	char fileName[100];
	if(buffNum==7){
		strcpy(fileName,"0:/Running.bin");}
	else if(buffNum==0){
		strcpy(fileName,"0:/Idle0.bin");}
	else if(buffNum==1){
		strcpy(fileName,"0:/Idle1.bin");}
	else if(buffNum==2){
		strcpy(fileName,"0:/Idle2.bin");}
	else if(buffNum==3){
		strcpy(fileName,"0:/Idle3.bin");}
	else if(buffNum==4){
		strcpy(fileName,"0:/Idle4.bin");}
	else if(buffNum==5){
		strcpy(fileName,"0:/Idle5.bin");}
	else if(buffNum==6){
		strcpy(fileName,"0:/Idle6.bin");}
	
	res = f_open(&file, fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
	//printf("test2");
	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
	}else{
		printf("\r\n f_open() success .. \r\n");
	}
	
	//int i =0; 
	int packlength =0;		
	
	//while(1){
		res = f_read(&file,buffer,1,&br);
	   
		if(res == FR_OK)
		{	
			//printf("\r\n f_read() success .. \r\n");
			//if(i==0){
				packlength = buffer[0];
		//	}
		//	i++;
				//printf("%" PRIu8 "\n",buffer[0]);
		}else{
			printf("\r\n f_read() fail .. \r\n");
		}
		
		//if(f_eof(&file)){break;}
	//}
	
	printf("packlength : %d\n",packlength);
	f_close(&file);
	f_mount(0, NULL);
	return packlength;
}



