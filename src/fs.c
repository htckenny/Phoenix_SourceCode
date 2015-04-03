#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <fat_sd/ff.h>
#include "fs.h"
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <util/hexdump.h>
#include "parameter.h"
#include <dev/i2c.h>
#include "Tele_function.h"
#include "subsystem.h"
#include <csp/csp_endian.h>


FATFS fs;
FRESULT res;
FIL file;
UINT br,bw;
uint8_t buffer[300];
FILINFO *fno;

int downlink_data_before_t(uint8_t datatype,uint32_t time1){
	/*      Now T1 is not convert yet, please check T1 endian to h      */

	char * fileName;
    uint8_t size;
    uint32_t datatime;
	if(datatype==1){
		fileName="0:/hk.bin";
	    size=hk_length;
	}else if(datatype==2){
		fileName="0:/inms.bin";
		size=inms_data_length;
	}else if(datatype==3){
		fileName="0:/seuv.bin";
		size=seuv_length;
	}else if(datatype==5){
		fileName="0:/wod.bin";
		size=wod_length;
	}else
		return Error;

	f_mount(0, &fs);
	res = f_open(&file,fileName,FA_READ|FA_WRITE );
		if(res!=FR_OK)
		{
			printf("\r\n f_open() fail .. \r\n");
			return Error;
		}

	while(1){


	res = f_read(&file,&buffer,size,&br);
	if(res == FR_OK){
      memcpy(&datatime,&buffer,4);
      datatime=csp_ntoh32(datatime);

      if((datatime<time1)|(datatime==time1))
    	  SendDataWithCCSDS_AX25(datatype,&buffer[0]);
	}else
	break;

    if(f_eof(&file)){
		f_close(&file);
        f_mount(0, NULL);
    	return No_Error;
    }
}


		f_close(&file);
        f_mount(0, NULL);
        return Error;
}
int downlink_data_after_t(uint8_t datatype,uint16_t time1){
	/*      Now T1 is not convert yet, please check T1 endian to h      */
	char * fileName;
	    uint8_t size;
	    uint32_t datatime;
		if(datatype==1){
			fileName="0:/hk.bin";
		    size=hk_length;
		}else if(datatype==2){
			fileName="0:/inms.bin";
			size=inms_data_length;
		}else if(datatype==3){
			fileName="0:/seuv.bin";
			size=seuv_length;
		}else if(datatype==5){
			fileName="0:/wod.bin";
			size=wod_length;
		}else
			return Error;

		f_mount(0, &fs);
		res = f_open(&file,fileName,FA_READ|FA_WRITE );
			if(res!=FR_OK)
			{
				printf("\r\n f_open() fail .. \r\n");
				return Error;
			}

		while(1){


		res = f_read(&file,&buffer,size,&br);
		if(res == FR_OK){
	      memcpy(&datatime,&buffer,4);
	      datatime=csp_ntoh32(datatime);
	      if((datatime>time1)|(datatime==time1))
	    	 SendDataWithCCSDS_AX25(datatype,&buffer[0]);
		}else
		break;

	    if(f_eof(&file)){
			f_close(&file);
	        f_mount(0, NULL);
	    	return No_Error;
	    }
	}


			f_close(&file);
	        f_mount(0, NULL);
	        return Error;
}
int downlink_data_between_t(uint8_t datatype,uint16_t time1,uint16_t time2){
	char * fileName;
	    uint8_t size;
	    uint32_t datatime;
		if(datatype==1){
			fileName="0:/hk.bin";
		    size=hk_length;
		}else if(datatype==2){
			fileName="0:/inms.bin";
			size=inms_data_length;
		}else if(datatype==3){
			fileName="0:/seuv.bin";
			size=seuv_length;
		}else if(datatype==5){
			fileName="0:/wod.bin";
			size=wod_length;
		}else
			return Error;

		f_mount(0, &fs);
		res = f_open(&file,fileName,FA_READ|FA_WRITE );
			if(res!=FR_OK)
			{
				printf("\r\n f_open() fail .. \r\n");
				return Error;
			}

		while(1){


		res = f_read(&file,&buffer,size,&br);
		if(res == FR_OK){
	      memcpy(&datatime,&buffer,4);
	      datatime=csp_ntoh32(datatime);
	      if((datatime<time1)|(datatime==time1))
	    	  if((datatime>time2)|(datatime==time2))
	    		SendDataWithCCSDS_AX25(datatype,&buffer[0]);

	      if((datatime<time2)|(datatime==time2))
	 	      if((datatime>time1)|(datatime==time1))
	 	    	SendDataWithCCSDS_AX25(datatype,&buffer[0]);

		}else
		break;

	    if(f_eof(&file)){
			f_close(&file);
	        f_mount(0, NULL);
	    	return No_Error;
	    }
	}


			f_close(&file);
	        f_mount(0, NULL);
	        return Error;
}


int inms_data_write(uint8_t frameCont[] )
{
	f_mount(0, &fs);
	char fileName[]="0:/inms.bin";

	res = f_open(&file,fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
	}
	//將pointer指向文件最後面
	f_lseek(&file,file.fsize);

	res = f_write(&file,&frameCont,inms_data_length,&bw);

	if(res!=FR_OK){
		printf("\r\n inms_write() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;
	}
	else{
		printf("\r\n inms_write() success .. \r\n");
		f_close(&file);

		f_mount(0, NULL);
		return No_Error;
	}
}

int inms_data_read(int serial,void * txbuf){   // serial =1~N

	if(serial==0) serial=1;
	f_mount(0, &fs);
	char fileName[20];
	char s[]="0:/inms/inms_";
	strcpy(fileName,s);
	int head = (serial-1)/100;
	char num[5];
	sprintf(num,"%d", head);
	strcat(fileName,num);
	strcat(fileName,".bin");


	res = f_open(&file,fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
	}else{
		printf("\r\n f_open() success .. \r\n");
	}
        int tmp = (serial-1) % 100;
	    f_lseek(&file,tmp*inms_data_length);
		res = f_read(&file,&buffer,inms_data_length,&br);

	if(res != FR_OK){
		printf("\r\n wod_read() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;}
	else{
		memcpy(txbuf,&buffer,inms_data_length);
		f_close(&file);
		f_mount(0, NULL);
		return No_Error;
		}
}

int inms_data_delete(){
	f_mount(0, &fs);
	char fileName[]="0:/inms.bin";
	res = f_unlink(fileName);	  //先刪除

	if(res!=FR_OK)
		{
			printf("\r\n f_unlink() fail .. \r\n");
			return Error;
		}else{
			printf("\r\n f_unlink() success .. \r\n");
			return No_Error;
		}
}

int inms_script_write(int buffNum,uint8_t scriptCont[]){

	f_mount(0, &fs);
	char fileName[100];
	
	if(buffNum==8){
		strcpy(fileName,"0:/INMS/Running.bin");}
	else if(buffNum==0){
		strcpy(fileName,"0:/INMS/Idle0.bin");}
	else if(buffNum==1){
		strcpy(fileName,"0:/INMS/Idle1.bin");}
	else if(buffNum==2){
		strcpy(fileName,"0:/INMS/Idle2.bin");}
	else if(buffNum==3){
		strcpy(fileName,"0:/INMS/Idle3.bin");}
	else if(buffNum==4){
		strcpy(fileName,"0:/INMS/Idle4.bin");}
	else if(buffNum==5){
		strcpy(fileName,"0:/INMS/Idle5.bin");}
	else if(buffNum==6){
		strcpy(fileName,"0:/INMS/Idle6.bin");}

	printf("%s\n",fileName);
	
	res = f_unlink(fileName);	  //先刪除

	if(res!=FR_OK)
	{
		printf("\r\n f_unlink() fail .. \r\n");
	}else{
		printf("\r\n f_unlink() success .. \r\n");
	}

	res = f_open(&file,fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
	

	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
	}else{
		printf("\r\n f_open() success .. \r\n");
	}

	//將pointer指向文件最後面
	res = f_lseek(&file,file.fsize);

	hex_dump(scriptCont,157);
	res = f_write(&file,&scriptCont,(int)scriptCont[0],&bw);
	if(res!=FR_OK)
	{
		printf("\r\n f_write() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;
	}else{
		printf("\r\n f_write() success .. \r\n");

	}
	f_close(&file);
	f_mount(0, NULL);
	return No_Error;
}
void inms_script_read(int buffNum,int packlength,void * txbuf){

	f_mount(0, &fs);

	char fileName[100];
	if(buffNum==8){
		strcpy(fileName,"0:/INMS/Running.bin");}
	else if(buffNum==0){
		strcpy(fileName,"0:/INMS/Idle0.bin");}
	else if(buffNum==1){
		strcpy(fileName,"0:/INMS/Idle1.bin");}
	else if(buffNum==2){
		strcpy(fileName,"0:/INMS/Idle2.bin");}
	else if(buffNum==3){
		strcpy(fileName,"0:/INMS/Idle3.bin");}
	else if(buffNum==4){
		strcpy(fileName,"0:/INMS/Idle4.bin");}
	else if(buffNum==5){
		strcpy(fileName,"0:/INMS/Idle5.bin");}
	else if(buffNum==6){
		strcpy(fileName,"0:/INMS/Idle6.bin");}
	
	res = f_open(&file, fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );

	if(res!=FR_OK)
	{
		printf("f_open() fail .. \r\n");
	}else{
		printf("f_open() success .. \r\n");
	}

	while(1){
		res = f_read(&file,&buffer,packlength,&br); //20->packlength
	   
		if(res == FR_OK){
			printf("f_read() success .. \r\n");
		}
		else{
			printf("f_read() fail .. \r\n");
		}
		if(f_eof(&file)){break;}
	}

	memcpy(txbuf,&buffer,packlength);

	
	f_close(&file);
	f_mount(0, NULL);

}

int inms_script_length(int buffNum){
	
	f_mount(0, &fs);
	int packlength =0;
	char fileName[100];
	if(buffNum==8){
		strcpy(fileName,"0:/INMS/Running.bin");}
	else if(buffNum==0){
		strcpy(fileName,"0:/INMS/Idle0.bin");}
	else if(buffNum==1){
		strcpy(fileName,"0:/INMS/Idle1.bin");}
	else if(buffNum==2){
		strcpy(fileName,"0:/INMS/Idle2.bin");}
	else if(buffNum==3){
		strcpy(fileName,"0:/INMS/Idle3.bin");}
	else if(buffNum==4){
		strcpy(fileName,"0:/INMS/Idle4.bin");}
	else if(buffNum==5){
		strcpy(fileName,"0:/INMS/Idle5.bin");}
	else if(buffNum==6){
		strcpy(fileName,"0:/INMS/Idle6.bin");}
	
	res = f_open(&file, fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );

	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
	}else{
		printf("\r\n f_open() success .. \r\n");
	}

	for(int i =0;i<300;i++){
		buffer[i]=0;
	}

		res = f_read(&file,buffer,1,&br);
	   printf("buffer = %02X\n",buffer[0]);
		if(res == FR_OK){
			if(buffer[0] != 0)
				packlength = buffer[0];
			else
				packlength = 0;
		}else{
			printf("\r\n f_read() fail .. \r\n");
		}
	
	printf("packlength : %d\n",packlength);
	f_close(&file);
	f_mount(0, NULL);
	return packlength;
}


int wod_write(uint8_t * frameCont )
{
	f_mount(0, &fs);
	char fileName[]="0:/wod.bin";

	res = f_open(&file,fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
	}

	f_lseek(&file,file.fsize);
	res = f_write(&file,frameCont,wod_length,&bw);

	if(res!=FR_OK){
		printf("\r\n wod_write() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;
	}
	else{
		printf("\r\n wod_write() success .. \r\n");

		f_close(&file);
		f_mount(0, NULL);
		return No_Error;
	}
}

int wod_read(int serial,void * txbuf){   // serial =1~N

	if(serial==0) serial=1;
	f_mount(0, &fs);
	char fileName[20];
	char s[]="0:/wod/wod_";
	strcpy(fileName,s);
	int head = (serial-1)/100;
	char num[5];
	sprintf(num,"%d", head);
	strcat(fileName,num);
	strcat(fileName,".bin");


	res = f_open(&file,fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
	}else{
		printf("\r\n f_open() success .. \r\n");
	}
        int tmp = (serial-1) % 100;
	    f_lseek(&file,tmp*wod_length);
		res = f_read(&file,&buffer,wod_length,&br);

	if(res != FR_OK){
		printf("\r\n wod_read() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;}
	else{
		memcpy(txbuf,&buffer,wod_length);
		f_close(&file);
		f_mount(0, NULL);
		return No_Error;
		}
}

int wod_delete(){
	f_mount(0, &fs);
	char fileName[]="0:/wod.bin";
	res = f_unlink(fileName);	  //先刪除

	if(res!=FR_OK)
		{
			printf("\r\n f_unlink() fail .. \r\n");
			return Error;
		}else{
			printf("\r\n f_unlink() success .. \r\n");
			return No_Error;
		}
}



int seuv_write()
{
	f_mount(0, &fs);
	char fileName[]="0:/seuv.bin";

	res = f_open(&file,fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
    if(res!=FR_OK)
    	printf("SEUV  f_open fail!!\n");
	f_lseek(&file,file.fsize);
	res = f_write(&file,&seuvFrame.packettime,(int)sizeof(seuv_frame_t),&bw);
    if(res!=FR_OK)
    	printf("SEUV  f_open fail!!\n");
	if(res!=FR_OK){
		f_close(&file);
		f_mount(0, NULL);
		return Error;
	}
	else{
		f_close(&file);

		f_mount(0, NULL);
		return No_Error;
	}
}

int seuv_read(int serial,void * txbuf){   // serial =1~N

	if(serial==0) serial=1;
	f_mount(0, &fs);
	char fileName[20];
	char s[]="0:/seuv/seuv_";
	strcpy(fileName,s);
	int head = (serial-1)/100;
	char num[5];
	sprintf(num,"%d", head);
	strcat(fileName,num);
	strcat(fileName,".bin");


	res = f_open(&file,fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
	}else{
		printf("\r\n f_open() success .. \r\n");
	}
        int tmp = (serial-1) % 100;
	    f_lseek(&file,tmp*seuv_length);
		res = f_read(&file,&buffer,seuv_length,&br);

	if(res != FR_OK){
		printf("\r\n wod_read() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;}
	else{
		memcpy(txbuf,&buffer,seuv_length);
		f_close(&file);
		f_mount(0, NULL);
		return No_Error;
		}
}
int seuv_delete(){
	f_mount(0, &fs);
	char fileName[]="0:/seuv.bin";
	res = f_unlink(fileName);	  //先刪除

	if(res!=FR_OK)
		{
			printf("\r\n f_unlink() fail .. \r\n");
			return Error;
		}else{
			printf("\r\n f_unlink() success .. \r\n");
			return No_Error;
		}
}


int hk_write(uint8_t * frameCont )
{
	f_mount(0, &fs);
	char fileName[]="0:/hk.bin";

	res = f_open(&file,fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
	f_lseek(&file,file.fsize);
	res = f_write(&file,frameCont,100+sizeof(parameter_t),&bw);

	if(res!=FR_OK){
		printf("\r\n hk_write() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;
	}
	else{
		printf("\r\n hk_write() success .. \r\n");
		f_close(&file);

		f_mount(0, NULL);
		return No_Error;
	}
}

int hk_read(int serial,void * txbuf){   // serial =1~N

	if(serial==0) serial=1;
	f_mount(0, &fs);
	char fileName[20];
	char s[]="0:/hk/hk_";
	strcpy(fileName,s);
	int head = (serial-1)/100;
	char num[5];
	sprintf(num,"%d", head);
	strcat(fileName,num);
	strcat(fileName,".bin");


	res = f_open(&file,fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
	}else{
		printf("\r\n f_open() success .. \r\n");
	}
        int tmp = (serial-1) % 100;
	    f_lseek(&file,tmp*hk_length);
		res = f_read(&file,&buffer,hk_length,&br);

	if(res != FR_OK){
		printf("\r\n wod_read() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;}
	else{
		memcpy(txbuf,&buffer,hk_length);
		f_close(&file);
		f_mount(0, NULL);
		return No_Error;
		}
}

int hk_delete(){
	f_mount(0, &fs);
	char fileName[]="0:/hk.bin";
	res = f_unlink(fileName);	  //先刪除

	if(res!=FR_OK)
		{
			printf("\r\n f_unlink() fail .. \r\n");
			return Error;
		}else{
			printf("\r\n f_unlink() success .. \r\n");
			return No_Error;
		}
}


int para_r(){   // serial =1~N

	f_mount(0, &fs);
	char fileName[]="0:/para.bin";
    res = f_open(&file,fileName,FA_READ|FA_WRITE );
	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
		return Error;
	}else{
		printf("\r\n f_open() success .. \r\n");
		res = f_read(&file,&buffer,(int)sizeof(parameter_t),&br);
	}

	if(res != FR_OK){
		printf("\r\n para_read() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;}
	else{
		f_close(&file);
		f_mount(0, NULL);
		memcpy(&parameters.first_flight,&buffer,(int)sizeof(parameter_t));
		hex_dump(&buffer,(int)sizeof(parameter_t));
		return No_Error;
		}
}

int para_w(){   // serial =1~N


	f_mount(0, &fs);
	char fileName[]="0:/para.bin";
    res = f_open(&file,fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
	}else{
		printf("\r\n f_write open() success .. \r\n");
	}

	res = f_write(&file,&parameters.first_flight,(int)sizeof(parameter_t),&br);

	if(res != FR_OK){
		printf("\r\n para write() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;}
	else{
		f_close(&file);
		f_mount(0, NULL);
		return No_Error;
		}
}

int para_d(){   // serial =1~N

	f_mount(0, &fs);
	char fileName[]="0:/para.bin";
	res = f_unlink(fileName);	  //先刪除

	if(res!=FR_OK)
		{
			printf("\r\n f_unlink() fail .. \r\n");
			return Error;
		}else{
			printf("\r\n f_unlink() success .. \r\n");
			return No_Error;
		}

}


int adcs_para_r(){   // serial =1~N

	f_mount(0, &fs);
	char fileName[]="0:/adcs_para.bin";
    res = f_open(&file,fileName,FA_READ|FA_WRITE );
	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
		return Error;
	}else{
		printf("\r\n f_open() success .. \r\n");
		res = f_read(&file,&buffer,(int)sizeof(adcs_para_t),&br);
	}

	if(res != FR_OK){
		printf("\r\n wod_read() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;}
	else{
		f_close(&file);
		f_mount(0, NULL);
		memcpy(&adcs_para.strategy,&buffer,(int)sizeof(adcs_para_t));  //import
		hex_dump(&buffer,(int)sizeof(adcs_para_t));
		return No_Error;
		}
}

int adcs_para_w(){   // serial =1~N


	f_mount(0, &fs);
	char fileName[]="0:/adcs_para.bin";
    res = f_open(&file,fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
	}else{
		printf("\r\n f_write open() success .. \r\n");
	}

	res = f_write(&file,&adcs_para.strategy,(int)sizeof(adcs_para_t),&br);

	if(res != FR_OK){
		printf("\r\n para write() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;}
	else{
		f_close(&file);
		f_mount(0, NULL);
		return No_Error;
		}
}

int adcs_para_d(){   // serial =1~N

	f_mount(0, &fs);
	char fileName[]="0:/adcs_para.bin";
	res = f_unlink(fileName);	  //先刪除

	if(res!=FR_OK)
		{
			printf("\r\n f_unlink() fail .. \r\n");
			return Error;
		}else{
			printf("\r\n f_unlink() success .. \r\n");
			return No_Error;
		}

}


int inms_data_dump(){
	f_mount(0, &fs);
	char fileName[]="0:/inms.bin";
    int count=0;
	res = f_open(&file,fileName,FA_READ|FA_WRITE );
	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");

	}else
        while(1){


					res = f_read(&file,&buffer,inms_data_length,&br);

					if(res == FR_OK){
						printf("INMS Data Dump packet '%d' \r\n",count);
						hex_dump(&buffer[0],inms_data_length);
						count++;
					}
					else{
						printf("f_read() fail .. \r\n");
						break;
					}
					if(f_eof(&file)){break;}
			}



			f_close(&file);
			f_mount(0, NULL);

			return No_Error;
}


int seuv_data_dump(){
	f_mount(0, &fs);
		char fileName[]="0:/seuv.bin";
	    int count=0;
		res = f_open(&file,fileName,FA_READ|FA_WRITE );
		if(res!=FR_OK)
		{
			printf("\r\n f_open() fail .. \r\n");

		}else
	        while(1){


						res = f_read(&file,&buffer,seuv_length,&br);

						if(res == FR_OK){
							printf("INMS Data Dump packet '%d' \r\n",count);
							hex_dump(&buffer[0],seuv_length);
							count++;
						}
						else{
							printf("f_read() fail .. \r\n");
							break;
						}
						if(f_eof(&file)){break;}
				}
               memcpy(&seuvFrame.packettime,&buffer[0],seuv_length);
               seuvFrame.packettime=csp_ntoh32(seuvFrame.packettime);

               printf("SEUV seuvFrame.packettime = %u \r\n",(unsigned int)seuvFrame.packettime);
               printf("SEUV seuvFrame.samples = %u \r\n",(unsigned int)seuvFrame.samples);
               printf("CH1 AVG = %f \r\n",seuvFrame.ch1AVG);
               printf("CH1 STD = %f \r\n",seuvFrame.ch1STD);
               printf("CH2 AVG = %f \r\n",seuvFrame.ch2AVG);
               printf("CH2 STD = %f \r\n",seuvFrame.ch2STD);
               printf("CH3 AVG = %f \r\n",seuvFrame.ch3AVG);
               printf("CH3 STD = %f \r\n",seuvFrame.ch3STD);
               printf("CH4 AVG = %f \r\n",seuvFrame.ch4AVG);
               printf("CH4 STD = %f \r\n",seuvFrame.ch4STD);

				f_close(&file);
				f_mount(0, NULL);



	return No_Error;
}

int wod_data_dump(){
	f_mount(0, &fs);
		char fileName[]="0:/wod.bin";
	    int count=0;
		res = f_open(&file,fileName,FA_READ|FA_WRITE );
		if(res!=FR_OK)
		{
			printf("\r\n f_open() fail .. \r\n");

		}else
	        while(1){


						res = f_read(&file,&buffer,wod_length,&br);

						if(res == FR_OK){
							printf("INMS Data Dump packet '%d' \r\n",count);
							hex_dump(&buffer[0],wod_length);
							count++;
						}
						else{
							printf("f_read() fail .. \r\n");
							break;
						}
						if(f_eof(&file)){break;}
				}



				f_close(&file);
				f_mount(0, NULL);



	return No_Error;
}

int hk_data_dump(){
	f_mount(0, &fs);
		char fileName[]="0:/hk.bin";
	    int count=0;
		res = f_open(&file,fileName,FA_READ|FA_WRITE );
		if(res!=FR_OK)
		{
			printf("\r\n f_open() fail .. \r\n");

		}else
	        while(1){


						res = f_read(&file,&buffer,hk_length,&br);

						if(res == FR_OK){
							printf("INMS Data Dump packet '%d' \r\n",count);
							hex_dump(&buffer[0],hk_length);
							count++;
						}
						else{
							printf("f_read() fail .. \r\n");
							break;
						}
						if(f_eof(&file)){break;}
				}



				f_close(&file);
				f_mount(0, NULL);



	return No_Error;
}

