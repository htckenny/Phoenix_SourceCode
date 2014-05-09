#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <fat_sd/ff.h>
#include "fs.h"
#include <util/hexdump.h>


void vTaskinms(void * pvParameters) {
		
	    vTaskDelay(7000);
         while(1){
               vTaskDelay(60000);
		int len;
		len = inms_script_length(7);
		uint8_t script[len];
		inms_script_read(7,len,&script);
		printf("script len = %d\n",len);

		printf("hexdump from script \r\n");
		hex_dump(script,len);

		printf("-------------------------------------------\n");
		printf("Script_LEN:\n");
		printf("%02X%02X\n",script[1],script[0]);
		printf("Script_HDR:\n");
		printf("%02X%02X%02X%02X \n",script[2],script[3],script[4],script[5]);
		printf("%02X%02X%02X%02X \n",script[6],script[7],script[8],script[9]);
		printf("%02X%02X \n",script[10],script[11]);
		printf("TimeTable:\n");
		int flag =15;
		while(1){
        if(script[flag]==65)
        printf("%02d %02d %02d       S1\n",script[flag-3],script[flag-2],script[flag-1]);
        else if(script[flag]==66)
        printf("%02d %02d %02d       S2\n",script[flag-3],script[flag-2],script[flag-1]);
        else if(script[flag]==67)
        printf("%02d %02d %02d       S3\n",script[flag-3],script[flag-2],script[flag-1]);
        else if(script[flag]==68)
        printf("%02d %02d %02d       S4\n",script[flag-3],script[flag-2],script[flag-1]);
        else if(script[flag]==69)
        printf("%02d %02d %02d       S5\n",script[flag-3],script[flag-2],script[flag-1]);
        else if(script[flag]==85){
        printf("                     EOT\n");
        break;}
        flag++;
		}
		//----------------------
        int SS=1;
        flag=flag+3;
        printf("S%d\n",SS);
        int Sequence_Flag=1;
        while(1){
        if(script[flag]==241){
          if(Sequence_Flag==0){
        	  Sequence_Flag++;
        	  printf("S%d\n",SS);}
            printf("%02d %02d OBC_SU_ON  %02X %02X\n",script[flag-1],script[flag-2],script[flag+1],script[flag+2]);
            flag =flag+4+(int)script[flag+1];
            }
        else if(script[flag]==4){
            if(Sequence_Flag==0){
          	  Sequence_Flag++;
          	  printf("S%d\n",SS);}
        	printf("%02d %02d SU_STIM    %02X %02X %02X\n",script[flag-1],script[flag-2],script[flag+1],script[flag+2],script[flag+3]);
        	  flag =flag+4+(int)script[flag+1];}
        else if(script[flag]==11){
            if(Sequence_Flag==0){
          	  Sequence_Flag++;
          	  printf("S%d\n",SS);}
        	printf("%02d %02d SU_DUMP    %02X %02X\n",script[flag-1],script[flag-2],script[flag+1],script[flag+2]);
        	  flag =flag+4+(int)script[flag+1];}
        else if(script[flag]==242){
            if(Sequence_Flag==0){
          	  Sequence_Flag++;
          	  printf("S%d\n",SS);}
        	printf("%02d %02d OBC_SU_OFF %02X %02X\n",script[flag-1],script[flag-2],script[flag+1],script[flag+2]);
        	  flag =flag+4+(int)script[flag+1];}
        else if(script[flag]==254){
            if(Sequence_Flag==0){
          	  Sequence_Flag++;
          	  printf("S%d\n",SS);}
            printf("%02d %02d OBC_EOT    %02X %02X\n",script[flag-1],script[flag-2],script[flag+1],script[flag+2]);
            flag =flag+4+(int)script[flag+1];
             SS++;
             Sequence_Flag=0;
             }
        else if(script[flag]==83){
            if(Sequence_Flag==0){
          	  Sequence_Flag++;
          	  printf("S%d\n",SS);}
            printf("%02d %02d SU_HVARM   %02X %02X\n",script[flag-1],script[flag-2],script[flag+1],script[flag+2]);
            flag =flag+4+(int)script[flag+1];}
        else if(script[flag]==201){
            if(Sequence_Flag==0){
          	  Sequence_Flag++;
          	  printf("S%d\n",SS);}
            printf("%02d %02d SU_HVON    %02X %02X\n",script[flag-1],script[flag-2],script[flag+1],script[flag+2]);
            flag =flag+4+(int)script[flag+1];}
        else if(script[flag]==8){
            if(Sequence_Flag==0){
          	  Sequence_Flag++;
          	  printf("S%d\n",SS);}
            printf("%02d %02d SU_SCI     %02X %02X %02X%02X%02X%02X%02X\n",script[flag-1],script[flag-2],script[flag+1]
             ,script[flag+2],script[flag+3],script[flag+4],script[flag+5],script[flag+6],script[flag+7]);
            flag =flag+4+(int)script[flag+1];}
        else break;
		}
		printf("XSUM:\n");
        printf("%02X %02X\n",script[flag-2],script[flag-1]);
}}
