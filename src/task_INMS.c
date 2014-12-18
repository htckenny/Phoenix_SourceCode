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

#include <dev/usart.h>
#include <io/nanopower2.h>
#include <io/nanomind.h>
#include <csp/csp.h>
#include <csp/csp_endian.h>
#include "parameter.h"

#define scriptNum 7
#define isSimulator 1
typedef struct __attribute__((packed)) {
	int tt_hour, tt_min, tt_sec, tt_seq;
} timetable;
int rec[scriptNum];
/*initializing for identifying which script is running*/
int scriptRunning ;
/*for the detection of the OBC_SU_ERRr*/
int obcSuErrFlag = 0;
int maxlength = 0;

void bubblesort(uint32_t refsec[]){
	int i,c,d;
	int j = 0;
	uint32_t swap;
	uint32_t aftsec[scriptNum];
	for(i=0;i<scriptNum;i++){
		rec[i]=0;
		aftsec[i]=refsec[i];
	}
	//	for(c=0;c<(scriptNum-1);c++){
	//		for(d=0;d< (scriptNum-c-1);d++){
	//			if(refsec[d] > refsec[d+1]){
	//				swap = refsec[d];
	//				refsec[d] = refsec[d+1];
	//				refsec[d+1] = swap;
	//			}
	//		}
	//	}
	for(c=0;c<(scriptNum-1);c++){
		for(d=0;d< (scriptNum-c-1);d++){
			if(aftsec[d] > aftsec[d+1]){
				swap = aftsec[d];
				aftsec[d] = aftsec[d+1];
				aftsec[d+1] = swap;
			}
		}
	}
	for(i=0;i<scriptNum;i++){
		for(j=0;j<scriptNum;j++){
			if(refsec[i]==aftsec[j]){
				rec[j]=i;
			}
		}
	}
}
void inmssort(uint32_t *sec){

	uint32_t refsec[scriptNum] ;
	timestamp_t t;
	t.tv_sec = 0;
	t.tv_nsec = 0;
	obc_timesync(&t, 6000);
	//time_t tt = t.tv_sec;
	//printf("t.tv_sec = %d\n",t.tv_sec);


	for (int i =0;i<scriptNum;i++){
		refsec[i]=0;
	}
	printf("t.tv_sec = %d\n",t.tv_sec);
	for(int i = 0;i<scriptNum;i++){
		refsec[i] = (sec[i]-t.tv_sec);
		printf("refsec[%d] = %" PRIu32 "\n",i,refsec[i]);
	}
	bubblesort(refsec);
}
/*FLETCHER-16  checksum based on REQ: INMS-I-127*/
uint16_t fletcher(uint8_t script[],int length){

	uint16_t C0_int= 0,C1_int=0;
	uint16_t XSUM_W = 65535;//0xFFFF
	for(int i =0;i< length;i++){
		C0_int = C0_int+script[i];
		C1_int = C1_int+C0_int;
		C0_int = (C0_int) % (255);
		C1_int = (C1_int) % (255);
	}
	XSUM_W = (C0_int)||(C1_int>>8);
	return XSUM_W;
}
/*Return the system time with big clock(Real seconds) or small clock(86400)*/
uint32_t timeGet (int clockType){
	uint32_t onBoardTime =0;
	timestamp_t t;
	t.tv_sec = 0;
	t.tv_nsec = 0;
	obc_timesync(&t, 6000);
	//	if (size == 0 ){
	//		//big clock
	//		onBoardTime = t.tv_sec;
	//	}
	//	else if (size== 1){
	//		//small clock
	//		onBoardTime = t.tv_sec % 86400;
	//	}
	onBoardTime = (clockType == 0) ?( t.tv_sec) :( t.tv_sec % 86400);
	return onBoardTime;
}
void vTaskInmsErrorHandle(void * pvParameters){
	uint8_t rsp_err_code=0;
	uint8_t seq_cnt=0;
	uint8_t obcerrpacket[174];
	while(1){
		switch(obcSuErrFlag)  {
			case 0:		//No error
				rsp_err_code = 0x00;
				break;
			case 1:		//INMS packet time-out error
				rsp_err_code = 0xf0;
				break;
			case 2:		//INMS packet length error detected
				rsp_err_code = 0xf1;
				break;
			case 3:		//INMS emergency turn OFF executed
				rsp_err_code = 0xf2;
				break;
			case 4:		//INMS over-current condition detected
				rsp_err_code = 0xf3;
				break;
			case 5:		//INMS script command error detected
				rsp_err_code = 0xf4;
				break;
				//others to be defined by PHOENIX team
				//No external error being defined by now
			default:
				break;
		}
		extern uint8_t script[scriptNum][maxlength];
		extern int len[scriptNum];
		for (int i =0;i<scriptNum;i++){
		    	len[i]=inms_script_length(i);
		 }
		for(int i=0;i<scriptNum;i++){
					inms_script_read(i,len[i],&script[i]);
		}
		obcerrpacket[0] = 0xfa;
		obcerrpacket[1] = seq_cnt; //not sure what is it
//		scriptRunning
		obcerrpacket[2] = rsp_err_code;
		obcerrpacket[3] = script[scriptRunning][len[scriptRunning]-2];
		obcerrpacket[4] = script[scriptRunning][len[scriptRunning]-1];
		for(int i=5;i<5+10;i++){
			obcerrpacket[5] = script[scriptRunning][i-3];
		}
		int jflag = 15;
		int sflag;
		for (int i=0;i<7;i++){
			obcerrpacket[jflag] = script[i][len[i]-2];
			obcerrpacket[jflag+1] = script[i][len[i]-1];
			sflag=2;
			for(int j=jflag+2;j<jflag+12;j++,sflag++){
				obcerrpacket[j] = script[i][sflag];
			}
			jflag = jflag+12;
		}
		 inms_data_write(obcerrpacket);
	}
}
void vTaskInmsReceive(void * pvParameters){
	int numReceive=0;
	char ucharAdcs[22];
	char ucharTotal[174+22];		//response packet is FIXED 174 BYTES+ADCS 22 BYTES
	char uchar[174];
//	int num=0;
//	char snum[10];
	for(int k=0;k<sizeof(ucharAdcs);k++){
		ucharAdcs[k]=0;
	}
	for(int k=0;k<sizeof(ucharTotal);k++){
		ucharTotal[k]=0;
	}
	while(1){
		/*response_pkt*/
		vTaskDelay(1000);
		numReceive = usart_messages_waiting(2);
//		if (numReceive!=0){
//			printf("numReceive = %d\n",numReceive);
//		}
		int ADCSexist = 0;
		if ( numReceive  != 0){
			printf("Get %d response packet!\n",numReceive);
			if (ADCSexist==1){
				if(i2c_master_transaction(0,3/*ADCS address*/, 22,1,&ucharAdcs,22,2) == E_NO_ERR){
					printf("Get Time, Attitude, Position from ADCS");
				}
				else{
					printf("Get data from ADCS FAILED \r\n");
				}
				for (int i=0;i<22;i++){
					ucharTotal[i]=ucharAdcs[i];
				}
				for (int i=22;i<196;i++){
					ucharTotal[i] = usart_getc(2);
	//				ucharTotal[i] = usart_getc(2);
				}
			}
			else{
				for(int i=0;i<174;i++){
					uchar[i] = usart_getc(2);
				}
			}
			hex_dump(uchar,numReceive);
			// inms_data_write(ucharTotal);
			 numReceive=0;
		/* ---------------------    */
		}
	}
}
void vTaskinms(void * pvParameters) {
	vTaskDelay(7000);
	while(1){
		//	vTaskDelay(60000);

		int len[scriptNum];											//the length of each script
		uint32_t epoch_sec[scriptNum];	             	//the seconds from UTC epoch time 2000/01/01 00:00:00 Am
		for (int i=0;i<scriptNum;i++){
			epoch_sec[i]=0;
		}
		printf("STEP #1 : Getting length of all scripts.................\n");
		/* STEP #1 Get the length of all of the scripts*/
		for(int i=0;i<scriptNum;i++){
			printf("script %d: ",i);
			len[i]=inms_script_length(i);
			if(len[i]>=maxlength){
				maxlength = len[i];
			}
		}
		printf("MAX length : %d\n",maxlength);
		uint8_t script[scriptNum][maxlength];

		printf("STEP #2a : Performing Fletcher 16........................\n");
		/* STEP #2 Read the script, put the data into script[] and perform Fletcher-16 checksum and additional check*/
		for(int i=0;i<scriptNum;i++){
			inms_script_read(i,len[i],&script[i]);
			uint16_t xsum = fletcher(script[i],len[i]);
			if(xsum==0){
				printf("No. %d script XSUM through Fletcher-16 [PASS]\n",i);
			}
			else{
				printf("No. %d script XSUM through Fletcher-16 [FAIL]\n",i);
				//should generate error report ----------------------------------------------------------------------------------------------------
			}
		}
		printf("STEP #2b: Performing additional check with script length........................\n");
		for(int i=0;i<scriptNum;i++){
			if(len[i]==script[i][0]){
				printf("No. %d script length check [PASS]\n",i);
			}
			else{
				printf("No. %d script length check [FAIL]\n",i);
			}
		}

		printf("STEP #3 : Converting 4 bytes times into seconds........................\n");
		/*STEP #3 Convert the 4 byte time stamps into seconds since UTC epoch ;*/
		for(int i = 0;i<scriptNum;i++){
			epoch_sec[i] = (script[i][2]) + (script[i][3]<<8) + ( script[i][4]<<16) + (script[i][5]<<24);
//			printf("%d = %" PRIu32 "\n", i, epoch_sec[i]);
		}

		/*Sort these 7 epoch times by calling the function inmssort*/
		inmssort(&epoch_sec);
		for(int i=0;i<scriptNum;i++){
			printf("[%d] => %d\n",i,rec[i]);
		}


		vTaskDelay(2000);

		/* for temporary setting,  now only have idle0, idle1*/
		if(isSimulator){
			rec[0]=1;
			rec[1]=0;
		}
		/*                                                                                                 */
		printf("STEP #4 : Recording TimeTable with structure.......................\n");
		/*STEP #4 Record the TimeTable with structure*/
		for (int i=0;i<scriptNum;i++){
			/*initialize*/
			int flag = 0;		//record which byte is running now
			int ttflag=0;		//time-table flag
			double p;
			timetable timetable_t[20]; 		//(option) Use pointer
			flag = 15;
			/*Mark an IDLE-SLOT script buffer as the RUNNING-SCRIPT*/
			scriptRunning =  rec[i];	//for the need to jump to the next script

			while(script[rec[i]][flag]!=0x55){		//0x55 = EOT
				printf(" cmd= %d\n", script[rec[i]][flag]);
				if (script[rec[i]][flag] == 0x41)	{ 									//S1
					timetable_t[ttflag].tt_hour	=  script[rec[i]][flag-1];
					timetable_t[ttflag].tt_min	=  script[rec[i]][flag-2];
					timetable_t[ttflag].tt_sec	=  script[rec[i]][flag-3];
					timetable_t[ttflag].tt_seq	= 1;
					printf("Sequence 1 ..\n");
				}
				else if (script[rec[i]][flag] == 0x42){									//S2
					timetable_t[ttflag].tt_hour	=  script[rec[i]][flag-1];
					timetable_t[ttflag].tt_min	=  script[rec[i]][flag-2];
					timetable_t[ttflag].tt_sec	=  script[rec[i]][flag-3];
					timetable_t[ttflag].tt_seq	= 2;
					printf("Sequence 2 ..\n");
				}
				else if (script[rec[i]][flag] == 0x43){									//S3
					timetable_t[ttflag].tt_hour	=  script[rec[i]][flag-1];
					timetable_t[ttflag].tt_min	=  script[rec[i]][flag-2];
					timetable_t[ttflag].tt_sec	=  script[rec[i]][flag-3];
					timetable_t[ttflag].tt_seq	= 3;
					printf("Sequence 3 ..\n");
				}
				else if (script[rec[i]][flag] == 0x44){									//S4
					timetable_t[ttflag].tt_hour	=  script[rec[i]][flag-1];
					timetable_t[ttflag].tt_min	=  script[rec[i]][flag-2];
					timetable_t[ttflag].tt_sec	=  script[rec[i]][flag-3];
					timetable_t[ttflag].tt_seq	= 4;
					printf("Sequence 4 ..\n");
				}
				else if (script[rec[i]][flag] == 0x45){									//S5
					timetable_t[ttflag].tt_hour	=  script[rec[i]][flag-1];
					timetable_t[ttflag].tt_min	=  script[rec[i]][flag-2];
					timetable_t[ttflag].tt_sec	=  script[rec[i]][flag-3];
					timetable_t[ttflag].tt_seq	= 5;
					printf("Sequence 5 ..\n");
				}
				else{
					ttflag--;
				}
				printf("hour:%d, min:%d, sec:%d, seq:%d\n",timetable_t[ttflag].tt_hour,timetable_t[ttflag].tt_min,timetable_t[ttflag].tt_sec,timetable_t[ttflag].tt_seq);
				flag ++;
				ttflag++;
				printf("flag = %d, ttflag = %d\n", flag,ttflag);
			}
			flag++;
			ttflag--;
			int ttflagMax = ttflag;
			printf("STEP #5 : Checking the correctness of the sequence...................\n");

			/*Check if the first sequence is S1 [Req-INMS-I-228]*/
			if (timetable_t[0].tt_seq ==1){
				printf("The first sequence is S1!!\n");
			}
			else{
				printf("The first sequence is not S1, ERROR! \n");
				//should return error code .
			}
			/*Check if the Script sequence slots be used sequentially [Req-INMS-I-229]
			  need more scenario to test*/
			int last_seq =timetable_t[0].tt_seq;
			for(int i=1;i<ttflag;i++){
				if(timetable_t[i].tt_seq-last_seq < 2){
					last_seq = timetable_t[i].tt_seq;
				}
				else{
					printf("The sequence slots are not used sequentially, ERROR! \n");
					//should return error code .
				}
			}
			printf("STEP #6 :Setting End of time flag ...................\n");
			int eotflag [20];  //need prove
			int eotnum=0;
			int nnflag=flag;
			for(int i =0;i<20;i++){
				eotflag[i]=0;
			}
			//find where is OBC_EOT 0xFE
			eotflag[eotnum++]=nnflag;
			while(nnflag <= script[rec[i]][0]){
				if(script[rec[i]][nnflag] == 254){
					eotflag[eotnum] = nnflag+3;
					eotnum++;
				}
				nnflag ++;
			}
			for(int i=0;i<eotnum;i++){
				printf("EOTflag[%d] =%d\n",i,eotflag[i]);
			}

			printf("STEP #7 : Executing command...................\n");
			/*Start execute the command by following the  timetable*/
			uint32_t tTable_24 = 0;
			uint32_t refTime = 0;
			ttflag = 0;
			printf("%d",timetable_t[ttflag].tt_seq);
			int printTime=1;
			int seqcount=0;
			while(1){
				printf("virtual clock = %d\n",virtual_clock);  //-----------------------------------------------------need  test--------------------------------------------//
				if(seqcount > ttflagMax){
					break;
				}
				vTaskDelay(1000);
				refTime = timeGet(1);  //get the small clock time
				/*reduce the time for easy debug*/
//				if (scriptRunning ==0){
//					refTime = 58200+timeGet(1);
//				}
//				else if(scriptRunning==1){
//					refTime = 290+timeGet(1);
//				}

				/*                                                       */
				tTable_24  = timetable_t[ttflag].tt_hour*3600 + timetable_t[ttflag].tt_min*60 +  timetable_t[ttflag].tt_sec;
				if (printTime==1){
					printf("%" PRIu32 " -- %" PRIu32 "\n",refTime,tTable_24);
					printf("\E[2A\r");
				}
				if(refTime == tTable_24){
					 printTime=0;
					if(timetable_t[ttflag].tt_seq == 1){
						flag =eotflag[0];
						printf("\n\n\n putting flag 1 done\n");
					}
					else if (timetable_t[ttflag].tt_seq == 2){
						flag =eotflag[1];
						printf("\n\n\n putting flag 2 done\n");
					}
					else if (timetable_t[ttflag].tt_seq == 3){
						flag =eotflag[2];
						printf("\n\n\n putting flag 3 done\n");
						printf("flag = %d\n",flag);
					}
					else if (timetable_t[ttflag].tt_seq == 4){
						flag =eotflag[3];
						printf("\n\n\n putting flag 4 done\n");
					}
					else if (timetable_t[ttflag].tt_seq == 5){
						flag =eotflag[4];
						printf("\n\n\n putting flag 5 done\n");
					}
					printf("putting flag done1\n");
					int leng;
					int delayTimeNow=0 ;
					int delayTimeTarget=0;
					int numReceive = 0;
					int tempTime=0;
//					char uchar[174];			//response packet is FIXED 174 BYTES
					printf("flag = %d, total = %d\n",flag,script[rec[i]][0]);
					while(flag<=script[rec[i]][0]){
						printf("Into the sequence  loop : Sending command....\n");
						leng = script[rec[i]][flag+3]; //get Command's length
						printf("Flag = %d\n",flag);
						printf("Content = %d\n",script[rec[i]][flag+2]);
//						if(script[rec[i]][flag+1]*60+script[rec[i]][flag]==1110){
//							printf("Transfer 1110 to 20s");
//							vTaskDelay(1000*20);    //delay for testing SU_DUMP
//						}
//						else{
//							vTaskDelay(1000*(script[rec[i]][flag+1]*60+script[rec[i]][flag]));    //delay min+sec
//						}
						printf("delay: %d\n",script[rec[i]][flag+1]*60+script[rec[i]][flag]);

						if(script[rec[i]][flag+2]==241){   																 //OBC_SU_ON  = 241
							//command EPS to open INMS , still need pin definition to be confirm!!
							/*----For simulator----*/
							if (isSimulator){
								for (int j=2;j<=leng+3;j++){
									usart_putstr(2,&script[rec[i]][flag+j],1);
								}
							}
							 /*----------------------------*/
							printf("COMMAND: OBC_SU_ON...........\n");
						}
						else if (script[rec[i]][flag+2]==242){ 														//OBC_SU_OFF = 242
							//command EPS to open INMS , still need pin definition to be confirm!!
							/*----For simulator----*/
							if (isSimulator){
								for (int j=2;j<=leng+3;j++){
									usart_putstr(2,&script[rec[i]][flag+j],1);
								}
							}
							 /*----------------------------*/
							printf("COMMAND: OBC_SU_OFF...........\n");
						}
						else if(script[rec[i]][flag+2]==254){ 															//0xFE = OBC_EOT  = 254 ------------------------------------->check 0xFD or 0xFE
							printf("COMMAND: OBC_EOT...........\n");
							//eotCount++;
							/*----For simulator----*/
							if (isSimulator){
								for (int j=2;j<=leng+3;j++){
									usart_putstr(2,&script[rec[i]][flag+j],1);
								}
							}
							 /*----------------------------*/
							ttflag++;
							seqcount++;
							printTime=1;
							/*delay*/
							delayTimeNow=refTime;
							delayTimeTarget = delayTimeNow +script[rec[i]][flag+1]*60+script[rec[i]][flag];
							tempTime = delayTimeNow;
							while(delayTimeTarget != delayTimeNow){
								printf("%d-----------%d\n",delayTimeNow-tempTime,delayTimeTarget-tempTime);
								printf("\E[1A\r");
								delayTimeNow = delayTimeNow+1;
								vTaskDelay(1000);
							}
							/*delay*/
							break;
						}
						else{
							for (int j=2;j<=leng+3;j++){
								usart_putstr(2,&script[rec[i]][flag+j],1);
								printf("COMMAND : %02x \n",script[rec[i]][flag+j]);
							}

							vTaskDelay(200);
						}
						/*delay*/
						delayTimeNow=refTime;
						if(script[rec[i]][flag+1]*60+script[rec[i]][flag]==1110){
							delayTimeTarget = delayTimeNow+66;
						}
						else{
							delayTimeTarget = delayTimeNow +script[rec[i]][flag+1]*60+script[rec[i]][flag];
						}
						tempTime = delayTimeNow;
						while(delayTimeTarget != delayTimeNow){
							printf("%d-----------%d\n",delayTimeNow-tempTime,delayTimeTarget-tempTime);
							printf("\E[1A\r");
							delayTimeNow = delayTimeNow+1;
							vTaskDelay(1000);
						}
						/*delay*/
						flag = flag +leng+4;
					}
				}
			}
		}
	}
}
