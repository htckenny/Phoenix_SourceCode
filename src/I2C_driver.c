//#include <stdlib.h>
//#include <string.h>
//#include <stdint.h>
//#include <inttypes.h>
#include <dev/cpu.h>
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
#include <util/timestamp.h>
#include <util/vermagic.h>
//#include <time.h>

#define E_NO_ERR -1

int delete_buf(){

	 unsigned int reg=36;  // delete a frame in receive buffer   // no resp
	  uint8_t txdata[1];
	  txdata[0] = reg;

	  if ( i2c_master_transaction(0,80, txdata,1,0,0,5) == E_NO_ERR) {
		  printf(" Delete a frame in receive buffer \r\n");
		  return 1;
	  }
	  else{

		  printf(" Delete  frame fail \r\n");
		  return -1;
	  }

}

int arm_ant(){

	  uint8_t txdata[1];
	  txdata[0] = 173;    // arm 0x49 ant board

	  if ( i2c_master_transaction(0,49,txdata,1,0,0,5) == E_NO_ERR) {
	        return 1;
	  }
	  else
	  return 0;
}
int callsign(){
	uint8_t txdata[8]; // TO buffer size
				  txdata[0] = 34;
				  txdata[1] = 71;//GS-NCKU
				  txdata[2] = 83;
				 	 	  txdata[3] = 45;
				 	 	  txdata[4] = 78;
				 	 	  txdata[5] = 67;
				 	 	  txdata[6] = 75;
				 	 	  txdata[7] = 85;
    uint8_t txdata2[8]; // FROM buffer size
                  txdata2[0] = 35;
				  txdata2[1] = 80;//PHOENIX
				 	  txdata2[2] = 72;
				 	  txdata2[3] = 79;
				 	  txdata2[4] = 69;
				 	  txdata2[5] = 78;
				 	  txdata2[6] = 73;
				 	  txdata2[7] = 88;

	  if ( i2c_master_transaction(0,81, txdata,8,0,0,5) == E_NO_ERR) {
		  if ( i2c_master_transaction(0,81, txdata2,8,0,0,5) == E_NO_ERR)
	        return 1;
		  else
			  return 0;
	  }
	  else
	  return 0;


}
int deploy_ant(){

		unsigned int para=255;  // max 255 sec
	    unsigned int reg=165;     // deploy ant board one by one


	  uint8_t txdata[2];
	  txdata[0] = reg;
	  txdata[1] = para;
	  if ( i2c_master_transaction(0,49, txdata,2,0,0,5) == E_NO_ERR) {
	        return 1;
	  }
	  else
	  return 0;
}
int tx_mode(int mode){

		unsigned int para=mode;  // nominal telemetry mode
	    unsigned int reg=37;     //Set transmitter output mode
     if(para==1)
    	 printf(" Set TX to nominal telemetry mode \r\n");
     else if(para==0)
    	 printf(" Set TX to external telemetry mode \r\n");
     else if(para==3)
    	 printf(" Set TX to receive loopback mode \r\n");

	  uint8_t txdata[2];
	  txdata[0] = reg;
	  txdata[1] = para;
	  if ( i2c_master_transaction(0,81,txdata,2,0,0,5) == E_NO_ERR) {
	        return 1;
	  }
	  else
	  return 0;
}
int CIC(){   //check incoming command

	 unsigned int reg=33;
	  uint8_t txdata[1];
	  txdata[0] = reg;

	  int rx_length = 1;
     uint8_t val[rx_length];  // have 1 byte response

	  if ( i2c_master_transaction(0,80, txdata,1,&val,rx_length,10) == E_NO_ERR) {
	        return (unsigned int)val[0];
	  }
	  else
	  return -1;
}
int down_data_read(int day,int type,int ssn,int packlength,void * txbuf){
	  printf("wait for kenny\n" );
}
int return_error(){
	uint8_t txdata[238];
	txdata[0]=16;
				  for(int a=1;a<6;a++)
					  txdata[a]=255;

			    if ( i2c_master_transaction(0,81,txdata,6,0,0,5) == E_NO_ERR) {
			    				  return 1;}
			    else  return 0;

}

int rccs(){ //Request COM Current State
      	  uint8_t txdata[238];
		  txdata[0] = 26;

		 int rx_length = 16;
	     uint8_t val[rx_length];  // have 16 byte response

		  if ( i2c_master_transaction(0,80, txdata,1,&val,rx_length,10) == E_NO_ERR) {
			  txdata[0]=16;
			  for(int a=1;a<(rx_length+1);a++)
				  txdata[a]=val[a-1];

			      vTaskDelay(100);
		    if ( i2c_master_transaction(0,81,txdata,17,0,0,5) == E_NO_ERR) {
		    				  return 1;}
			  else{
				printf("have error on send rccs \n");
				  return -1;
				  }
		  }
		  else{
			  printf("have error on rccs \n");
		  return -1;
		  }
}

int recs(){  //Request EPS Current State
      	  uint8_t txdata[238];
		  txdata[0] = 8;

		 int rx_length = 43;
	     uint8_t val[rx_length];  // have 16 byte response

		  if ( i2c_master_transaction(0,2, txdata,1,&val,rx_length,10) == E_NO_ERR) {
			  txdata[0]=16;


			  for(int a=1;a<(rx_length+1);a++)
			 				  txdata[a]=val[a-1];
						      vTaskDelay(100);

		    if ( i2c_master_transaction(0,81,txdata,44,0,0,5) == E_NO_ERR) {
		    				  return 1;}
			  else{
				printf("have error on send recs \n");
				  return -1;
				  }
		  }
		  else{
			  printf("have error on recs \n");
		  return -1;
		  }
}
int execute(char * txbuf){   // send back received command to GS
	uint8_t txdata[238];
	uint8_t dframe[101];
	int count;
	int datatype;
	int ssn;
    int x=txbuf[1];
    uint8_t time;
	//---------------------------------------------------------//
	switch(x){
	case 0x11: //downlink 1 sector of data (100 frames)
		printf("Command Type: 0x11  downlink  1 sector of data (100 frames) \r\n");
		 datatype=txbuf[4];///////////////////////////////
		 ssn=txbuf[4]-(32*datatype);/////////////////////////
	printf("Data Type: %d Day: %d  Serial number: %d \r\n",datatype,txbuf[3],ssn);
		for(int a=ssn*100;a<(ssn*100)+100;a++){
      if(down_data_read(txbuf[3],datatype,a,101,&dframe)==1){
    	  memcpy(txdata[1],&dframe,101);
    	  txdata[0]=16;
    	  if ( i2c_master_transaction(0,81,txdata,102,0,0,5) != E_NO_ERR) {
    				  return 1;}
    	  else
    				  return 0;
      }
      else{
    	   printf("Can't get data frame packet error\n" );
           return 0;}
		}
	case 0x12:  //Multiple Sector retrieve
	case 0x13:  //Target frame retrieve
	case 0x21:     //test downlink a-->z
		printf("Command Type: 0x21  downlink a-->z \r\n");
			  txdata[0] = 16;
                 count =1;
              for(int n=97;n<123;n++){
            	  txdata[count]=n;
            	  count++;
              }
			  if ( i2c_master_transaction(0,81, txdata,27,0,0,5) == E_NO_ERR) {
			        return 1;
			  }
			  else
			  return 0;

	case 0x22:  //test downlink 0~236
			   // TX buffer size
		printf("Command Type: 0x21  downlink 0~236 \r\n");
			  txdata[0] = 16;

			 	count =0;
		     for(int n=1;n<238;n++){
			  txdata[n]=count;
			 	count++;
			 	               }


			  if ( i2c_master_transaction(0,81, txdata,236,0,0,5) == E_NO_ERR) {
			        return 1;
			  }
			  else
			  return 0;
	case 0x23: //get time
		printf("Command Type: 0x23 show on-board time \r\n");
		txdata[0] = 16;
		timestamp_t t;
		t.tv_sec = 0;
		t.tv_nsec = 0;
		obc_timesync(&t, 6000);
		time_t tt = t.tv_sec;
		time = t.tv_sec;
		printf("OBC time is: %d\r\n",time);
		txdata[1] = time;



		if ( i2c_master_transaction(0,81,txdata,2,0,0,5) == E_NO_ERR) {
					        return 1;
					  }
					  else
					  return 0;
	case 0x31:  //upload INMS script
	case 0x32:   // upload SEUV scipt
	case 0x33:   // SEUV_take data
	case 0x34:   // SEUV_take_data_1ch
	case 0x35:   // SEUV_ON
	case 0x36:   // SEUV_OFF
	case 0x51://Request ADCS Current State
	case 0x52://Request COM Current State
		printf("Command Type: 0x52  Request COM Current State 0~236 \r\n");
		if(rccs()==1)
		return 1;
		else
		return 0;
	case 0x53://Request EPS Current State
		printf("Command Type: 0x53  Request EPS Current State 0~236 \r\n");
				if(recs()==1)
				return 1;
				else
				return 0;

	case 0x54://Request WOD
	case 0x55://Request PHOENIX HK
	case 0x56://Request on board time
	case 0x57://Request operation system current state
	case 0x58://Request system parameter

    default:
              printf("Invalid command\n" );
              return_error();
    return 0;
}}

int send_back(void * txbuf){   // send back received command to GS



	  uint8_t txdata[18];
	  txdata[0] = 16;
      memcpy(txdata[1],txbuf,17);

	  if ( i2c_master_transaction(0,81, txdata,18,0,0,2) == E_NO_ERR) {
	        return 1;
	  }
	  else
	  return 0;
}

int send_test(){   // send back received command to GS



	 unsigned int reg=17;
	  uint8_t txdata[240];
	  txdata[0] = reg;
	  txdata[1] = 71;//GS-NCKU
	 	 	 	  txdata[2] = 83;
	 	 	 	  txdata[3] = 45;
	 	 	 	  txdata[4] = 78;
	 	 	 	  txdata[5] = 67;
	 	 	 	  txdata[6] = 75;
	 	 	 	  txdata[7] = 85;
	 	  txdata[8] = 80;//PHOENIX
	 	 	  txdata[9] = 72;
	 	 	  txdata[10] = 79;
	 	 	  txdata[11] = 69;
	 	 	  txdata[12] = 78;
	 	 	  txdata[13] = 73;
	 	 	  txdata[14] = 88;
      for(int a =15;a<240;a++)
    	  txdata[a] = a;

	  if ( i2c_master_transaction(0,81, txdata,240,0,0,2) == E_NO_ERR) {
	        return 1;
	  }
	  else
	  return 0;
}

int RIC(){   // read incomming command

	 unsigned int reg=34;
	  uint8_t txdata[1];
	  txdata[0] = reg;
	  int rx_length = 17;
	  uint8_t val[rx_length];  // expect  1+16 byte response , first byte is length1~16

	  if ( i2c_master_transaction(0,80, txdata,1,&val,rx_length,10) == E_NO_ERR) {
		    hex_dump(val,15);     // get command   print in

	/*   if( send_back(val)==1){  // sned it back
			   printf(" Replyed to GS done \r\n");
			   vTaskDelay(1000);
			   delete_buf();   // delete one frame in receive buffer
		   }else
			   printf(" Replyed to GS fail \r\n");
    */
		  //  vTaskDelay(100);
		    if( execute(val)==1){  // Execute the command
		   			   delete_buf();   // delete one frame in receive buffer
		   			   return 1;
		    }else{
		      printf(" execute fail \r\n");}
		    delete_buf();
	  }
	  else
	  return -1;
}

void vTaskI2C(void * pvParameters) {

		vTaskDelay(5000);
	    while(1){
	 if(arm_ant()==1){
//	printf("Arm ant success \r\n");
	break;}
	 else{
		 printf("Arm ant fail \r\n");
	    }
	             }

	    while(1){
	   	 if(deploy_ant()==1){
	   	break;}
	   	 else{
	   		 printf("Deploy ant fail \r\n");
	   	    }}

	    while(1){
	 if(tx_mode(1)==1){
	break;}
	 else{
		 printf("tx_mode set fail \r\n");}}

	    while(1){
		 if(callsign()==1){
		break;}
		 else{
			 printf("tx_mode set fail \r\n");}}


	/*  vTaskDelay(5000);     //*************** Just send it back
	    while(1){
	    if( send_test()==1){
	     printf(" Send a frame to GS \r\n");
	    	 vTaskDelay(5000);
	     }else{
	    	printf(" Send test fail \r\n");}}

*/
		 printf("Start to checking imcoming command \r\n");
		 printf("GS can start to send uplink command \r\n");
		 int flag;
	    while(1){
	    	flag = CIC();
	    	if( flag>10 || flag<0 )
	    		flag = CIC();


	    	if(flag>0){
	    	  printf("Find %d frame in receive buffer \r\n",flag);
               RIC();
               }

	    	 vTaskDelay(500);
	    	}
	    }





