/*
 * FS.h
 *
 *  Created on: 	2014/11/20
 *  Last Update: 	2015/09/30
 *  Author: 		rusei, Kenny
 *
 */
#ifndef FS_H_
#define FS_H_
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

int inms_script_write(int buffNum,uint8_t scriptCont[], int delete_flag, int length);
void inms_script_read(int buffNum,int packlength,void * txbuf);
int inms_script_length(int buffNum);
/*---------------------------------------------------*/
/*             		  INMS Data                      */
int inms_data_write(uint8_t frameCont[]);
int inms_data_read(char fileName[], void * txbuf);      					 // serial= which packet you want to read , Range 1~N
int inms_data_delete();                                                       //return 0 when success, return -1 when fail.
/*---------------------------------------------------*/
/*                Whole Orbit Data                   */
int wod_write(uint8_t frameCont[]);
int wod_read(char fileName[], void * txbuf);
int wod_delete();
/*---------------------------------------------------*/
/*                 Solar EUV Data                    */
int seuv_write();
int seuv_read(char fileName[],void * txbuf);
int seuv_delete();
/*---------------------------------------------------*/
/*              House Keeping Data                   */
int hk_write(uint8_t frameCont[]);
int hk_read(char fileName[],void * txbuf);
int hk_delete();
/*---------------------------------------------------*/
/*            Early Orbit Phase Data                 */
int eop_write(uint8_t frameCont[]);
int eop_read(char fileName[],void * txbuf);
int eop_delete();
/*---------------------------------------------------*/
int para_w();
int para_r();
int para_d();
/*---------------------------------------------------*/
int downlink_data_before_t(uint8_t datatype,uint32_t time1);
int downlink_data_after_t(uint8_t datatype,uint16_t time1);
int downlink_data_between_t(uint8_t datatype,uint16_t time1,uint16_t time2);

int delete_data_before_t(uint8_t datatype,uint32_t time1);
int delete_data_after_t(uint8_t datatype,uint16_t time1);
int delete_data_between_t(uint8_t datatype,uint16_t time1,uint16_t time2);
/*---------------------------------------------------*/
int inms_data_dump();
int wod_data_dump();
int seuv_data_dump();
int hk_data_dump();
int thermal_data_dump();

/*---------------------------------------------------*/
int T_data_d();
int thurmal_2_w();
int thurmal_1_w();
/*---------------------------------------------------*/
int schedule_write(uint8_t * frameCont);
int schedule_read(uint8_t * txbuf);
int schedule_reset();
int schedule_delete(int range, uint8_t * frameCont);
int schedule_shift(uint8_t *frameCont);
int schedule_dump();
/*---------------------------------------------------*/

int scan_files_Downlink (char* path, int mode, uint32_t timeRec_T1, uint32_t timeRec_T2) ;   	
int scan_files_Delete (char* path, int mode, uint32_t timeRec_T1, uint32_t timeRec_T2) ;   	

#endif /* FS_H_ */
