/*
 * FS.h
 *
 *  Created on: 	2014/11/20
 *  Last Update: 	2016/01/10
 *  Author: 		rusei, Kenny
 *
 */
#ifndef FS_H_
#define FS_H_
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
/*---------------------------------------------------*/
/*             		  INMS Script                    */
int inms_script_write_flash(int buffNum, uint8_t scriptCont[], int delete_flag, int length);
int inms_script_read_flash(int buffNum, int packlength, void * txbuf);
int inms_script_length_flash(int buffNum);
int inms_script_delete_flash(int buffNum);
/*---------------------------------------------------*/
/*             		  INMS Data                      */
void inms_data_write_dup(uint8_t frameCont[]);
int inms_data_write(uint8_t frameCont[], int SD_partition);
int inms_data_read(char fileName[], void * txbuf);      					 // serial= which packet you want to read , Range 1~N
int inms_data_delete();                                                       //return 0 when success, return -1 when fail.
/*---------------------------------------------------*/
/*                Whole Orbit Data                   */
void wod_write_dup(uint8_t frameCont[]);
int wod_write(uint8_t frameCont[], int SD_partition);
int wod_read(char fileName[], void * txbuf);
int wod_delete();
/*---------------------------------------------------*/
/*                 Solar EUV Data                    */
void seuv_write_dup();
int seuv_write(int SD_partition);
int seuv_read(char fileName[], void * txbuf);
int seuv_delete();
/*---------------------------------------------------*/
/*              House Keeping Data                   */
void hk_write_dup(uint8_t frameCont[]);
int hk_write(uint8_t frameCont[], int SD_partition);
int hk_read(char fileName[], void * txbuf);
int hk_delete();
/*---------------------------------------------------*/
/*            Early Orbit Phase Data                 */
void eop_write_dup(uint8_t frameCont[]);
int eop_write(uint8_t frameCont[], int SD_partition);
int eop_read(char fileName[], void * txbuf);
int eop_delete();
/*---------------------------------------------------*/
void para_d_flash();
void para_w_flash();
int para_r_flash();
/*---------------------------------------------------*/
int downlink_data_before_t(uint8_t datatype, uint32_t time1);
int downlink_data_after_t(uint8_t datatype, uint16_t time1);
int downlink_data_between_t(uint8_t datatype, uint16_t time1, uint16_t time2);

int delete_data_before_t(uint8_t datatype, uint32_t time1);
int delete_data_after_t(uint8_t datatype, uint16_t time1);
int delete_data_between_t(uint8_t datatype, uint16_t time1, uint16_t time2);
/*---------------------------------------------------*/

/*---------------------------------------------------*/
int T_data_d();
int thurmal_2_w();
int thurmal_1_w();
/*---------------------------------------------------*/
/*                 Schedule related                  */
int schedule_reset_flash();
int schedule_write_flash(uint8_t * frameCont);
int schedule_read_flash(uint8_t * txbuf);
int schedule_write(uint8_t * frameCont);
int schedule_read(uint8_t * txbuf);
int schedule_reset();
int schedule_delete(int range, uint8_t * frameCont);
int schedule_shift(uint8_t *frameCont);
int schedule_dump();
/*---------------------------------------------------*/

int scan_files_Downlink (char* path, int mode, uint32_t timeRec_T1, uint32_t timeRec_T2);
int scan_files_Delete (char* path, int mode, uint32_t timeRec_T1, uint32_t timeRec_T2);
int scan_files_Count (char* path, int mode, uint32_t timeRec_T1, uint32_t timeRec_T2);

int errPacket_write(uint8_t *frameCont);
int errPacket_read(uint8_t * txbuf);
int errPacket_dump();
int errPacket_reset();

extern char* fileName_HK[2];
extern char* fileName_INMS[2];
extern char* fileName_SEUV[2];
extern char* fileName_EOP[2];
extern char* fileName_WOD[2];

#endif /* FS_H_ */
