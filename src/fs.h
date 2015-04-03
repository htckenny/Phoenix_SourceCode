/*
 * FS.h
 *
 *  Created on: 2014/11/20
 *
 */
#ifndef FS_H_
#define FS_H_
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

int inms_script_write(int buffNum,uint8_t scriptCont[]);
void inms_script_read(int buffNum,int packlength,void * txbuf);
int inms_script_length(int buffNum);
int inms_data_write(uint8_t frameCont[] );
int inms_data_read(int serial,void * txbuf);        // serial= which packet you want to read , Range 1~N
int inms_data_delete();                                                       //return 0 when success, return -1 when fail.
/*---------------------------------------------------*/

int wod_write(uint8_t frameCont[] );
int wod_read(int serial,void * txbuf);
int wod_delete();
/*---------------------------------------------------*/
int seuv_write();
int seuv_read(int serial,void * txbuf);
int seuv_delete();
/*---------------------------------------------------*/
int hk_write(uint8_t frameCont[] );
int hk_read(int serial,void * txbuf);
int hk_delete();
/*---------------------------------------------------*/

int para_w();
int para_r();
int para_d();


int inms_data_dump();
int wod_data_dump();
int seuv_data_dump();
int hk_data_dump();
#endif /* FS_H_ */
