/*
 * task_SEUV.h
 *
 *  Created on: 2015/7/8
 *      Author: rusei
 */

#ifndef SEUV_TASK_H_
#define SEUV_TASK_H_

#endif /* SEUV_TASK_H_ */

// uint8_t seuv_take_data(uint8_t ch, int gain, uint8_t frame[][3*seuvFrame.samples]) ;
uint8_t seuv_take_data(uint8_t ch, int gain, uint8_t *frame);
void get_a_packet(int gain);