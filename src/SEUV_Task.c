/*
 * SEUV_Task.c
 *
 *  Created on:     2015/3/18
 *  Last Update:    2015/9/25
 *      Author: rusei, Kenny
 */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include <util/timestamp.h>
#include <dev/i2c.h>
#include "parameter.h"
#include "subsystem.h"
#include "fs.h"
#include <io/nanomind.h>
#include <csp/csp_endian.h>
#include "SEUV_Task.h"
#include <string.h>

/**
 * This function is used for calculate the average and the standard deviation of the SEUV data
 * @param ch      channel
 * @param data    data array
 * @param numbers number of the data
 */


void calculate_avg_std(uint8_t ch, uint8_t data[], uint8_t numbers) {
    int tmp[numbers];
    int total = 0;
    int flag;
    for (flag = 0; flag < numbers; flag++)
        tmp[flag] = (data[flag * 3] * 256 * 256) + (data[flag * 3 + 1] * 256) + data[flag * 3 + 2];

    for (flag = 0; flag < numbers; flag++)
        total = total + tmp[flag];


    if (ch == 1) {
        seuvFrame.ch1AVG = (total / numbers);
        total = 0;
        for (flag = 0; flag < numbers; flag++) {
            total += (tmp[flag] - seuvFrame.ch1AVG) * (tmp[flag] - seuvFrame.ch1AVG);
        }
        seuvFrame.ch1STD = sqrt(total);
    }
    else if (ch == 2) {
        seuvFrame.ch2AVG = (total / numbers);
        total = 0;
        for (flag = 0; flag < numbers; flag++) {
            total += (tmp[flag] - seuvFrame.ch2AVG) * (tmp[flag] - seuvFrame.ch2AVG);
        }
        seuvFrame.ch2STD = sqrt(total);
    }
    else if (ch == 3) {
        seuvFrame.ch3AVG = (total / numbers);
        total = 0;
        for (flag = 0; flag < numbers; flag++) {
            total += (tmp[flag] - seuvFrame.ch3AVG) * (tmp[flag] - seuvFrame.ch3AVG);
        }
        seuvFrame.ch3STD = sqrt(total);
    }
    else if (ch == 4) {
        seuvFrame.ch4AVG = (total / numbers);
        total = 0;
        for (flag = 0; flag < numbers; flag++) {
            total += (tmp[flag] - seuvFrame.ch4AVG) * (tmp[flag] - seuvFrame.ch4AVG);
        }
        seuvFrame.ch4STD = sqrt(total);
    }
}

uint8_t seuv_take_data(uint8_t ch, int gain, uint8_t frame[][3*seuvFrame.samples]) {

    uint8_t rx[10];    
    uint8_t tx[2];

    if (gain == 1) {
        if (ch == 1) {
            tx[0] = parameters.seuv_ch1_G1_conf;
            tx[1] = parameters.seuv_ch1_G1_conf;
        }
        else if (ch == 2) {
            tx[0] = parameters.seuv_ch2_G1_conf;
            tx[1] = parameters.seuv_ch2_G1_conf;
        }
        else if (ch == 3) {
            tx[0] = parameters.seuv_ch3_G1_conf;
            tx[1] = parameters.seuv_ch3_G1_conf;
        }
        else if (ch == 4) {
            tx[0] = parameters.seuv_ch4_G1_conf;
            tx[1] = parameters.seuv_ch4_G1_conf;
        }        
    }
    else if (gain == 8){
        if (ch == 1) {
            tx[0] = parameters.seuv_ch1_G8_conf;
            tx[1] = parameters.seuv_ch1_G8_conf;
        }
        else if (ch == 2) {
            tx[0] = parameters.seuv_ch2_G8_conf;
            tx[1] = parameters.seuv_ch2_G8_conf;
        }
        else if (ch == 3) {
            tx[0] = parameters.seuv_ch3_G8_conf;
            tx[1] = parameters.seuv_ch3_G8_conf;
        }
        else if (ch == 4) {
            tx[0] = parameters.seuv_ch4_G8_conf;
            tx[1] = parameters.seuv_ch4_G8_conf;
        }        
    }

    i2c_master_transaction(0, seuv_node, &tx, 1, &rx, seuv_data_length, seuv_delay);

    if (i2c_master_transaction(0, seuv_node, 0 , 0, &rx, seuv_data_length, seuv_delay) == E_NO_ERR)
        memcpy(frame, &rx, 3);
    else
        return  ERR_I2C_FAIL;

    return ERR_SUCCESS;
}

// void seuv_work_with_inms() {
//     power_control(3, ON);
//     uint8_t count = 0;

//     for (int ch = 1; ch < 5; ch++){
//         count += seuv_take_data(ch);
//     }

//     if (count == 0){
//         seuv_write();
//     }

//     printf("SEUV Work With INMS Done\n");
//     power_control(3, OFF);
// }
void get_a_packet() {

    uint8_t count;           // error count , 0 = no error
    count = 0;
    uint8_t frame[4][3 * seuvFrame.samples];
    // uint8_t frame_2 [3 * seuvFrame.samples];
    // uint8_t frame_3 [3 * seuvFrame.samples];
    // uint8_t frame_4 [3 * seuvFrame.samples];

    // uint8_t frame_g1 [seuvFrame.samples];
    // uint8_t frame_g8 [seuvFrame.samples];

    power_control(3, ON);
    vTaskDelay(2000);
    for (int i = 0 ; i < seuvFrame.samples ; i++){
        for (int ch = 0 ; ch < 4 ; ch++){
            seuv_take_data(ch + 1, 1, &frame[ch] + (3 * i));        //channel 1~4 sample 50 times
        }
        // seuv_take_data(2, 1, &frame[1] + (3 * i));
        // seuv_take_data(3, 1, &frame[2] + (3 * i));
        // seuv_take_data(4, 1, &frame[3] + (3 * i));
    }

    if (count == 0) {
        for (int ch = 0 ; ch < 4 ; ch++){
            calculate_avg_std(ch + 1, frame[ch], seuvFrame.samples);
        }        
        if (seuv_write() == No_Error)
            printf("Write a packet into SEUV.bin\n");
        else
            printf("Fail to write into SEUV.bin\n");
    }
    else {
        printf("Fail to get SEUV data\n");
    }
    count = 0 ;
    for (int i = 0 ; i < 4 ;i++){
        *frame[i] = 0;
    }
    for (int i = 0 ; i < seuvFrame.samples ; i++){
        for (int ch = 0 ; ch < 4 ; ch++){
            seuv_take_data(ch + 1, 8, &frame[ch] + (3 * i));        //channel 1~4 sample 50 times
        }
    }
    if (count == 0) {
        for (int ch = 0 ; ch < 4 ; ch++){
            calculate_avg_std(ch + 1, frame[ch], seuvFrame.samples);
        }         
        if (seuv_write() == No_Error)
            printf("Write a packet into SEUV.bin\n");
        else
            printf("Fail to write into SEUV.bin\n");
    }
    else {
        printf("Fail to get SEUV data\n");
    }

    power_control(3, OFF);
}

void SolarEUV_Task(void * pvParameters) {

    portTickType xLastWakeTime;
    portTickType xFrequency = 1000;

    while (1) {

        if (parameters.seuv_period != 0) {
            xFrequency = parameters.seuv_period * 1000;
        }
        /* Set the delay time during one sampling operation*/
        xLastWakeTime = xTaskGetTickCount();
        printf("mode = %d\n", parameters.seuv_mode);
        if (parameters.seuv_mode == 0x01) {         /* Mode A: check if the CubeSat is in the sun light area */
            if (HK_frame.sun_light_flag == 1)       /* If True, get a packet */
                get_a_packet();
        }
        else if (parameters.seuv_mode == 0x02) {    /* Mode B: Keep sampling every 8 seconds */           
                get_a_packet();
        }
        else if (parameters.seuv_mode == 0x03) {    
            printf("mode = 3, no measurement taken\n");
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

    /* End of seuv */
    vTaskDelete(NULL);

}

