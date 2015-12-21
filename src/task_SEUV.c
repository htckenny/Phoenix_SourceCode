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
#include <util/hexdump.h>
#include <dev/i2c.h>
#include "parameter.h"
#include "subsystem.h"
#include "fs.h"
#include <nanomind.h>
#include <csp/csp_endian.h>
#include "task_SEUV.h"
#include <string.h>
#include <math.h>

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
    /* Calculate the raw data from the received three bytes */
    for (flag = 0; flag < numbers; flag++)
        tmp[flag] = (data[flag * 3] << 16) + (data[flag * 3 + 1] << 8) + data[flag * 3 + 2];

    /* Add all of the numbers of samples */
    for (flag = 0; flag < numbers; flag++)
        total = total + tmp[flag];

    /* Calculate the Average and the standard deviation of the samples */
    if (ch == 1) {
        // printf("%d\n", total );
        seuvFrame.ch1AVG = (total / numbers);
        // printf("%f\n", seuvFrame.ch1AVG);
        total = 0;
        for (flag = 0; flag < numbers; flag++) {
            // printf("%d\t%f\n", tmp[flag], seuvFrame.ch1AVG);
            total += pow((tmp[flag] - seuvFrame.ch1AVG), 2);
            // printf("total = %d\n", total);
        }
        // printf("total = %f\n", total);
        seuvFrame.ch1STD = sqrt(total);
        // printf("std = %f\n", seuvFrame.ch1STD);
    }
    else if (ch == 2) {
        seuvFrame.ch2AVG = (total / numbers);
        total = 0;
        for (flag = 0; flag < numbers; flag++) {
            total += pow((tmp[flag] - seuvFrame.ch2AVG), 2);
        }
        seuvFrame.ch2STD = sqrt(total);
    }
    else if (ch == 3) {
        seuvFrame.ch3AVG = (total / numbers);
        total = 0;
        for (flag = 0; flag < numbers; flag++) {
            total += pow((tmp[flag] - seuvFrame.ch3AVG), 2);
        }
        seuvFrame.ch3STD = sqrt(total);
    }
    else if (ch == 4) {
        seuvFrame.ch4AVG = (total / numbers);
        total = 0;
        for (flag = 0; flag < numbers; flag++) {
            total += pow((tmp[flag] - seuvFrame.ch4AVG), 2);
        }
        seuvFrame.ch4STD = sqrt(total);
    }
}

// uint8_t seuv_take_data(uint8_t ch, int gain, uint8_t frame[][3*seuvFrame.samples]) {
uint8_t seuv_take_data(uint8_t ch, int gain, uint8_t *frame) {
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

    i2c_master_transaction(0, seuv_node, &tx, 1, 0, 0, seuv_delay);
    // printf("node : %d\n", tx[0]);
    /* Delay time to allow MCP3424 finish one sampling */
    vTaskDelay(75); //Stiil need to discuss
    if (i2c_master_transaction(0, seuv_node, 0, 0, &rx, seuv_data_length, seuv_delay) == E_NO_ERR){
        hex_dump(&rx, 5); 
        memcpy(frame, rx, 3);
    }
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
void get_a_packet(int gain) {

    uint8_t count = 0;           // error count , 0 = no error
    uint8_t frame_1 [3 * parameters.seuv_sample_rate];
    uint8_t frame_2 [3 * parameters.seuv_sample_rate];
    uint8_t frame_3 [3 * parameters.seuv_sample_rate];
    uint8_t frame_4 [3 * parameters.seuv_sample_rate];

    for (int i = 0 ; i < parameters.seuv_sample_rate ;i++){
        frame_1[i] = 0;
        frame_2[i] = 0;
        frame_3[i] = 0;
        frame_4[i] = 0;
    }


    /* Take data from SEUV in numbers of samples [Gain = 1]*/
    if (gain == 1) {
        /* Power on SEUV */
        power_control(3, ON);
        vTaskDelay(2000);

        for (int i = 0 ; i < parameters.seuv_sample_rate ; i++){
            seuv_take_data(1, 1, &frame_1[3 * i]);  
            seuv_take_data(2, 1, &frame_2[3 * i]);  
            seuv_take_data(3, 1, &frame_3[3 * i]);  
            seuv_take_data(4, 1, &frame_4[3 * i]);  
        }
        printf("Gain 1 : take data\n");
        if (count == 0) {
            calculate_avg_std(1, frame_1, parameters.seuv_sample_rate);      
            calculate_avg_std(2, frame_2, parameters.seuv_sample_rate);
            calculate_avg_std(3, frame_3, parameters.seuv_sample_rate);
            calculate_avg_std(4, frame_4, parameters.seuv_sample_rate);

            printf("1 A = %f\n", seuvFrame.ch1AVG);
            printf("1 S = %f\n", seuvFrame.ch1STD);
            printf("2 A = %f\n", seuvFrame.ch2AVG);
            printf("2 S = %f\n", seuvFrame.ch2STD);
            printf("3 A = %f\n", seuvFrame.ch3AVG);
            printf("3 S = %f\n", seuvFrame.ch3STD);
            printf("4 A = %f\n", seuvFrame.ch4AVG);
            printf("4 S = %f\n", seuvFrame.ch4STD);
            seuvFrame.samples += 0 ; 
            // printf("sample = %d\n", seuvFrame.samples);
            if (seuv_write() == No_Error)
                printf("Write a packet into SEUV.bin\n");
            else
                printf("Fail to write into SEUV.bin\n");
            seuvFrame.samples -= 0 ; 
        }
        else {
            printf("Fail to get SEUV data\n");
        }
    }
    else if (gain == 8) {
        printf("Gain 8 : take data\n");
        //channel 1~4 sample 50 times
        for (int i = 0 ; i < parameters.seuv_sample_rate ; i++){     
            seuv_take_data(1, 8, &frame_1[3 * i]);  
            seuv_take_data(2, 8, &frame_2[3 * i]);  
            seuv_take_data(3, 8, &frame_3[3 * i]);  
            seuv_take_data(4, 8, &frame_4[3 * i]);  
        }
        if (count == 0) {       
            calculate_avg_std(1, frame_1, parameters.seuv_sample_rate);      
            calculate_avg_std(2, frame_2, parameters.seuv_sample_rate);
            calculate_avg_std(3, frame_3, parameters.seuv_sample_rate);
            calculate_avg_std(4, frame_4, parameters.seuv_sample_rate);

            printf("1 A = %f\n", seuvFrame.ch1AVG);
            printf("1 S = %f\n", seuvFrame.ch1STD);
            printf("2 A = %f\n", seuvFrame.ch2AVG);
            printf("2 S = %f\n", seuvFrame.ch2STD);
            printf("3 A = %f\n", seuvFrame.ch3AVG);
            printf("3 S = %f\n", seuvFrame.ch3STD);
            printf("4 A = %f\n", seuvFrame.ch4AVG);
            printf("4 S = %f\n", seuvFrame.ch4STD);
            seuvFrame.samples += 1 ; 
            if (seuv_write() == No_Error)
                printf("Write a packet into SEUV.bin\n");
            else
                printf("Fail to write into SEUV.bin\n");
            seuvFrame.samples -= 1 ; 
        }
        else {
            printf("Fail to get SEUV data\n");
        }   
        /* Power off SEUV */
        power_control(3, OFF);
    }
}

void SolarEUV_Task(void * pvParameters) {

    portTickType xLastWakeTime;
    portTickType xFrequency = 1000;
    parameter_init();
    while (1) {

        if (parameters.seuv_period != 0) {
            xFrequency = parameters.seuv_period * 1000;
            // xFrequency = 10 * 1000;
        }
        /* Set the delay time during one sampling operation*/
        xLastWakeTime = xTaskGetTickCount();
        printf("mode = %d\n", parameters.seuv_mode);
        if (parameters.seuv_mode == 0x01) {         /* Mode A: check if the CubeSat is in the sun light area */
            if (HK_frame.sun_light_flag == 1) {     /* If True, get a packet */
                get_a_packet(1);
                get_a_packet(8);
            }      
        }
        else if (parameters.seuv_mode == 0x02) {    /* Mode B: Keep sampling every 8 seconds */           
            get_a_packet(1);
            get_a_packet(8);
        }
        else if (parameters.seuv_mode == 0x03) {    
            printf("No measurement taken\n");
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

    /* End of seuv */
    vTaskDelete(NULL);
}

