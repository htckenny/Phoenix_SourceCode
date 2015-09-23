/*
 * SEUV_Task.c

 *
 *  Created on: 2015/3/18
 *      Author: rusei
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

void calculate_avg_std(uint8_t ch, uint8_t data[] , uint8_t numbers) {
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
    if (ch == 2) {
        seuvFrame.ch2AVG = (total / numbers);
        total = 0;
        for (flag = 0; flag < numbers; flag++) {
            total += (tmp[flag] - seuvFrame.ch2AVG) * (tmp[flag] - seuvFrame.ch2AVG);
        }
        seuvFrame.ch2STD = sqrt(total);
    }
    if (ch == 3) {
        seuvFrame.ch3AVG = (total / numbers);
        total = 0;
        for (flag = 0; flag < numbers; flag++) {
            total += (tmp[flag] - seuvFrame.ch3AVG) * (tmp[flag] - seuvFrame.ch3AVG);
        }
        seuvFrame.ch3STD = sqrt(total);
    }
    if (ch == 4) {
        seuvFrame.ch4AVG = (total / numbers);
        total = 0;
        for (flag = 0; flag < numbers; flag++) {
            total += (tmp[flag] - seuvFrame.ch4AVG) * (tmp[flag] - seuvFrame.ch4AVG);
        }
        seuvFrame.ch4STD = sqrt(total);
    }
}

uint8_t seuv_take_data(uint8_t ch) {
    uint8_t rx[10];
    int a;
    int b;
    uint8_t frame[3 * seuvFrame.samples];
    uint8_t tx[2];

    if (ch == 1) {
        tx[0] = parameters.seuv_ch1_conf;
        tx[1] = parameters.seuv_ch1_conf;
    }
    else if (ch == 2) {
        tx[0] = parameters.seuv_ch2_conf;
        tx[1] = parameters.seuv_ch2_conf;
    }
    else if (ch == 3) {
        tx[0] = parameters.seuv_ch3_conf;
        tx[1] = parameters.seuv_ch3_conf;
    }
    else if (ch == 4) {
        tx[0] = parameters.seuv_ch4_conf;
        tx[1] = parameters.seuv_ch4_conf;
    }
    else {
        tx[0] = parameters.seuv_ch1_conf;
        tx[1] = parameters.seuv_ch1_conf;
    }

    i2c_master_transaction(0, seuv_node, &tx, 1, &rx, seuv_data_length, seuv_delay);


    for (a = 0; a < seuvFrame.samples; a++) {
        if (i2c_master_transaction(0, seuv_node, 0 , 0, &rx, seuv_data_length, seuv_delay) == E_NO_ERR)
            for (b = 0; b < 3; b++)
                frame[(a * 3) + b] = rx[b];
        else
            return  ERR_I2C_FAIL;
    }
    calculate_avg_std(ch, frame, seuvFrame.samples);




    return ERR_SUCCESS;
}

void seuv_work_with_inms() {
    power_control(3, ON);
    uint8_t count = 0;

    for (int ch = 1; ch < 5; ch++)
        count += seuv_take_data(ch);

    if (count == 0)
        seuv_write();
    printf("SEUV Work With INMS Done\n");
    power_control(3, OFF);
}

void get_a_packet() {

    power_control(3, ON);
    vTaskDelay(2000);
    uint8_t ch;
    uint8_t count;   // error count , 0 = no error
    timestamp_t t;
    count = 0;
    for (ch = 1; ch < 5; ch++) {
        count += seuv_take_data(ch);
    }

    if (count == 0) {
        t.tv_sec = 0;
        t.tv_nsec = 0;
        obc_timesync(&t, 6000);  //get time
        seuvFrame.packettime = csp_hton32(t.tv_sec);
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

        xLastWakeTime = xTaskGetTickCount();

        if (parameters.seuv_mode == 0x01) {
            if (HK_frame.sun_light_flag == 1)
                get_a_packet();
        }


        if (parameters.seuv_mode == 0x02) {
            get_a_packet();
        }

        vTaskDelayUntil( &xLastWakeTime, xFrequency );
    }

    /* End of seuv */
    vTaskDelete(NULL);

}

