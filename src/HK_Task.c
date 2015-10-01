/*
 * HK_Task.c

 *
 *  Created on: 2015/3/28
 *      Author: rusei
 */
#include <util/timestamp.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <dev/cpu.h>
#include <util/delay.h>
#include "parameter.h"
#include <io/nanomind.h>
#include <csp/csp_endian.h>
#include <dev/i2c.h>
#include "Tele_function.h"
#include "subsystem.h"
#include <string.h>
#include <util/hexdump.h>
#include <time.h>
#include <dev/adc.h>
#include <dev/usart.h>
#include "mpio.h"
#include "fs.h"
#include <io/nanopower2.h>
#include <csp/csp.h>

int num;
uint8_t hk_buffer[200];

/* Thermistor Sensor data */
void TS12_16() {

    int nums = 0;
    char uchar[174 * 10];
    nums = usart_messages_waiting(2);
    if (nums != 0) {
        for (int f = 0; f < nums; f++)
            uchar[f] = usart_getc(2);

        if ((uint8_t)uchar[0] == 0x0A) {
            memcpy(&Tdata[0], &num, 4);
            memcpy(&Tdata[4], &uchar[0], 174);
            thurmal_2_w();
            printf("Get a Side panel thermal sensor packet # %d \n", num);
            hex_dump(&Tdata[0], 178);
        }
    }
}

/* interface board  INMS temperature */
int TS11() {
    uint8_t rx[10];
    uint8_t tx[2];
    tx[0] = 0xD0;
    tx[1] = 0xD0;

    i2c_master_transaction(0, interface_node, &tx, 2, 0, 0, 0);
    i2c_master_transaction(0, interface_node, 0 , 0, &rx, 4, seuv_delay);

    if (i2c_master_transaction(0, interface_node, 0 , 0, &rx, 4, seuv_delay) != E_NO_ERR)
        return Error;

    // memcpy(&ThurmalFrame.T10,&rx[0],3);
    ThurmalFrame.T11 = (rx[0] << 8) + rx[1];


    return No_Error;
}

/* interface temperature */
int TS10() {   
    uint8_t rx[10];

    uint8_t tx[2];
    tx[0] = 0xF0;
    tx[1] = 0xF0;


    i2c_master_transaction(0, interface_node, &tx, 2, 0, 0, 0);
    i2c_master_transaction(0, interface_node, 0 , 0, &rx, 4, seuv_delay);


    if (i2c_master_transaction(0, interface_node, 0 , 0, &rx, 4, seuv_delay) != E_NO_ERR)
        return Error;


    // memcpy(&ThurmalFrame.T10,&rx[0],3);
    ThurmalFrame.T10 = (rx[0] << 8)+ rx[1];
    //ThurmalFrame.T10=0x0A;

    return No_Error;
}

/* obc temp */ 
int TS9() {  
    uint16_t * adc_buffer;

    adc_buffer = adc_start_blocking(1);

    ThurmalFrame.T9 = (uint16_t)((((adc_buffer[0] * 2493.0) / 1023) - 424) / 6.25);
    //ThurmalFrame.T9=0x09;
    return No_Error;
}

 // adcs temp
int TS7() { 
    uint8_t txbuffer = 175;
    uint8_t rxbuffer[6];

    if (i2c_master_transaction(0, adcs_node, &txbuffer, 1, &rxbuffer, 6, adcs_delay) != E_NO_ERR)
        return Error;


    ThurmalFrame.T7 = (uint16_t)rxbuffer[5];

    return No_Error;
}

// antenna temp
int TS6() {   
    uint8_t txbuffer = 0xC0;
    uint16_t rxbuffer;

    if (i2c_master_transaction(0, ant_node, &txbuffer, 1, &rxbuffer, 2, com_delay) != E_NO_ERR)
        return Error;

    ThurmalFrame.T6 = rxbuffer;

    return No_Error;
}
// com temp
int TS5() {   
    uint8_t txbuffer = com_rx_hk;
    uint16_t rxbuffer[7];

    if (i2c_master_transaction(0, com_rx_node, &txbuffer, 1, &rxbuffer, 14, com_delay) != E_NO_ERR)
        return Error;

    //rxbuffer=csp_ntoh16(rxbuffer);
    ThurmalFrame.T5 = rxbuffer[5];
    //ThurmalFrame.T5=0x05;

    return No_Error;
}

// eps temp
int TS1_4() {   
    eps_hk_t * chkparam;

    i2c_frame_t * frame = csp_buffer_get(I2C_MTU);
    frame->dest = eps_node;
    frame->data[0] = EPS_PORT_HK;
    frame->data[1] = 0;
    frame->len = 2;
    frame->len_rx = 2 + (uint8_t) sizeof(eps_hk_t);
    frame->retries = 0;

    if (i2c_send(0, frame, 0) != E_NO_ERR) {
        csp_buffer_free(frame);
        return Error;
    }

    if (i2c_receive(0, &frame, 20) != E_NO_ERR)
        return Error;

    chkparam = (eps_hk_t *)&frame->data[2];
    eps_hk_unpack(chkparam);
    csp_buffer_free(frame);

    ThurmalFrame.T1 = chkparam->temp[0];
    ThurmalFrame.T2 = chkparam->temp[1];
    ThurmalFrame.T3 = chkparam->temp[2];
    ThurmalFrame.T4 = chkparam->temp[3];


    return No_Error;

}

void clean_all() {

    ThurmalFrame.T1 = 0;
    ThurmalFrame.T2 = 0;
    ThurmalFrame.T3 = 0;
    ThurmalFrame.T4 = 0;
    ThurmalFrame.T5 = 0;
    ThurmalFrame.T6 = 0;
    ThurmalFrame.T7 = 0;
    ThurmalFrame.T8 = 0;
    ThurmalFrame.T9 = 0;
    ThurmalFrame.T10 = 0;
    ThurmalFrame.T11 = 0;
}

uint16_t interface_3V3_current() {
    uint8_t rx[10];
    uint8_t tx[2];
    tx[0] = 0x90;
    tx[1] = 0x90;

    i2c_master_transaction(0, interface_node, &tx, 2, 0, 0, 0);
    i2c_master_transaction(0, interface_node, 0 , 0, &rx, 4, seuv_delay);


    if (i2c_master_transaction(0, interface_node, 0 , 0, &rx, 4, seuv_delay) != E_NO_ERR)
        return 0;

    return  (rx[0] * 256 + rx[1]);
}

uint16_t interface_5V_current() {
    uint8_t rx[10];
    uint8_t tx[2];
    tx[0] = 0xB0;
    tx[1] = 0xB0;

    i2c_master_transaction(0, interface_node, &tx, 2, 0, 0, 0);
    i2c_master_transaction(0, interface_node, 0 , 0, &rx, 4, seuv_delay);


    if (i2c_master_transaction(0, interface_node, 0 , 0, &rx, 4, seuv_delay) != E_NO_ERR)
        return 0;

    return  (rx[0] * 256 + rx[1]);
}

void thermal_test(void * pvParameters) {
    int nums = 0;
    char uchar[174 * 10];
    num = 0;
    power_OFF_ALL();
    vTaskDelay(2000);


    power_control(4, ON);
    power_control(1, ON);

    while (1) {

        num++;
        clean_all();
        ThurmalFrame.packet_number = num;
        TS1_4();
        TS5();
        TS6();
        TS7();
        TS9();
        TS10();
        TS11();
        printf("------------------------------------- \n");
        printf("NUM = %d ,T1= %04X ,T2 %04X ,T3= %04X ,T4= %04X ,T5= %04X \n", (int)ThurmalFrame.packet_number, ThurmalFrame.T1, ThurmalFrame.T2, ThurmalFrame.T3, ThurmalFrame.T4, ThurmalFrame.T5);
        printf("T6= %04X ,T7 %04X ,T8= %04X ,T9= %04X ,T10= %04X \n", ThurmalFrame.T6, ThurmalFrame.T7, ThurmalFrame.T8, ThurmalFrame.T9, ThurmalFrame.T10);

        thurmal_1_w();
        TS12_16();


        nums = usart_messages_waiting(2);
        if (nums != 0) {
            for (int f = 0; f < nums; f++)
                uchar[f] = usart_getc(2);
            hex_dump(&uchar, 174);

            timestamp_t t;
            t.tv_sec = 0;
            t.tv_nsec = 0;
            obc_timesync(&t, 6000);
            time_t tt = t.tv_sec + 946684800;

            printf("OBC time is: %s\r\n", ctime(&tt));
        }
        vTaskDelay(1000);
    }
}

uint8_t hk_get() {
    uint8_t txbuf;
    timestamp_t t;
    t.tv_sec = 0;
    t.tv_nsec = 0;
    obc_timesync(&t, 6000);
    // t.tv_sec = csp_hton32(t.tv_sec);
    memcpy(&hk_buffer[0], &t.tv_sec, 4); // get Time LSB


    /*get adcs hk1*/
       txbuf = 0x88;
       if (i2c_master_transaction(0, adcs_node, &txbuf, 1, &hk_buffer[4], 48, adcs_delay) != E_NO_ERR)
           return Error;
    /*get adcs hk2*/
       txbuf = 0x89;
       if (i2c_master_transaction(0, adcs_node, &txbuf, 1, &hk_buffer[52], 36, adcs_delay) != E_NO_ERR)
           return Error;
    /*get adcs hk3*/
       txbuf = 0x8A;
       if (i2c_master_transaction(0, adcs_node, &txbuf, 1, &hk_buffer[88], 12, adcs_delay) != E_NO_ERR)
           return Error;

    /* get System Status*/
    HK_frame.reboot_count = parameters.reboot_count;
    HK_frame.interface_3V3_current = interface_3V3_current();
    HK_frame.interface_5V_current = interface_5V_current();
    memcpy(&hk_buffer[100], &HK_frame.mode_status_flag, (int)sizeof(hk_frame_t));

    // get Thermal Status
    TS1_4();
    TS5();
    TS6();
    TS7();
    TS9();
    TS10();
    TS11();
    memcpy(&hk_buffer[100 + sizeof(hk_frame_t)], &ThurmalFrame.T6, ((int)sizeof(thurmal_frame_t)) - 14);


   // now total length = 122 byte

    // Finished , print the complete frame
    hex_dump(&hk_buffer[0], hk_length);


    return No_Error;

}
void clean_hk_buffer() {

    for (int f = 0; f < 200; f++)
        hk_buffer[f] = 0;
}

void HK_Task(void * pvParameters) {

    while (1) {

        if (parameters.first_flight == 1)
            vTaskDelay(3000);  //30000
        else
            vTaskDelay(6000);  //60000

        clean_hk_buffer();
        clean_all();

        if (hk_get() != No_Error)
            printf("HK get fail\n");
        else {
            hk_write(&hk_buffer[0]);
            printf("HK Get and Saved\n");
        }
    }
    /** End of HK  ,should never reach this */
    vTaskDelete(NULL);
}
