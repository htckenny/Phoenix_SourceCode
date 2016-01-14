
#include <dev/i2c.h>
#include "fs.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <util/hexdump.h>
#include "parameter.h"
#include "subsystem.h"
#include "tele_function.h"

void delete_buf() {

	uint8_t txdata = com_rx_delete;
	i2c_master_transaction(0, com_rx_node, &txdata, 1, 0, 0, com_delay);
}

/* Read incoming commnad */
void Read_Execute() {  

	uint8_t txdata = com_rx_get;
	uint8_t val[201];  // expect  1+200 byte response , first byte is length1~200

	if (i2c_master_transaction_2(0, com_rx_node, &txdata, 1, &val, 200, com_delay) == E_NO_ERR) {
		hex_dump(&val[0], 6);    // get command  print in
		hex_dump(&val[6], val[0]); // Byte 0~1=packet data length, byte 2~5= signal strength , byte 6~N = data part
		delete_buf();   // delete one frame in receive buffer
		decodeCCSDS_Command(&val[6], val[0]);
	}
	else
		printf("read Telecommand Fails\n");
}

/* Check incoming command */
int CIC() { 

	uint8_t txdata = com_rx_check;
	uint8_t val; 

	if (i2c_master_transaction_2(0, com_rx_node, &txdata, 1, &val, 1, com_delay) == E_NO_ERR) {
		return (uint8_t)val;
	}
	else
		return 255;
}

void Telecom_Task(void * pvParameters) {

	uint8_t flag;
	// int tx_wdt_flag = 0;
	// uint8_t txdata = com_tx_hk;

	set_Call_Sign(0);
	set_tx_rate(8);

	if (parameters.shutdown_flag == 1) {
		printf("Shutdown Command Detected!! \r\n");
	}
	if (tx_mode(1) != 0) {  //set transceiver into auto mode
		printf("tx_mode set fail \r\n");
	}

	while (1) {

		/*-------Avoid COM WDT(1 minute) reset itself ---------*/
		// tx_wdt_flag++;
		// if (tx_wdt_flag >= 25) {
		// 	txdata = com_tx_hk;
		// 	i2c_master_transaction(0, com_tx_node, &txdata, 1, &txdata, 1, com_delay);

		// 	if ((txdata < 12) && (parameters.com_bit_rates == 8)) {
		// 		set_tx_rate(parameters.com_bit_rates);
		// 		set_Call_Sign(0);
		// 	}
		// 	if ((txdata > 3) && (parameters.com_bit_rates == 1)) {
		// 		set_tx_rate(parameters.com_bit_rates);
		// 		set_Call_Sign(0);
		// 	}
		// 	tx_wdt_flag = 0;
		// }		

		/*----------------------------------*/
		flag = CIC();

		if ((flag > 0) && (flag < 41)) {
			printf("--------------------------------------------- \r\n");
			printf("Find %d frame in receive buffer \r\n", flag);
			Read_Execute();
			printf("--------------------------------------------- \r\n");
			vTaskDelay(0.1 * delay_time_based);
		}

		if (flag > 40 || flag == 0)
			vTaskDelay(1 * delay_time_based);

		parameters.com_bit_rates = (parameters.first_flight == 1) ? 1 : 8;
		set_tx_rate(parameters.com_bit_rates);
	}
}