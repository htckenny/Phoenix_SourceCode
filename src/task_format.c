#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <fat_sd/ff.h>
#include <util/hexdump.h>
#include <util/timestamp.h>
#include <nanomind.h>
#include <dev/i2c.h>
#include <dev/adc.h>
#include <vfs/vfs.h>
#include <time.h>
#include <string.h>
#include <inttypes.h>

#include "fs.h"
#include "parameter.h"
#include "subsystem.h"


void task_format(void* pvParameters) {

	uint8_t label ;
	label =  (uintptr_t) pvParameters;
	printf("label = %d\n", label);
	FIL file;
	FATFS fs;
	if (f_mount(label, NULL) == FR_OK)
		printf("unmount %d\n", label);

	if (f_mount(label, &fs) == FR_OK)
		printf("mount %d\n", label);

	if (label == 0) {
		f_mkfs(0, 0, 0);
		f_mkdir("0:/HK_DATA");
		f_mkdir("0:/INM_DATA");
		f_mkdir("0:/SEU_DATA");
		f_mkdir("0:/EOP_DATA");
		f_mkdir("0:/WOD_DATA");
		f_mkdir("0:/image");
		f_open(&file, "0:/part0", FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
		f_close(&file);
	}
	else if (label == 1) {
		f_mkfs(1, 0, 0);
		f_mkdir("1:/HK_DATA");
		f_mkdir("1:/INM_DATA");
		f_mkdir("1:/SEU_DATA");
		f_mkdir("1:/EOP_DATA");
		f_mkdir("1:/WOD_DATA");
		f_mkdir("1:/image");
		f_open(&file, "1:/part1", FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
		f_close(&file);
	}
	format_task = NULL;
	vTaskDelete(NULL);
}
