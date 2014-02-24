/* Read a text file and display it */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

FATFS FatFs;   /* Work area (file system object) for logical drive */


void vTaskread(void * pvParameters) {
	
	FIL fil;       /* File object */
    char line[82]; /* Line buffer */
    FRESULT fr;    /* FatFs return code */


    /* Register work area to the default drive */
    f_mount(&FatFs, "", 0);

    /* Open a text file */
    fr = f_open(&fil, "test.txt", FA_READ);
    if (fr) return (int)fr;

    /* Read all lines and display it */
    while (f_gets(line, sizeof line, &fil))
        printf("this is the txt print!! \n %d",line);

    /* Close the file */
    f_close(&fil);

    //return 0;
}


