#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <fat_sd/ff.h>

FRESULT scan_files (char* path);
#define F_PUTS			0	//write string into the file
#define F_READ			1	//read the string out of the file
#define F_UNLINK		0	//delete the file
#define SCAN_FILES		0	//scan the directory

FATFS fs;
FRESULT res;
FIL file;
UINT br;
BYTE buffer[4096];

void vTasksdtest(void * pvParameters) {

	int n;
	f_mount(0, &fs);
	res = f_open(&file, "0:/data.txt",FA_OPEN_ALWAYS|FA_READ|FA_WRITE );

	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
	}else{
		printf("\r\n f_open() success .. \r\n");
	}
#if F_READ

	while(1){
		res = f_read(&file,buffer,1,&br);

		if(res == FR_OK)
		{
			printf("%s",buffer);
		}else{
			printf("\r\n f_read() fail .. \r\n");
		}
		if(f_eof(&file)){break;}
	}
#endif

#if F_PUTS
	//將pointer指向文件最後面
	res = f_lseek(&file,file.fsize);

	n = f_puts("\r\n hello PHOENIX ..\r\n", &file) ;//寫入字串
	if(n<1)  //判斷是否成功
	{
		printf("\r\n f_puts() fail .. \r\n");
	}else{
		printf("\r\n f_puts() success .. \r\n");
	}

#endif

#if F_UNLINK
	res = f_unlink("test.jpg");
	if(res!=FR_OK)
	{
		printf("\r\n f_unlink() fail .. \r\n");
	}else{
		printf("\r\n f_unlink() success .. \r\n");
	}
#endif

#if SCAN_FILES
	printf("\r\n the directory files : \r\n");
	scan_files("/");
#endif
	f_close(&file);
	f_mount(0, NULL);
    while(1);
}

FRESULT scan_files (
    char* path        /* Start node to be scanned (also used as work area) */
)
{
    FRESULT res;
    FILINFO fno;
    FATDIR dir;
    int i;
    char *fn;   /* This function is assuming non-Unicode cfg. */

#if _USE_LFN

    static char lfn[_MAX_LFN + 1];
    fno.lfname = lfn;
    fno.lfsize = sizeof lfn;

#endif

    res = f_opendir(&dir, path);             /* Open the directory */
    if (res == FR_OK) {
        i = strlen(path);
        for (;;) {
            res = f_readdir(&dir, &fno);                   /* Read a directory item */
            if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
            if (fno.fname[0] == '.') continue;             /* Ignore dot entry */

#if _USE_LFN
            fn = *fno.lfname ? fno.lfname : fno.fname;
#else
            fn = fno.fname;
#endif
            if (fno.fattrib & AM_DIR) {                    /* It is a directory */
                sprintf(&path[i], "/%s", fn);
                res = scan_files(path);
                if (res != FR_OK) break;
                path[i] = 0;
            } else {                                       /* It is a file. */
                printf("\r\n %s/%s \r\n", path, fn);
            }
        }
    }
    return res;
}

