int inms_script_write(int buffNum,uint8_t scriptCont[]);
void inms_script_read(int buffNum,int packlength,void * txbuf);
//int inms_script_run(int buffNum);
int inms_script_length(int buffNum);
int wod_write(char day[], uint8_t frameCont[]);
void wod_down(int type,char day[],int serNum,void * txbuf);
