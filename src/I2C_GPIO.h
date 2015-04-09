void i2c_mpio_init(void) ;
void i2c_mpio_start(void);
void i2c_mpio_stop(void) ;
void i2c_mpio_write(unsigned char data) ;
unsigned char i2c_mpio_read(unsigned char send_ack) ;
int send_i2c_adcs(unsigned char reg1,unsigned char reg2, int rxNum);