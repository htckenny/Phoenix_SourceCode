/*
 * subsystem.c
 *
 *  Created on: 	2015/03/18
 *  Last update: 	2016/03/04
 *      Author: rusei, Kenny
 */

#include <io/nanopower2.h>
#include <dev/i2c.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <math.h>
#include <csp/csp_endian.h>
#include <csp/csp.h>
#include <dev/arm/ds1302.h>
#include <util/timestamp.h>
#include <nanomind.h>
#include <util/bytesize.h>
#include <vfs/vfs.h>

#include "subsystem.h"
#include "parameter.h"
#include "fs.h"
#include "../lib/liba712/src/drivers/mpio.h"

/* Parameter for ECEG to ECI function */
#define PI 3.14159265
#define omega 7.29211585275553e-005
#define JD_2000 2451545

/* Time period */
#define	SECONDS_DAY			(3600L*24L)
#define	SECONDS_WEEK		(3600L*24L*7L)



/*------------------------------------------------------------
 * Conversion of Time
 *------------------------------------------------------------*/
/* Beginning year of calendar value */
#define	TIME_T_BASE_YEAR	1970

/* Calendar value of 1980/1/6 00:00:00 */
#define	TIME_T_ORIGIN		315964800L

int status_update()
{
	status_frame.mode_task = (mode_task != NULL) ? 1 : 0;
	status_frame.bat_check_task = (bat_check_task != NULL) ? 1 : 0;
	status_frame.com_task = (com_task != NULL) ? 1 : 0;
	status_frame.wod_task = (wod_task != NULL) ? 1 : 0;
	status_frame.beacon_task = (beacon_task != NULL) ? 1 : 0;

	status_frame.init_task = (init_task != NULL) ? 1 : 0;
	status_frame.adcs_task = (adcs_task != NULL) ? 1 : 0;
	status_frame.seuv_task = (seuv_task != NULL) ? 1 : 0;
	status_frame.eop_task = (eop_task != NULL) ? 1 : 0;
	status_frame.hk_task = (hk_task != NULL) ? 1 : 0;

	status_frame.inms_error_handle = (inms_error_handle != NULL) ? 1 : 0;
	status_frame.inms_current_moniter = (inms_current_moniter != NULL) ? 1 : 0;
	status_frame.inms_task = (inms_task != NULL) ? 1 : 0;
	status_frame.inms_task_receive = (inms_task_receive != NULL) ? 1 : 0;

	status_frame.schedule_task = (schedule_task != NULL) ? 1 : 0;
	status_frame.seuv_cm_task = (seuv_cm_task != NULL) ? 1 : 0;

	return E_NO_ERR;
}

uint32_t get_time()
{

	unsigned long time;
	struct ds1302_clock clock;
	if (ds1302_clock_read_burst(&clock) < 0)
		return 0;

	if (ds1302_clock_to_time((time_t *) &time, &clock) < 0)
		return 0;

	return (uint32_t)time;

}

int parameter_init()
{
	abort_transfer_flag 				= 0;
	magnetometer_deploy					= 0;
	HK_frame.sun_light_flag				= 0;
	SD_lock_flag						= 0;
	use_GPS_header						= 0;
	inms_tm_status						= 1;
	seuv_with_INMS 						= 0;

	/*--File System store count--*/
	parameters.wod_store_count			= 0;
	parameters.inms_store_count			= 0;
	parameters.seuv_store_count			= 0;
	parameters.hk_store_count			= 0;
	parameters.eop_store_count			= 0;

	/* Protocol sequence count */
	parameters.obc_packet_sequence_count = 0;
	parameters.ax25_sequence_count       = 0;
	parameters.tc_count                  = 0;

	/* System Configuration */
	parameters.first_flight				= 1;		/* indicates if this is the first flight */
	parameters.shutdown_flag			= 0;
	parameters.ant_deploy_flag			= 0;		/* indicates that antenna already deployed or not */
	parameters.hk_collect_period		= 60;
	parameters.beacon_period			= 30;
	parameters.reboot_count				= 0;
	parameters.com_bit_rates			= 0x01;

	/*  seuv related  */
	parameters.seuv_period				= 15;
	parameters.seuv_sample_rate			= 20;

	parameters.seuv_ch1_G1_conf			= 0x98;		/* Gain 1 configuration */
	parameters.seuv_ch2_G1_conf			= 0xB8;
	parameters.seuv_ch3_G1_conf			= 0xD8;
	parameters.seuv_ch4_G1_conf			= 0xF8;

	parameters.seuv_ch1_G8_conf			= 0x9B;		/* Gain 8 configuration */
	parameters.seuv_ch2_G8_conf			= 0xBB;
	parameters.seuv_ch3_G8_conf			= 0xDB;
	parameters.seuv_ch4_G8_conf			= 0xFB;
	parameters.seuv_mode				= 0x03;

	/* battery*/
	parameters.vbat_recover_threshold	= 7500;
	parameters.vbat_safe_threshold		= 7000;

	parameters.INMS_timeout 			= 400;
	parameters.inms_status				= 1;
	parameters.SD_partition_flag		= 0;

	parameters.ErrSerialNumber			= 0;

	seuvFrame.samples = parameters.seuv_sample_rate << 1 ;		/* samples */
	/* 0 1 2 3 4 5 6 |  7    */
	/*  sample rate  | Gain  */

	/* Read last parameter from FLASH */
	if (para_r_flash() == No_Error) {
		parameters.reboot_count ++;
		para_w_flash();
		return No_Error;
	}
	else
		para_w_flash();
	return Error;
}

void deploy_antenna()
{
	uint8_t txdata[2];

	/* arm ant board */
	txdata[0] = ant_arm;
	i2c_master_transaction_2(0, ant_node, &txdata, 1, 0, 0, com_delay);
	vTaskDelay(0.1 * delay_time_based);

	/* deploy ant board one by one */
	txdata[0] = ant_deploy;
	txdata[1] = ant_deploy_timeout;
	i2c_master_transaction_2(0, ant_node, &txdata, 2, 0, 0, com_delay);
	printf("Antenna Deployed!!\n");
}
int antenna_status_check()
{
	uint8_t txdata;
	uint8_t rxdata[2];
	txdata = 0xC3;
	if (i2c_master_transaction_2(0, ant_node, &txdata, 1, &rxdata, 2, com_delay) == E_NO_ERR) {
		if (((rxdata[0] == 0x01) || (rxdata[0] == 0x00)) && (rxdata[1] == 0x00))
			return No_Error;
		else
			return Error;
	}
	return Error;
}

void power_control(int device, int stats)
{
	/**
	 * Device code:
	 * 1: ADCS
	 * 2: GPS
	 * 3: SEUV
	 * 4: INMS
	 * 5: Interface Board
	 *
	 * Status =>
	 * 1: ON
	 * 0: OFF
	 */

	//   unsigned int mode = stats;
	printf("power %d with device %d\n", stats, device);
	eps_output_set_single_req eps_switch;
	eps_switch.mode = (uint8_t)stats;
	eps_switch.delay = 0;

	uint8_t txdata[100];
	txdata[0] = EPS_PORT_SET_SINGLE_OUTPUT;
	/* ADCS  */
	if (device == 1) {
		/* channel 0 = ADCS 5V */
		eps_switch.channel = 0;
		memcpy(&txdata[1], &eps_switch, sizeof(eps_output_set_single_req));
		i2c_master_transaction_2(0, eps_node, &txdata, 1 + sizeof(eps_output_set_single_req), 0, 0, eps_delay);

		/* channel 3 = ADCS 3.3V */
		eps_switch.channel = 3;
		memcpy(&txdata[1], &eps_switch, sizeof(eps_output_set_single_req));
		i2c_master_transaction_2(0, eps_node, &txdata, 1 + sizeof(eps_output_set_single_req), 0, 0, eps_delay);
	}
	/* GPS  */
	else if (device == 2) {
		/* channel 4 = GPS 3.3V */
		eps_switch.channel = 4;

		memcpy(&txdata[1], &eps_switch, sizeof(eps_output_set_single_req));
		i2c_master_transaction_2(0, eps_node, &txdata, 1 + sizeof(eps_output_set_single_req), 0, 0, eps_delay);
	}
	/* SEUV */
	else if (device == 3) {
		if (stats == ON) {
			/* channel 2 = SEUV 5V */
			eps_switch.channel = 2;
			memcpy(&txdata[1], &eps_switch, sizeof(eps_output_set_single_req));
			i2c_master_transaction_2(0, eps_node, &txdata, 1 + sizeof(eps_output_set_single_req), 0, 0, eps_delay);

			/* channel 5 = SEUV 3.3V */
			eps_switch.channel = 5;
			memcpy(&txdata[1], &eps_switch, sizeof(eps_output_set_single_req));
			i2c_master_transaction_2(0, eps_node, &txdata, 1 + sizeof(eps_output_set_single_req), 0, 0, eps_delay);
		}
		else if (stats == OFF) {
			/* channel 5 = SEUV 3.3V */
			eps_switch.channel = 5;
			memcpy(&txdata[1], &eps_switch, sizeof(eps_output_set_single_req));
			i2c_master_transaction_2(0, eps_node, &txdata, 1 + sizeof(eps_output_set_single_req), 0, 0, eps_delay);

			/* channel 2 = SEUV 5V */
			eps_switch.channel = 2;
			memcpy(&txdata[1], &eps_switch, sizeof(eps_output_set_single_req));
			i2c_master_transaction_2(0, eps_node, &txdata, 1 + sizeof(eps_output_set_single_req), 0, 0, eps_delay);
		}
	}
	/* INMS */
	else if (device == 4) {
		/*      INMS Power GPIO Control        */
		if (stats == ON) {
			io_set(5);
			vTaskDelay(0.3 * delay_time_based);
			io_set(1);
			vTaskDelay(1 * delay_time_based);
			io_clear(5);
			io_clear(1);
		}
		else if (stats == OFF) {
			io_set(6);
			vTaskDelay(0.3 * delay_time_based);
			io_set(0);
			vTaskDelay(1 * delay_time_based);
			io_clear(6);
			io_clear(0);
		}
	}
	/* Interface Board */
	else if (device == 5) {
		if (stats == ON)
			io_set(2);
		else if (stats == OFF)
			io_clear(2);
	}
}
/** Power off all the subsystems */

void power_OFF_ALL()
{
	power_control(1, OFF);
	power_control(2, OFF);
	power_control(3, OFF);
	power_control(4, OFF);
}

/* Helper function */
uint16_t Interface_tmp_get()
{
	uint8_t rx[5];
	uint8_t tx[2];
	tx[0] = 0xF0;	//0d240

	i2c_master_transaction_2(0, interface_node, &tx, 1, 0, 0, interface_delay) ;
	vTaskDelay(0.01 * delay_time_based);
	if (i2c_master_transaction_2(0, interface_node, &tx , 1, &rx, 4, interface_delay) == E_NO_ERR) {
		return (rx[0] << 8) + rx[1];
	}
	else
		return 0;
}
uint16_t Interface_inms_thermistor_get()
{
	uint8_t rx[5];
	uint8_t tx[2];
	tx[0] = 0xD0;	//0d208

	i2c_master_transaction_2(0, interface_node, &tx, 1, 0, 0, interface_delay) ;
	vTaskDelay(0.01 * delay_time_based);
	if (i2c_master_transaction_2(0, interface_node, &tx, 1, &rx, 4, interface_delay) == E_NO_ERR) {
		return (rx[0] << 8) + rx[1];
	}
	else
		return 0;
}
uint16_t Interface_3V3_current_get()
{
	uint8_t rx[5];
	uint8_t tx[2];
	tx[0] = 0x90;	//0d144
	i2c_master_transaction_2(0, interface_node, &tx, 1, 0, 0, interface_delay) ;
	vTaskDelay(0.01 * delay_time_based);
	if (i2c_master_transaction_2(0, interface_node, &tx, 1, &rx, 4, interface_delay) == E_NO_ERR) {
		return (rx[0] << 8) + rx[1];
	}
	else
		return 0;
}
uint16_t Interface_5V_current_get()
{
	uint8_t rx[5];
	uint8_t tx[2];
	tx[0] = 0xB0;	//0x176
	i2c_master_transaction_2(0, interface_node, &tx, 1, 0, 0, interface_delay) ;
	vTaskDelay(0.01 * delay_time_based);
	if (i2c_master_transaction_2(0, interface_node, &tx , 1, &rx, 4, interface_delay) == E_NO_ERR) {
		return (rx[0] << 8) + rx[1];
	}
	else
		return 0;
}

void generate_Error_Report(int type, uint16_t cause_value)
{
	/* 1. Low Battery Condition */
	/* 2. ADCS 3.3V current too high */
	/* 3. ADCS 5V current too high */
	/* 4. GPS current too high */
	/* 5. SEUV 3.3V current too high */
	/* 6. SEUV 5V current too high */
	/* 7. INMS 3.3V current too high */
	/* 8. INMS 5V current too high */
	/* 9. OBC temperature out of range */
	/* 10. COM temperature out of range */
	/* 11. Antenna temperature out of range */
	/* 12. IFB temperature out of range */
	/* 13. ADCS temperature out of range */
	/* 14. ADCS Rate Sensor & Magnetometer temp out of range */
	/* 15. INMS temperature out of range */
	/* 16. EPS temperature out of range */
	/* 17. Battery temperature out of range */

	uint8_t errPacket [10];
	timestamp_t t;
	t.tv_sec = 0;
	t.tv_nsec = 0;
	obc_timesync(&t, 6000);

	if (HK_frame.mode_status_flag == 2 && parameters.first_flight == 1)
		HK_frame.mode_status_flag = 4;

	memcpy(&errPacket[0], &t.tv_sec, 4);
	memcpy(&errPacket[4], &parameters.ErrSerialNumber, 2);
	memcpy(&errPacket[6], &type, 1);
	memcpy(&errPacket[7], &HK_frame.mode_status_flag, 1);
	memcpy(&errPacket[8], &cause_value, 2);

	if (errPacket_write(errPacket) == No_Error) {
		parameters.ErrSerialNumber ++;
		para_w_flash();
	}
	else {
		printf("error write into errpacket\n");
	}
}
int objects_under(const char *dir)
{
	int count = 0;
	DIR *dirp;
	struct dirent *ent;

	dirp = opendir(dir);

	if (dirp) {
		while ((ent = readdir(dirp)) != NULL) {
			if (ent->d_name[0] != '.')
				count++;
		}
		closedir(dirp);
	}

	return count;
}
int report_Collected_Data(char *args, uint16_t *buffer_length)
{

	DIR * dirp;
	struct dirent *ent;
	struct stat stat_buf;
	char buf[128 + 2];
	char * sub;
	char bytebuf[25];
	uint16_t data_number[5];
	/* Get args */
	char path[100];
	if (args == NULL || sscanf(args, "%s", path) < 1)
		return Error;

	dirp = opendir(path);
	if (dirp == NULL) {
		printf("ls: cannot open '%s' for list\r\n", path);
		return Error;
	}

	/* Loop through directories */
	while ((ent = readdir(dirp))) {
		if (ent->d_name[0] == '.')
			continue;
		strncpy(buf, path, 128);
		sub = buf;
		if (path[strlen(path) - 1] != '/')
			sub = strcat(buf, "/");
		sub = strcat(sub, ent->d_name);
		if (ent->d_type & DT_DIR) {
			sprintf(bytebuf, "%d", objects_under(sub));
			strcat(ent->d_name, "/");
		} else {
			stat(sub, &stat_buf);
			bytesize(bytebuf, 25, stat_buf.st_size);
		}

		/* Name */
		// printf("%-15s %6s\r\n", ent->d_name, bytebuf);
		if (strcmp(ent->d_name, "HK_DATA/") == 0) {
			data_number[0] = atoi(bytebuf);
			memcpy(&buffer_length[0], &data_number[0], 2);
		}
		else if (strcmp(ent->d_name, "INM_DATA/") == 0) {
			data_number[1] = atoi(bytebuf);
			memcpy(&buffer_length[1], &data_number[1], 2);
		}
		else if (strcmp(ent->d_name, "SEU_DATA/") == 0) {
			data_number[2] = atoi(bytebuf);
			memcpy(&buffer_length[2], &data_number[2], 2);
		}
		else if (strcmp(ent->d_name, "EOP_DATA/") == 0) {
			data_number[3] = atoi(bytebuf);
			memcpy(&buffer_length[3], &data_number[3], 2);
		}
		else if (strcmp(ent->d_name, "WOD_DATA/") == 0) {
			data_number[4] = atoi(bytebuf);
			memcpy(&buffer_length[4], &data_number[4], 2);
		}
	}
	closedir(dirp);
	return No_Error;
}
void ECEFtoECI(float * Time, int32_t * r_ECEF, int16_t * r_ECI)
{
	float r_ECEF_f[3];
	double T[3][3] = {{0}, {0}, {0}};
	double THETA = 0;
	double r_mutiply_mat[3][3] = {{0}, {0}, {0}};
	double JD, JD_0;
	double GMST, GAST;
	double D, D0, T1, H;
	double T2, EPSILONm, L, dL, OMEGA, dPSI, dEPSILON;
	int i, j;

	Time[2] = Time[2] + (Time[3] / 24) + (Time[4] / 1440) + (Time[5] / 86400);
	JD = (double)(367 * Time[0]) - (int)(7 * (Time[0] + (int)((Time[1] + 9) / 12)) / 4) - (int)(3 * ((int)((int)(Time[0] + (Time[1] - 9) / 7) / 100) + 1) / 4) + (int)(275 * Time[1] / 9) + Time[2] + 1721028.5;
	JD_0 = (double)((int64_t)(JD)) - 0.5;
	D = JD - JD_2000;
	D0 = JD_0 - JD_2000;
	H = (JD - JD_0) * 24;
	T1 = D / 36525;
	GMST = 6.697374558 + 0.06570982441908 * D0 + 1.00273790935 * H + 0.000026 * T1 * T1;
	GMST = (fmod(GMST, 24));

	if (GMST < 0) {
		GMST = 24 + GMST;
	}

	GMST = GMST * 15;
	T2 = (JD - JD_2000) / 36525;
	EPSILONm = 84381448 - 46.8150 * T2 - 0.00059 * T2 * T2 + 0.001813 * T2 * T2 * T2;
	L = 280.4665 + 36000.7698 * T2;
	dL = 218.3165 + 481267.8813 * T2;
	OMEGA = 125.04452 - 1934.136261 * T2;
	dPSI = -17.20 * sin(OMEGA * PI / 180) - 1.32 * sin(2 * L * PI / 180) - 0.23 * sin(2 * dL * PI / 180) + 0.21 * sin(2 * OMEGA * PI / 180);
	dEPSILON = 9.20 * cos(OMEGA * PI / 180) + 0.57 * cos(2 * L * PI / 180) + 0.1 * cos(2 * dL * PI / 180) - 0.09 * cos(2 * OMEGA * PI / 180);
	dPSI = dPSI * (1 / 3600);
	dEPSILON = dEPSILON * (1 / 3600);
	GAST = fmod(GMST + dPSI * cos((EPSILONm + dEPSILON) * PI / 180), 360);
	THETA = GAST;

	T[0][0] = cos(THETA * PI / 180);
	T[1][0] = sin(THETA * PI / 180);
	T[0][1] = -(T[1][0]);
	T[1][1] = T[0][0];
	T[2][2] = 1;

	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			r_ECEF_f[j] = r_ECEF[j] * 0.001;
			r_mutiply_mat[i][j] = T[i][j] * r_ECEF_f[j];
		}
		r_ECI[i] = r_mutiply_mat[i][0] + r_mutiply_mat[i][1] + r_mutiply_mat[i][2]; // Position of ECI
		r_ECI[i] = r_ECI[i] * 4;
		printf("%d\t", r_ECI[i] / 4);
	}
}

/* GMT version of function mktime(), i.e, inverse of gmtime() */
static time_t mktime2(struct tm *tm)
{
	int		i;
	long	days = 0L;
	static int	days_month[] = {
		31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
	};

	/* Compute elapsed days */
	for (i = TIME_T_BASE_YEAR; i < tm->tm_year + 1900; i++) {
		days += (i % 4 == 0) ? 366 : 365;
	}
	for (i = 1; i < tm->tm_mon + 1; i++) {
		days += days_month[i - 1];
		if (i == 2 && tm->tm_year % 4 == 0) days++;
	}
	days += tm->tm_mday - 1;

	/* Calendar value */
	return ((days * 24 + tm->tm_hour) * 60 + tm->tm_min) * 60 + tm->tm_sec;
}

/*------------------------------------------------------------
 * wtime_to_date() - Convert wtime into date and hour
 *------------------------------------------------------------
 *  struct tm wtime_to_date(wt); Date and hour representation
 *    wtime wt; Week number and elapsed time
 *------------------------------------------------------------*/
struct tm wtime_to_date(wtime wt)
{
	time_t	t;

	/* Compute calendar value at given instance */
	t	= (long)wt.week * SECONDS_WEEK + TIME_T_ORIGIN
	      + (long)((wt.sec > 0.0) ? wt.sec + 0.5 : wt.sec - 0.5);

	/* convert calendar value to date and hour representation */
	return *gmtime(&t);
}

/*------------------------------------------------------------
 * date_to_wtime() - Convert date and hour into wtime
 *------------------------------------------------------------
 *  wtime date_to_wtime(tmbuf); Week number and elapsed time
 *    struct tm tmbuf; Date and hour representation
 *------------------------------------------------------------*/
wtime date_to_wtime(struct tm tmbuf)
{
	time_t	t;
	wtime	wt;

	/* Calendar value at given instance */
	t	= mktime2(&tmbuf);

	/* Compute the number of weeks and the remainder */
	wt.week	= (t - TIME_T_ORIGIN) / SECONDS_WEEK;
	wt.sec	= (t - TIME_T_ORIGIN) % SECONDS_WEEK;

	return wt;
}
