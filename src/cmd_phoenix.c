/*
 * cmd_phoenix.c
 *
 *  Created on: 25/12/2015
 *      Author: Kenny Huang
 */

#include <inttypes.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <malloc.h>

#include <nanomind.h>

#include <util/console.h>
#include <util/timestamp.h>
#include <util/vermagic.h>
#include <util/log.h>
#include <csp/csp_endian.h>


int test(struct command_context *ctx) { 
	printf("test!!!!\n");
	return CMD_ERROR_NONE; 
}


command_t __root_command ph_commands[] = {
	{
		.name = "ph",
		.help = "client: PHOENIX",
		.handler = test,		
	}
};

void cmd_ph_setup(void) {
	command_register(ph_commands);
}
