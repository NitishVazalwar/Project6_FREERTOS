/*
 * logger.c
 *
 *  Created on: Feb 21, 2020
 *      Author: Kristina Brunsgaard
 */
#include "logger.h"


int enable;

void Log_enable(){ //begin printing log messages when called
	printf("\nLog Messages Enabled");
	enable = 1;
}
void Log_disable() { //ignore any log messages until re-enabled
	printf("\nLog Messages Disabled");
	enable = 0;
}

int Log_status(){ //returns a flag to indicate whether the logger is enabled or disabled
	return enable;
}

void Log_data(uint8_t *bytes, int length){ //display in hexadecimal an address and contents of a memory location, arguments are a pointer to a sequence of bytes and a specified length (in dword)
	int i;
	int space = 0;
	if (enable){
		if (length == 0) {
			printf("Log Error: 0 Bytes Entered\n");
		} else {
			printf("\nLOG: address: %08X\n     memory: ", bytes);
			for (i = 0; i < length; i++) {
				if (space == 4) {
					printf(" ");
					space = 0;
				}
				printf("%02X", bytes[i]);
				space++;
			}
			printf("\n");
		}
	}
}

void Log_string(char *string, int newLine){ //display a string
	if (enable && newLine) {
		printf("\nLOG: %s\n", string);
	} else if ((enable == 1) && (newLine == 0)) {
		printf("\nLOG: %s ", string);
	}
}

void Log_integer(int integer) { //display an integer
	if (enable) {
		printf("%d\n", integer);
	}
}

void Log_pointer(uint8_t *pointer) { //display an integer
	if (enable) {
		printf("%08X\n", pointer);
	}
}

