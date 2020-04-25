/*
 * logger.h
 *
 *  Created on: Feb 21, 2020
 *      Author: Kristina Brunsgaard
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#include "logger.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

extern int enable;

typedef enum newline
{	NO_NEWLINE = 0, //log_string print newline
	NEWLINE // log_string don't print newline
} newline;

void Log_enable();
void Log_disable();
int Log_status();
void Log_data(uint8_t *bytes, int length);
void Log_string(char *string, int newLine);
void Log_integer(int integer);
void Log_pointer(uint8_t *pointer);


#endif /* LOGGER_H_ */
