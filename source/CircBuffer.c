/*
 * CircBuffer.c
 *
 *  Created on: Apr 11, 2020
 *      Author: nitis
 */
#include <stdint.h>
#include <stdlib.h>
#include "CircBuffer.h"
#include "logger.h"
uint8_t Size(CircBuffer_t *buf)
{
	return buf->length;
	Log_string("Length is ", NEWLINE);
	Log_integer(buf->length);
}

CircBuffer_t* CircBufferCreate(void)
{
	return (CircBuffer_t*) malloc(sizeof(CircBuffer_t));
	Log_string("Circular Buffer is created ", NEWLINE);
}

CBufferReturn_t CircularBufferInit(CircBuffer_t * buf, uint8_t size)
{
	/* Allocate memory */
	buf->circbuffer_start = (uint16_t*) malloc(sizeof(uint16_t) * size);
	Log_string("Memory allocated", NEWLINE);
	/* Make sure memory is valid */
	if(!buf->circbuffer_start)
	{
		return FAIL;
	}

	/* Set buffer values */
	buf->head = buf->circbuffer_start;
	buf->tail = buf->circbuffer_start;
	buf->capacity = size;
	buf->length = 0;


	return SUCCESS;
	Log_string("Initialised ", NEWLINE);
}

CBufferReturn_t CircularBufferDestroy(CircBuffer_t * buf)
{
	free(buf->circbuffer_start);

	free(buf);

	return SUCCESS;
	Log_string("Freed", NEWLINE);
}

//uint16_t CBLengthData(CircBuffer_t *cb)
//{
//	return ((*cb->head – *cb->tail) & (*cb->capacity -1));// uses power of two assumption to
//													// determine length
//}
CBufferReturn_t	Initialized(CircBuffer_t * buf)
{
	/* Ensure the buffer pointers are all valid */
	if( buf->circbuffer_start && buf->head && buf->tail)
	{
		return SUCCESS;
	}

	else return FAIL;
}

CBufferReturn_t	CheckIfEmpty(CircBuffer_t * buf)
{
	if((buf->length == 0) && (buf->head == buf->tail))
		{
			return EMPTY;
		}
	else return SUCCESS;
}

CBufferReturn_t	CheckIfValid(CircBuffer_t * buf)
{
	if(buf->circbuffer_start){ return SUCCESS;Log_string("Check valid is done ", NEWLINE);}
	else return FAIL;
}




CBufferReturn_t CheckIfFull(CircBuffer_t * buf)
{
	if((buf->capacity==buf->length) && (buf->head==buf->tail))
	{
		return EMPTY;
		Log_string("Buffer empty ", NEWLINE);
	}
	else return SUCCESS;
	Log_string("Check if full ...", NEWLINE);
}

CBufferReturn_t	CBAdd(CircBuffer_t * buf, uint16_t c, uint8_t * flag)
{
	CBufferReturn_t ret;

	/* Check that the buffer is not full */
	if(CheckIfFull(buf) == FULL)
	{
		//Logger statement here
		*flag=1;
		ret = FULL;

//		if(REALLOCATE_BUFFER && (buf->numReallocs < 5))
//		{
//			ret = CircBufRealloc(buf);
//		}

		//else return ret;
	}

	//Critical Section starts here

	/* Add element by placing into current head position and moving head forward 1 or wrapping */
	*(buf->head) = c;
	(buf->head)++;
	(buf->length)++;


	uint16_t* bufend = (uint16_t*) buf->circbuffer_start + (sizeof(uint16_t) * buf->capacity);

	/* Check if it needs to be wrapped to the beginning */
	if(buf->head == bufend)
	{
		Log_string("Wrap performed", NEWLINE);
		buf->head = buf->circbuffer_start;
	}

	//End of critical section
	return SUCCESS;
	Log_string("Element added successfully ", NEWLINE);
}

CBufferReturn_t CBRead(CircBuffer_t * buf, uint16_t *out)
{
	CBufferReturn_t ret;
	/*Check if not empty*/
	if(CheckIfEmpty(buf)==EMPTY)
	{
		//logger statement
		ret=EMPTY;
	}
	//Critical section starts
	*out= *(buf->tail);
	(buf->head)++;
	(buf->length)--;

	uint16_t* bufend = (uint16_t*) buf->circbuffer_start + (sizeof(uint16_t) * buf->capacity);

		/* Check if it needs to be wrapped to the beginning */
		if(buf->tail == bufend)
		{
			Log_string("Wrap performed ", NEWLINE);
			buf->tail = buf->circbuffer_start;
		}

		//Critical section ends
		Log_string("Read done ", NEWLINE);
		return SUCCESS;
}







