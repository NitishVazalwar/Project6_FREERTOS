/*
 * math_functions.c
 *
 *  Created on: 23-Apr-2020
 *      Author: Hp
 */

#include "math_functions.h"

// Source is Making Embedded Systems by Elcia White pg. 253
uint16_t GetVariance(uint16_t *samples,uint16_t numSamples, uint16_t mean ){
	uint32_t sumSquares=0;
	uint32_t temp;
	uint32_t i=numSamples;

	while(i--){
		temp= *samples -mean;
		samples++;
		sumSquares += temp*temp;
	}

	return (sumSquares/(numSamples-1));
}


