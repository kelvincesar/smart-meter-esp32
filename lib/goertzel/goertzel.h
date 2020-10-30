// Inclusion guard, to prevent multiple includes of the same header
#ifndef GOERTZEL_H
#define GOERTZEL_H

#include <stdint.h>                         // C standard library
#include <math.h>
#include "data_buffers.h"
//#include <stdio.h>
#define PI_VALUE 3.141592653
#define PI_2	 1.570796327


// Structure with values from goertzel algorithm;
typedef struct {
	float DFT_m;
	float DFT_r;
	float DFT_i;
	float DFT_arg;
} GoertzelState;


// Functions
int goertzel (Buffer *buf, GoertzelState *goertz, uint16_t target_freq, uint16_t sample_rate, uint8_t config);
float fast_sqrt(float x);
#endif