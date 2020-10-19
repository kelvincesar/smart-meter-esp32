// Inclusion guard, to prevent multiple includes of the same header
#ifndef GOERTZEL_H
#define GOERTZEL_H

#include <stdint.h>                         // C standard library
#include <math.h>
#include "buffer.h"

#define PI_VALUE 3.14159265359
#define PI_2	 1.570796327


// Structure with values from goertzel algorithm;
typedef struct {
	float DFT_m;
	float DFT_r;
	float DFT_i;
	float DFT_arg;
} GoertzelState;


// Functions
int goertzel (Buffer *buf, GoertzelState *goertz, uint16_t target_freq, uint16_t sample_rate);
#endif