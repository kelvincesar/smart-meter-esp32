// Inclusion guard, to prevent multiple includes of the same header
#ifndef MOV_AVG_H
#define MOV_AVG_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Definitions ---------------------------------------------------------------*/
#define WindowLength 10

/* TypeDefs ------------------------------------------------------------------*/
typedef struct{
	uint16_t History[WindowLength]; /*Array to store values of filter window*/
	uint32_t Sum;	/* Sum of filter window's elements*/
	uint16_t WindowPointer; /* Pointer to the first element of window*/
}FilterTypeDef;

/* Function prototypes -------------------------------------------------------*/
void Moving_Average_Init(FilterTypeDef* filter_struct);
uint16_t Moving_Average_Compute(uint16_t raw_data, FilterTypeDef* filter_struct);

#endif
