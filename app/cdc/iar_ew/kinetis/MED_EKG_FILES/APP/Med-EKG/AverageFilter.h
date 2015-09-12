#ifndef _AVERAGE_FILTER
#define _AVERAGE_FILTER


//#include <hidef.h> /* for EnableInterrupts macro */
//#include "derivative.h" /* include peripheral declarations */
#include "derivative.h"
#include "PublicTypes.h"

#define NUMBER_OF_SAMPLES 	5

typedef struct
{
	UINT32 sum;
	UINT16 InputBuffer[NUMBER_OF_SAMPLES];
	UINT8 ActualIndex;
	UINT16 ValueThatGoesOut;
	UINT16 ValueThatGoesIn;
	UINT16 Result;
} MovingAverage_uint16_t;

typedef struct
{
	UINT16 sum;
	UINT8 InputBuffer[NUMBER_OF_SAMPLES];
	UINT8 ActualIndex;
	UINT8 ValueThatGoesOut;
	UINT8 ValueThatGoesIn;
	UINT8 Result;
} MovingAverage_uint8_t;



void MovingAverage_PushNewValue8b(MovingAverage_uint8_t *averageStruct,UINT8 newValue);
void MovingAverage_PushNewValue16b(MovingAverage_uint16_t *averageStruct,UINT16 newValue);
void MovingAverage_Clear8b(MovingAverage_uint8_t *averageStruct);
void MovingAverage_Clear16b(MovingAverage_uint16_t *averageStruct);


#endif //_AVERAGE_FILTER