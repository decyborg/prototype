#include "AverageFilter.h"



void MovingAverage_PushNewValue16b(MovingAverage_uint16_t *averageStruct,UINT16 newValue)
{
	averageStruct->ValueThatGoesIn = newValue;
	averageStruct->ValueThatGoesOut = averageStruct->InputBuffer[averageStruct->ActualIndex];
	
	averageStruct->InputBuffer[averageStruct->ActualIndex] = averageStruct->ValueThatGoesIn;
	
	if (averageStruct->ActualIndex == (NUMBER_OF_SAMPLES-1))
	{
		averageStruct->ActualIndex = 0;
	}
	else
	{
		averageStruct->ActualIndex++;
	}
	averageStruct->sum = averageStruct->sum + averageStruct->ValueThatGoesIn - averageStruct->ValueThatGoesOut;

	averageStruct->Result = (UINT16)(averageStruct->sum/NUMBER_OF_SAMPLES);
}



void MovingAverage_PushNewValue8b(MovingAverage_uint8_t *averageStruct,UINT8 newValue)
{
	averageStruct->ValueThatGoesIn = newValue;
	averageStruct->ValueThatGoesOut = averageStruct->InputBuffer[averageStruct->ActualIndex];
	
	averageStruct->InputBuffer[averageStruct->ActualIndex] = averageStruct->ValueThatGoesIn;
	
	if (averageStruct->ActualIndex == (NUMBER_OF_SAMPLES-1))
	{
		averageStruct->ActualIndex = 0;
	}
	else
	{
		averageStruct->ActualIndex++;
	}
	averageStruct->sum = averageStruct->sum + averageStruct->ValueThatGoesIn - averageStruct->ValueThatGoesOut;

	averageStruct->Result = (UINT8)(averageStruct->sum/NUMBER_OF_SAMPLES);
}


void MovingAverage_Clear8b(MovingAverage_uint8_t *averageStruct)
{
	UINT8 i;
	
	for (i=0;i<NUMBER_OF_SAMPLES; i++)
	{
		averageStruct->InputBuffer[i] = 0;		
	}
	
	averageStruct->ActualIndex = 0;
	averageStruct->Result = 0;
	averageStruct->sum = 0;
	averageStruct->ValueThatGoesIn = 0;
	averageStruct->ValueThatGoesOut = 0;
}

void MovingAverage_Clear16b(MovingAverage_uint16_t *averageStruct)
{
	UINT8 i;
	
	for (i=0;i<NUMBER_OF_SAMPLES; i++)
	{
		averageStruct->InputBuffer[i] = 0;		
	}
	
	averageStruct->ActualIndex = 0;
	averageStruct->Result = 0;
	averageStruct->sum = 0;
	averageStruct->ValueThatGoesIn = 0;
	averageStruct->ValueThatGoesOut = 0;
}