#ifndef _ADC
#define _ADC

#include "derivative.h" /* include peripheral declarations */
#include "PublicTypes.h"





void ADC_Init8b(UINT16 activeChannelsMask);
void ADC_Init12b(UINT16 activeChannelsMask);
void ADC0_Init16b(void);
void ADC1_Init16b(void);


UINT8 ADC_Read8b(UINT8 channelNumber);
UINT16 ADC_Read12b(UINT8 channelNumber);
UINT16 ADC0_Read16b(UINT8 channelNumber);
UINT16 ADC1_Read16b(UINT8 channelNumber);






#endif //_ADC