#ifndef _DAC
#define _DAC


#include "derivative.h"
#include "PublicTypes.h"

typedef union
{
 unsigned long Word;
 struct
 {
  unsigned char DACDATx_L;
  unsigned char DACDATx_H;
 }Bytes; 
}DACDATx_FULL_REG;

__no_init DACDATx_FULL_REG DACDATx @0x400CC000;

#define DACDAT0    DACDATx.Word
 

void DAC12_Vin_SWtrig(void) ;





#endif //_ADC