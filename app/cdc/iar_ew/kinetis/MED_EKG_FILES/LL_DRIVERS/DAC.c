/******************************************************************************/
/*                    DAC Initialization for Kinetis K50
/*
/*                      This application is using DAC0
/******************************************************************************/

#include "DAC.h"

void DAC12_Vin_SWtrig(void)  
{  
 SIM_SCGC2 |= SIM_SCGC2_DAC0_MASK;  //Gives clock to DAC0
  
 DAC0_DAT0H = 0x07;
 DAC0_DAT0L = 0xC1;
 /*
   DAC0_DAT0 = 0x07C1 for 1.6v according with the formula Vout = Vin*(1+DACDAT0[11:0])/4096
   *!See Reference Manual for More information
 */
 
 DAC0_C0 |= (DAC_C0_DACEN_MASK | DAC_C0_DACRFS_MASK);
 /*                  |                    |
                     |                     ----------- VDDA as DAC Ref Voltaje (Vin) 3.3v
                      -------------------------------- DAC0 is enabled
 
    *!See Reference Manual for more information
 */  
}
