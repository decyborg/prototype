/****************************************************************************
*                           FTM Driver for Kinetis
*****************************************************************************
* @file: Kinetis_FTM.c
* 
* @author : Santiago Lopez [SL]  Freescale Americas
*
* @version: 1.0 for IAR
*
* @date: 13/01/11
*
* @description: Drivers for Kinetis Flex Timer
*******************************************************************************
* @changes:
*
*    Date       Author         Version               Description of Change
*
*  13/01/11       SL             1.0                    Initial Version
******************************************************************************/


#include "Kinetis_FTM.h"

/*****************************************************************************
*                                Definitions
*****************************************************************************/

 

#define FTM0_MOD_VALUE  782//1500  //1mS using 48Mhz in System Clock with a prescaler of 64 

/*****************************************************************************
*                                  Globals
*****************************************************************************/

unsigned char gu8FTM0_TOF = 0;

/*****************************************************************************
*                                 Functions  
*****************************************************************************/

void FTM0_Init (void)
{ 
  SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK; //Gives clock to FTM0
  
  FTM0_MOD = FTM0_MOD_VALUE;
  FTM0_SC = (FTM_SC_CLKS(1) | FTM_SC_TOIE_MASK | FTM_SC_PS(6));   //Runs FTM System Clock Prescaler = 64
  
  NVICICER1|=(1<<30);                     //Clear any pending interrupts on FTM0
  NVICISER1|=(1<<30);                     //Enable interrupts from FTM0 module  
}

void FTM0_ISR (void)
{
 FTM0_SC &= ~FTM_SC_TOF_MASK;
 
 FTM0_MOD=FTM0_MOD_VALUE;
 gu8FTM0_TOF = 1; 
}
