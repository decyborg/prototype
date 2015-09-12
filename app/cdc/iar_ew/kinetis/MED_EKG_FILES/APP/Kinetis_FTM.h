/****************************************************************************
*                           FTM Driver for Kinetis
*****************************************************************************
* @file: Kinetis_FTM.h
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

#ifndef _KINETIS_FTM_H_
#define _KINETIS_FTM_H_

#include "derivative.h"

/*****************************************************************************
*                                  Globals
*****************************************************************************/

extern unsigned char gu8FTM0_TOF;

/*****************************************************************************
*                          Function Definitions  
*****************************************************************************/

extern  void FTM0_Init (void);
void FTM0_ISR (void);


#endif