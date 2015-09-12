/****************************************************************************
*                            FIR for Kinetis
*****************************************************************************
* @file: Kinetis_FIR.c
* 
* @author : Santiago Lopez [SL]  Freescale Americas
*
* @version: 1.0
*
* @description: Program to implement Finite Impulse Response Filters on Kinetis
******************************************************************************/

#include "Kinetis_FIR.h"

/******************************************************************************
*                                  Globals
*******************************************************************************/

/*Coeficients for band pass filter 0.1Hz - 150Hz*/

int gi32FIR_Coeficients1 [] = {
0,
-63,
-278,
-552,
-614,
-68,
1445,
3997,
7119,
9635,
9756,
5509,
-4490,
-20096,
-38816,
-55509,
-62878,
-52858,
-18704,
42740,
129254,
232586,
339327,
433272,
498780,
524286,
505032,
444227,
352326,
244605,
137715,
46149,
-20474,
-58683,
-70839,
-63503,
-45126,
-23764,
-5408,
6767,
12246,
12390,
9412,
5462,
2057,
-102,
-998,
-1015,
-641,
-257,
-46,
0
};

int gi32ADC_Buffer[FILTER_ORDER] = {0};

unsigned long gu16FIR_Data[FIR_DATA_TO_SEND] = {0};

unsigned char gu8FilterDataReady=0;

/*****************************************************************************
*                                 Functions
*****************************************************************************/

void vfnFilter_Periodic_Task (void)  
{
 static unsigned char u8FIR_ActualSample = FILTER_ORDER-1;
 static unsigned char u8FIR_Data_To_Send_Index = FIR_DATA_TO_SEND;
 volatile unsigned int u32FilteredValue;
 
 /* Put your data to be filtered here
    TestBuffer[u8FIR_ActualSample] += INPUT_OFFSET;
 */
   gi32ADC_Buffer[u8FIR_ActualSample] = (int)(ADC1_Read16b(0));
   
#ifndef BYPASS
   gi32ADC_Buffer[u8FIR_ActualSample] += INPUT_OFFSET; 
   gi32ADC_Buffer[u8FIR_ActualSample] /= INPUT_DIVISOR;
#endif
 
 /* Execute the filter 
 u32FilteredValue = u32Filter_Execution (&TestBuffer[u8FIR_ActualSample],&TestBuffer[FILTER_ORDER],&gi32FIR_Coeficients1[0]); 
 */
   
 u32FilteredValue = i32Filter_Execution (&gi32ADC_Buffer[u8FIR_ActualSample],&gi32ADC_Buffer[FILTER_ORDER],&gi32FIR_Coeficients1[0]);
 
 u32FilteredValue/=OUTPUT_DIVISOR;  //This is the output data for the actual sample
 
 #ifdef BYPASS
 u32FilteredValue = gi32ADC_Buffer[u8FIR_ActualSample];   
 #endif   
 
 if(u8FIR_ActualSample)
 { 
  u8FIR_ActualSample--; 
 }
 else
 {
  u8FIR_ActualSample += (unsigned char)(FILTER_ORDER-1); 
 }
 
#if (!((defined BYPASS)||(defined DAC0_TEST))) 
 /* Signal treatment for MED-EKG */

 if(u32FilteredValue > 0xFFFF)
    u32FilteredValue = 0xFFFF;
 
 u32FilteredValue = (unsigned long) u32FilteredValue;
 
 HR_periodic_task ((unsigned long) u32FilteredValue); //Calculate HR before sign the signal 
 
 /*Signal Signing proccess*/
#if 0 //0 Using Java GUI, 1 Using Windows GUI
 if (u32FilteredValue > 0x7FFF)
 {
  u32FilteredValue-=0x8000;
 }
 else
 {
   /* Sign change */
   u32FilteredValue |= 0x00008000; //Add sign bit
 }
#endif
 
#endif
 
 gu16FIR_Data[u8FIR_Data_To_Send_Index-1] = (unsigned long)(u32FilteredValue); //send data to buffer
 u8FIR_Data_To_Send_Index--;
 
 if(!u8FIR_Data_To_Send_Index)
 {
  gu8FilterDataReady=TRUE;
  u8FIR_Data_To_Send_Index=FIR_DATA_TO_SEND;
 }
 
 
#if (defined DAC0_TEST)
 u32FilteredValue /=16; 
 
 if (u32FilteredValue>4095)
     u32FilteredValue=4095;
 
 DACDAT0 = u32FilteredValue; 
#endif
 
}

/*********************************************************************************************************************************/

/*********************************************************************************************************************************/

int i32Filter_Execution (int *pi32ActualSample, int *pi32BufferLimit, int *pi32Coefptr)
{
 volatile unsigned char u8Filter_Order = FILTER_ORDER;
 volatile int i32Accumulate = 0;
  
#if (defined CODE)
 volatile int i32Multiply = 0;
#endif
 
 while (u8Filter_Order--)
 {
   
#if (defined CODE)
   i32Multiply = (int) ((*pi32ActualSample++)*(*pi32Coefptr++));
   i32Accumulate += i32Multiply;
#elif (defined MAC)
   i32Accumulate = __smlal(i32Accumulate,(*pi32ActualSample++),(*pi32Coefptr++)); //Assembly routine for Signed Multiply and Accumulate DSP instruction 
#else
#error "Invalid Definition"
#endif
   
   if(pi32ActualSample == pi32BufferLimit)
      pi32ActualSample -= FILTER_ORDER;
   
 } 
   
    /* Uncoment this to remove sign */ 
   if(i32Accumulate & (int)(0x80000000))
   return(0); 
   
   return (unsigned int)i32Accumulate;   
 }

/**************************************************************************************************************************************/
