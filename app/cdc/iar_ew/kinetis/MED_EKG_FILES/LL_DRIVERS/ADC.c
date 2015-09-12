#include "ADC.h"

//****************************************************************************/
//                               ADC DRIVERS
//                             FOR KINETIS K50
//****************************************************************************/

void ADC_Init12b(UINT16 activeChannelsMask)
{
/*
    ADCCFG_MODE = 0b01; //12 bit conversion mode
  	ADCSC2 = 0x00;
  	ADCSC1 = ADCSC1_ADCH_MASK;	//disable channels
    APCTL1 = activeChannelsMask & 0x00FF;
    APCTL2 = activeChannelsMask >> 8;
*/
}


void ADC_Init8b(UINT16 activeChannelsMask)
{
/*    ADCCFG_MODE = 0b00; //8 bit conversion mode
  	ADCSC2 = 0x00;
  	ADCSC1 = ADCSC1_ADCH_MASK;
    APCTL1 = activeChannelsMask & 0x00FF;
    APCTL2 = activeChannelsMask >> 8;
*/
}


void ADC0_Init16b(void)
{
        SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK; //Gives clock to ADC0
	
        ADC0_CFG1 = (ADC_CFG1_MODE(3) | ADC_CFG1_ADIV(1)); //16bit mode
	ADC0_SC1A = ADC_SC1_ADCH(31); //disable module

        /* Configure ADC pins as ADC Inputs manually */
        
        /*In Kinetis the ADC pins are ADC inputs by default */
        
}

void ADC1_Init16b(void)
{
        SIM_SCGC3 |= SIM_SCGC3_ADC1_MASK; //Gives clock to ADC0
	
        ADC1_CFG1 = (ADC_CFG1_MODE(3) | ADC_CFG1_ADIV(1) | ADC_CFG1_ADICLK(1)| ADC_CFG1_ADLSMP_MASK); //16bit mode
	ADC1_SC1A = ADC_SC1_ADCH(31); //disable module

        /* Configure ADC pins as ADC Inputs manually */
        
        /*In Kinetis the ADC pins are ADC inputs by default */
               
}


UINT16 ADC0_Read16b(UINT8 channelNumber)
{
	ADC0_SC1A = channelNumber & ADC_SC1_ADCH_MASK;	                /* Write to ADCSC1 to start conversion */
	while ((ADC0_SC2 & 0x80)); 				  	/* Wait if the conversion is in progress */
	while (!(ADC0_SC1A & 0x80)); 					/* Wait until the conversion is complete */
	return ADC0_RA; 
}

UINT16 ADC1_Read16b(UINT8 channelNumber)
{
	ADC1_SC1A = channelNumber & ADC_SC1_ADCH_MASK;	                /* Write to ADCSC1 to start conversion */
	while ((ADC1_SC2 & 0x80)); 				  	/* Wait if the conversion is in progress */
	while (!(ADC1_SC1A & 0x80)); 					/* Wait until the conversion is complete */
	return ADC1_RA; 
}


UINT16 ADC_Read12b(UINT8 channelNumber)
{
//	ADCSC1 = channelNumber & ADCSC1_ADCH_MASK;	/* Write to ADCSC1 to start conversion */
//	while (ADCSC2_ADACT == 1); 				  	/* Wait if the conversion is in progress */
//	while (ADCSC1_COCO == 0); 					/* Wait until the conversion is complete */
//	return ADCR; /* */
}


UINT8 ADC_Read8b(UINT8 channelNumber)
{
/*
	ADCSC1 = channelNumber & ADCSC1_ADCH_MASK;	// Write to ADCSC1 to start conversion
	while (ADCSC2_ADACT == 1); 				  	// Wait if the conversion is in progress
	while (ADCSC1_COCO == 0); 					// Wait until the conversion is complete 
	return ADCRL;
	*/
}
