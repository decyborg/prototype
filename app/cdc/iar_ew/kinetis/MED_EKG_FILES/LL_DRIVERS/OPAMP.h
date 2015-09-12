#ifndef _OPAMP
#define _OPAMP


#include "derivative.h" /* include peripheral declarations */
#include "PublicTypes.h"

        #define Gain2       0x02	
        #define Gain3       0x01
        #define Gain4       0x03
        #define Gain5       0x08
	#define Gain6       0x0A
        #define Gain7       0x09
        #define Gain8       0x0B
	#define Gain9       0x04
        #define Gain10      0x06
        #define Gain11      0x05
        #define Gain12      0x07
	#define Gain13      0x0C
        #define Gain14      0x0E
        #define Gain15      0x0D
        #define Gain16      0x0F
	#define Gain17      0x1D
        #define Gain18      0x3F
	
	

        #define ENABLE_OPAMPS     SIM_SCGC1 |= SIM_SCGC1_OPAMP_MASK //Gives clock to opamps
        #define ENABLE_TRIAMPS    SIM_SCGC1 |= SIM_SCGC1_TRIAMP_MASK //Gives clock to TRIAMPS

void opamp1_gp_mode(void) ;
void opamp2_gp_mode(void) ;

void TRIAMP1C0_HighMode(void);
void TRIAMP2C0_HighMode(void);

void opamp1_noninverting_mode(unsigned char gain);
void opamp2_noninverting_mode(unsigned char gain);


#endif //_ADC