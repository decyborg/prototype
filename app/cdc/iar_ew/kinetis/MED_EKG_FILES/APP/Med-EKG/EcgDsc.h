#ifndef _ECG_DSC
#define _ECG_DSC


//#include <hidef.h> /* for EnableInterrupts macro */
//#include "derivative.h" /* include peripheral declarations */
#include "derivative.h"
#include "PublicTypes.h"
#include "IICV1Driver.h"
#include "RealTimerCounter.h"

#define DSC_SLAVE_ADDRESS		(0x02 << 1)
#define DATA_LENGTH_FROM_DSC	68	//bytes to read from the DSC, 

#define	PACKET_ID_POSITION			0
#define CHECKSUM_POSITION			1
#define DATA_START_POSITION			2
									//from 3 to 68 (3+64)
#define DATA_END_POSITION			65
#define DATA_HEART_RATE_POSITION	66

//1 packet id
//1 checksum
//64 data
//2 HR

//68 total bytes


extern UINT8 DataFromDsc[];
extern UINT8 EcgDsc_HeartRate;

extern const pFunc_t EcgDsc_Events[];	//the easy way

void EcgDsc_Init(void);
UINT8 EcgDsc_DiagnosticModeStartMeasurement(void);
UINT8 EcgDsc_HeartRateStartMeasurement(void);
void EcgDsc_DiagnosticModeStopMeasurement(void);
void EcgDsc_PeriodicTask(void);
	  


#endif //_ECG_DSC