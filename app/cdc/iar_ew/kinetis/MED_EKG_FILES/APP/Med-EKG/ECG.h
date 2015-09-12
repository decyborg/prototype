#ifndef _ECG
#define _ECG

//the following defines are mutually exclusive, if none of them is defined, ECG will function with MCU
//#define SEND_SINE_WAVE		//uncomment this to send a dummy sine-wave
//#define ECG_DSC				//uncomment this to use the DSC
#define ECG_FIR                         //uncomment this to use the FIR Filter 

//#include <hidef.h> /* for EnableInterrupts macro */
//#include "derivative.h" /* include peripheral declarations */
#include "derivative.h"
#include "PublicTypes.h"
#include "ADC.h"
#include "AverageFilter.h"
#include "RealTimerCounter.h"
#include "DAC.h"

#define ECG_SAMPLING_PERIOD				2   // in ms

#define	ADC_CHANNEL_FEEDBACK_SIGNAL		0	//baseline (instrumentation amplification output) signal
#define	ADC_CHANNEL_ECG_SIGNAL			0	//ecg signal channel

#define ECG_ARRAY_LENGTH				15
#define OLDEST_ELEMENT				ECG_ARRAY_LENGTH-1
#define NEWEST_ELEMENT				0


#define ECG_DATA_BUFFER_LENGTH	64

#ifdef ECG_DSC
#define HR_SLOPE_THRESHOLD     2200
#else
#define HR_SLOPE_THRESHOLD     1000
#endif

#define MAX_TIME_WITHOUT_PULSES		(5000/ECG_SAMPLING_PERIOD)	//time in ms/ECG_SAMPLING_PERIOD

extern UINT8 Ecg_HeartRate;
extern UINT8 EcgDataBuffer[];

void Ecg_PeriodicTask(void);


//diagnostic mode
void Ecg_DiagnosticModeStopMeasurement(void);
UINT8 Ecg_DiagnosticModeStartMeasurement(void);

void Ecg_Init(void);

extern const pFunc_t Ecg_Events[];	//the easy way


typedef enum
{
	EVENT_ECG_NONE,
	EVENT_ECG_HEART_RATE_MEASUREMENT_COMPLETE_OK,
	EVENT_ECG_HEART_RATE_MEASUREMENT_ERROR,
	EVENT_ECG_HEART_BEAT_OCCURRED,
	EVENT_ECG_DIAGNOSTIC_MODE_NEW_DATA_READY	
} Ecg_Event_e;


#endif //_ECG