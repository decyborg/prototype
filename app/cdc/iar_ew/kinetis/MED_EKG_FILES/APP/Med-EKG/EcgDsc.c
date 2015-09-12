#include "EcgDsc.h"

#define ECG_READ_DSC_INTERVAL		8	//ms
#define MAX_COMMUNICATION_ATTEMPTS	5	//if DSC doesn't respond in this number of attampts, report an error


typedef enum
{
	EVENT_ECG_DSC_NONE,
	EVENT_ECG_DSC_HEART_RATE_MEASUREMENT_COMPLETE_OK,
	EVENT_ECG_DSC_HEART_RATE_MEASUREMENT_ERROR,
	EVENT_ECG_DSC_HEART_BEAT_OCCURRED,
	EVENT_ECG_DSC_DIAGNOSTIC_MODE_NEW_DATA_READY,
	EVENT_IIC_COMMUNICATION_ERROR	
} EcgDsc_Event_e;


typedef enum
{
	STATE_IDLE,						//StateIdle,	
	STATE_WRITING_START_COMMAND,	//StateWritingStartCommand,
	STATE_WRITING_STOP_COMMAND,		//StateWritingStopCommand,
	STATE_READING_DATA,				//StateReadingData,
	STATE_READING_HEART_RATE		//StateReadingHeartRate	
} EcgDscStates_e;

static UINT8 EcgDscActualEvent = EVENT_ECG_DSC_NONE;
static TIMER_OBJECT TimerEcgDscReadDsc;
static UINT8 TimerEcgDscReadDscIndex;
static UINT8 EcgDscIsHeartRateMode;
static UINT8 EcgDscActualState;
static UINT8 EcgDscActualSubState;

static TIMER_OBJECT TimerWaitForDsc;
static UINT8 TimerWaitForDscIndex;
static UINT8 DscIsReady = FALSE;


#define TIMER_WAIT_FOR_DSC_INTERVAL			1	//ms



static UINT8 CommunicationAttempts;
UINT8 DataFromDsc[DATA_LENGTH_FROM_DSC];
UINT8 EcgDsc_HeartRate;
UINT8 ErrorCounter;

const UINT8 CmdEcgDiagnosticModeStartMeasurement = 0x00;
const UINT8 CmdEcgDiagnosticModeStopMeasurement = 0x01;

const UINT8 CmdEcgReadHeartRate = 0x03;
const UINT8 CmdEcgIsNewDataAvailable = 0x04;		//DSC will answer with:
													//0 if no data is available
													//1 if the buffer 1 is available
													//2 if the buffer 2 is available

const UINT8 CmdEcgDiagnosticModeReadData  = 0x20;	//DSC will send last available buffer
const UINT8 CmdEcgDiagnosticModeReadData1 = 0x21;	//DSC will send buffer1
const UINT8 CmdEcgDiagnosticModeReadData2 = 0x22;	//DSC will send buffer2


#define NO_DATA_AVAILABLE		0x00
#define NEW_DATA_AVAILABLE_1	0x01
#define NEW_DATA_AVAILABLE_2	0x02

/* Private functions */
//states
static void StateIdle(void);
static void StateWritingStartCommand(void);
static void StateWritingStopCommand(void);
static void StateReadingData(void);
static void StateReadingHeartRate(void);

//timer events
static void TimerEcgDscReadDsc_Event(void);
static void TimerWaitForDsc_Event(void);

//functions
void static CheckWriteStatus(void);			/**< Checks if the DSC were written correctly */
void static CheckReadStatus(void);			/**< Checks if the DSC were read correctly */


/* Main state machine */
void (*const EcgDscStateMachine[]) (void) =
{
	StateIdle,	
	StateWritingStartCommand,
	StateWritingStopCommand,
	StateReadingData,
	StateReadingHeartRate
}; 




static void TimerEcgDscReadDsc_Event(void)
{
	if (EcgDscIsHeartRateMode)
	{
		EcgDscActualState = STATE_READING_HEART_RATE;		
	}
	else
	{		
		EcgDscActualState = STATE_READING_DATA;	
	}
}



void static CheckWriteStatus()
{
	if (IIC_IS_WRITE_OK())
	{
		IIC_CLEAR_FLAG_WRITE_OK();				
		EcgDscActualSubState++;
	}
	else
	{
		EcgDscActualSubState--;	/* If the data was not written, go back to prev state */
		CommunicationAttempts++;	
	}	
}



void static CheckReadStatus()
{
	if (IIC_IS_READ_OK())
	{
		IIC_CLEAR_FLAG_READ_OK();
		EcgDscActualSubState++;
	}
	else
	{
		EcgDscActualSubState--;	/* If data was not read, go back to prev substate */
		CommunicationAttempts++;	
	}	
}


/*********************
 	states
*********************/
static void StateIdle(void)
{
	
}

static void StateWritingStartCommand(void)
{
	if (!IIC_IS_BUSY())
	{
		switch (EcgDscActualSubState)
		{
			case 0:
				//write command to start measurement
				vfnIICV1Write(DSC_SLAVE_ADDRESS, (UINT8 *)&CmdEcgDiagnosticModeStartMeasurement, 1);
				EcgDscActualSubState++;
				break;
				
			case 1:
				//check if write operation was successful
				CheckWriteStatus();
				break;
				
			case 2:
				//if write was OK, start timer to poll DSC
				TimerEcgDscReadDscIndex = AddTimerQ(&TimerEcgDscReadDsc);		
				EcgDscActualState = STATE_IDLE;				
				CommunicationAttempts = 0;		/* Restart error counter */									
				EcgDscActualSubState = 0;
				break;
		}		
	}
		
	//Check if DSC hasn't responded 
	if (CommunicationAttempts >= MAX_COMMUNICATION_ATTEMPTS)
	{
		EcgDscActualState = STATE_IDLE;
		CommunicationAttempts = 0;		/* Restart error counter */									
		EcgDscActualSubState = 0;
		EcgDscActualEvent = EVENT_IIC_COMMUNICATION_ERROR;
	}
}

static void StateWritingStopCommand(void)
{
	if (!IIC_IS_BUSY())
	{
		switch (EcgDscActualSubState)
		{
			case 0:				
				RemoveTimerQ(TimerEcgDscReadDscIndex);		//stop timer to poll DSC
				vfnIICV1Write(DSC_SLAVE_ADDRESS, (UINT8 *)&CmdEcgDiagnosticModeStopMeasurement, 1);
				EcgDscActualSubState++;
				break;
				
			case 1:
				CheckWriteStatus();
				break;
				
			case 2:
				EcgDscActualState = STATE_IDLE;
				CommunicationAttempts = 0;		/* Restart error counter */
				EcgDscActualSubState = 0;
				break;
		}		
	}
		
	//Check if DSC hasn't responded 
	if (CommunicationAttempts >= MAX_COMMUNICATION_ATTEMPTS)
	{
		EcgDscActualState = STATE_IDLE;
		CommunicationAttempts = 0;		/* Restart error counter */									
		EcgDscActualSubState = 0;
		EcgDscActualEvent = EVENT_IIC_COMMUNICATION_ERROR;
	}	
}



static void StateReadingData(void)
{
	static UINT8 IsNewDataAvailable = 0;
	static UINT8 actualPacketId = 0;
	static UINT8 previousPacketId = 255;

	
	if (!IIC_IS_BUSY())
	{
		switch (EcgDscActualSubState)
		{
			case 0:				
				//poll the DSC to see if it has data
				//send CmdEcgIsNewDataAvailable command and read 1 byte response from DSC			
				vfnIICV1RandomRead(DSC_SLAVE_ADDRESS, (UINT8 *)&CmdEcgIsNewDataAvailable, 1, &IsNewDataAvailable, 1);
				EcgDscActualSubState++;
				break;
			
			case 1:
				CheckReadStatus();
				break;
				
			case 2:				
				//Instead of doing a random read, I will do a write and then a Read 
				//to give time to the DSC to fill its buffer					
				if (IsNewDataAvailable == NEW_DATA_AVAILABLE_1)
				{										
					//ask for buffer 1
					vfnIICV1Write(DSC_SLAVE_ADDRESS, (UINT8 *)&CmdEcgDiagnosticModeReadData1, 1);
					EcgDscActualSubState++;					
				}					
				else if (IsNewDataAvailable == NEW_DATA_AVAILABLE_2)
				{						
					//ask for buffer 2
					vfnIICV1Write(DSC_SLAVE_ADDRESS, (UINT8 *)&CmdEcgDiagnosticModeReadData2, 1);
					EcgDscActualSubState++;					
				}
				else //if (IsNewDataAvailable == NO_DATA_AVAILABLE) || IsNewDataAvailable == anything else
				{
					//if the DSC responded that has no data, start timer to poll again
					EcgDscActualState = STATE_IDLE;
					CommunicationAttempts = 0;		/* Restart error counter */
					EcgDscActualSubState = 0;
					TimerEcgDscReadDscIndex = AddTimerQ(&TimerEcgDscReadDsc);	//start timer to read again
					
					if (IsNewDataAvailable != NO_DATA_AVAILABLE)
					{
						ErrorCounter++;	
					}					
				}
				break;				
			
			case 3:
				CheckWriteStatus();
				break;
				
			case 4:
				//Wait some time so the DSC can send the data
				TimerWaitForDscIndex = AddTimerQ(&TimerWaitForDsc);
				EcgDscActualSubState++;
				break;
				
			case 5:
				if (DscIsReady)
				{
					DscIsReady = FALSE;
					vfnIICV1Read(DSC_SLAVE_ADDRESS, &DataFromDsc[0], DATA_LENGTH_FROM_DSC);
					EcgDscActualSubState++;
				}
				break;				

			case 6:
				CheckReadStatus();
				break;
			
			case 7:	
				//check if packet ID is correct and if checksum is correct			
				actualPacketId = DataFromDsc[PACKET_ID_POSITION];
				
				if (actualPacketId == (UINT8)(previousPacketId + 1))
				{
					volatile UINT8 i = 0;
					volatile INT16 CheckSum = 0;	
					volatile UINT16 rawData = 0;
					
					//No errors, calculate checksum
					i = DATA_START_POSITION;
					CheckSum = 0;		//checksum is the low byte of the sum of the words
					
					while (i<DATA_END_POSITION)
					{
						
						rawData = (UINT16)(DataFromDsc[i]);
						rawData = rawData + (UINT16)((UINT16)DataFromDsc[i+1] << 8);
						i+=2;
						
						CheckSum = (INT16) (CheckSum + (INT16)rawData);
					}
					
					
					EcgDsc_HeartRate = DataFromDsc[DATA_HEART_RATE_POSITION];
					
					CheckSum = CheckSum & 0x00FF;
					
					if (CheckSum != DataFromDsc[CHECKSUM_POSITION])
					{
						//error in checksum, read packet again
						EcgDscActualSubState = 2;
					}
					else
					{
						//everything was ok
						EcgDscActualState = STATE_IDLE;
						CommunicationAttempts = 0;		/* Restart error counter */									
						EcgDscActualSubState = 0;
						TimerEcgDscReadDscIndex = AddTimerQ(&TimerEcgDscReadDsc);	//start timer to read again
						EcgDscActualEvent = EVENT_ECG_DSC_DIAGNOSTIC_MODE_NEW_DATA_READY;	//generate event
					}					
				}
				else if (actualPacketId == previousPacketId)
				{
					//Repeated packet, ignore it
					ErrorCounter++;
					EcgDscActualState = STATE_IDLE;
					CommunicationAttempts = 0;		/* Restart error counter */									
					EcgDscActualSubState = 0;
					TimerEcgDscReadDscIndex = AddTimerQ(&TimerEcgDscReadDsc);	//start timer to read again
				}
				else if (actualPacketId == (UINT8)(previousPacketId + 2))
				{
					//1 missed packet, try to read again but swap buffer to read previous packet
					if (IsNewDataAvailable == NEW_DATA_AVAILABLE_1) 
					{
						IsNewDataAvailable = NEW_DATA_AVAILABLE_2;
					}
					else if (IsNewDataAvailable == NEW_DATA_AVAILABLE_2)
					{
						IsNewDataAvailable = NEW_DATA_AVAILABLE_1;
					}
					EcgDscActualSubState = 2;
				}
				else
				{
					//other error, fill buffer with INVALID_VALUEs
					//TO DO
					
					//Repeated packet, ignore it
					EcgDscActualState = STATE_IDLE;
					CommunicationAttempts = 0;		/* Restart error counter */									
					EcgDscActualSubState = 0;
					TimerEcgDscReadDscIndex = AddTimerQ(&TimerEcgDscReadDsc);	//start timer to read again
					ErrorCounter++;
				}								
				previousPacketId = actualPacketId;				
				break;
		}		
	}
		
	//Check if DSC hasn't responded 
	if (CommunicationAttempts >= MAX_COMMUNICATION_ATTEMPTS)
	{
		EcgDscActualState = STATE_IDLE;
		CommunicationAttempts = 0;		/* Restart error counter */									
		EcgDscActualSubState = 0;
		EcgDscActualEvent = EVENT_IIC_COMMUNICATION_ERROR;
	}	
}


static void TimerWaitForDsc_Event(void)
{
	DscIsReady = TRUE;
}

static void StateReadingHeartRate(void)
{
	if (!IIC_IS_BUSY())
	{
		switch (EcgDscActualSubState)
		{
			case 0:
				//poll the DSC to see if it has data
				//send CmdEcgIsNewDataAvailable command and read 1 byte response from DSC
				vfnIICV1RandomRead(DSC_SLAVE_ADDRESS, (UINT8 *)&CmdEcgIsNewDataAvailable, 1, &DataFromDsc[0], 2);
				EcgDscActualSubState++;
				break;
			
			case 1:
				CheckReadStatus();
				break;
				
				
			case 2:
				//if the DSC responded that has no data, start timer to poll again
				if (DataFromDsc[0] == NO_DATA_AVAILABLE)
				{
					EcgDscActualState = STATE_IDLE;
					CommunicationAttempts = 0;		/* Restart error counter */
					EcgDscActualSubState = 0;
					TimerEcgDscReadDscIndex = AddTimerQ(&TimerEcgDscReadDsc);	//start timer to read again
				}
				//Warning: removed if condition
				else //if ((DataFromDsc[1] == NEW_DATA_AVAILABLE_1) || (DataFromDsc[1] == NEW_DATA_AVAILABLE_2))
				{
					//if DSC has data, read it
					//Warning: removed heart rate at the end of the buffer to debug
					vfnIICV1RandomRead(DSC_SLAVE_ADDRESS, (UINT8 *)&CmdEcgDiagnosticModeReadData, 1, &DataFromDsc[0], 16 /*DSC_BUFFER_DATA_LENGTH*/);
					EcgDscActualSubState++;	
				}
				break;				
				
			case 3:
				vfnIICV1RandomRead(DSC_SLAVE_ADDRESS, (UINT8 *)&CmdEcgReadHeartRate, 1, &EcgDsc_HeartRate, 1);
				EcgDscActualSubState++;
				break;
				
			case 4:
				CheckReadStatus();
				break;
				
			case 5:				
				EcgDscActualState = STATE_IDLE;
				CommunicationAttempts = 0;		/* Restart error counter */									
				EcgDscActualSubState = 0;
				
				if (EcgDsc_HeartRate != 0)
				{
					//if calculated heart-rate is non-zero, send event
					EcgDscActualEvent = EVENT_ECG_DSC_DIAGNOSTIC_MODE_NEW_DATA_READY;
				}
				else
				{					
					TimerEcgDscReadDscIndex = AddTimerQ(&TimerEcgDscReadDsc);	//start timer to read again
				}
				break;
		}		
	}
		
	//Check if DSC hasn't responded 
	if (CommunicationAttempts >= MAX_COMMUNICATION_ATTEMPTS)
	{
		EcgDscActualState = STATE_IDLE;
		CommunicationAttempts = 0;		/* Restart error counter */									
		EcgDscActualSubState = 0;
		EcgDscActualEvent = EVENT_IIC_COMMUNICATION_ERROR;
	}	
}





/*******************************
* Public functions
*********************************/

/* call this only once at the beginning of the application */
void EcgDsc_Init(void)
{
	TimerEcgDscReadDsc.msCount = ECG_READ_DSC_INTERVAL;
	TimerEcgDscReadDsc.pfnTimerCallback = TimerEcgDscReadDsc_Event;
	TimerWaitForDsc.msCount =  TIMER_WAIT_FOR_DSC_INTERVAL;
	TimerWaitForDsc.pfnTimerCallback = TimerWaitForDsc_Event;
	vfnIICV1Init();		//init IIC
}


UINT8 EcgDsc_DiagnosticModeStartMeasurement(void)
{
	EcgDsc_HeartRate = 0;
	EcgDscActualState = STATE_WRITING_START_COMMAND;
	return TRUE;
}


UINT8 EcgDsc_HeartRateStartMeasurement(void)
{
	EcgDscActualState = STATE_WRITING_START_COMMAND;	//tell DSC to start reading
	EcgDscIsHeartRateMode = TRUE;
	return TRUE;
}



void EcgDsc_DiagnosticModeStopMeasurement(void)
{	
	RemoveTimerQ(TimerEcgDscReadDscIndex);	//stop timer	
	EcgDscActualState = STATE_WRITING_STOP_COMMAND;	//tell DSC to stop
}



void EcgDsc_PeriodicTask(void)
{
	vfnapIICV1Driver[gu8IICV1ActualState]();	//IIC state machine

	EcgDscStateMachine[EcgDscActualState]();
		
	/* Event handler */
	if (EcgDscActualEvent != EVENT_ECG_DSC_NONE)
	{
		if (EcgDsc_Events[EcgDscActualEvent] != NULL)
		{			
			EcgDsc_Events[EcgDscActualEvent]();	//execute registered event
			EcgDscActualEvent = EVENT_ECG_DSC_NONE;
		}		
	}
	
}