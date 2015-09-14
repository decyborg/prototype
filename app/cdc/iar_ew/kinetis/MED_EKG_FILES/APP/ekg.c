/******************************************************************************
 *
 * Freescale Semiconductor Inc.
 * (c) Copyright 2004-2010 Freescale Semiconductor, Inc.
 * ALL RIGHTS RESERVED.
 *
 **************************************************************************//*!
 *
 * @file ekg.c
 *
 * @author
 *
 * @version
 *
 * @date Aug-18-2010
 *
 * @brief  EKG demo.
 *****************************************************************************/


/******************************************************************************
 * Includes
 *****************************************************************************/
//#include "hidef.h"          /* for EnableInterrupts macro */
#include "derivative.h"     /* include peripheral declarations */
//#include "common.h"
#include "types.h"          /* Contains User Defined Data Types */
#include "usb_cdc.h"        /* USB CDC Class Header File */
#include "ekg.h"            /* Virtual COM Application Header File */
#include "SerialCommands.h"
#include "OPAMP.h"
#include "DAC.h"
#include "EcgDsc.h"
#include "Ecg.h"
#include "TWR_K50_UI.h"
#include "Kinetis_FIR.h"
#include "Kinetis_FTM.h" 

#if (defined _MCF51MM256_H) || (defined _MCF51JE256_H)
#include "exceptions.h"
#endif

#if (defined(_MC9S08MM128_H))
#define DEBOUNCE_TIME 10000
#endif
#if (defined _MCF51MM256_H)
#define DEBOUNCE_TIME 25000
#endif
#if (defined MCU_MK53N512CMD100)
#define DEBOUNCE_TIME 50000
#endif


/* skip the inclusion in dependency state */
#ifndef __NO_SETJMP
	#include <stdio.h>
#endif
//#include <stdlib.h>
//#include <string.h>

//#define SEND_SINE_WAVE

//#define SERIAL_TEST
/*****************************************************************************
 * Local Types - None
 *****************************************************************************/
typedef enum
{
	NO_MEASUREMENT,
	GLU_MEASUREMENT,
	BPM_MEASUREMENT,
	BPM_LEAK_TEST,
	ECG_MEASUREMENT,
	SPR_MEASUREMENT,
	HEIGHT_MEASUREMENT,
	WEIGHT_MEASUREMENT,
	TEMPERATURE_MEASUREMENT
}EkgCommand_e;

/*****************************************************************************
 * Local Functions Prototypes
 *****************************************************************************/
static void USB_App_Callback(uint_8 controller_ID,
                        uint_8 event_type, void* val);
static void USB_Notify_Callback(uint_8 controller_ID,
                        uint_8 event_type, void* val);
static void Virtual_Com_Recv_Serial_Data(void);

static void Virtual_Com_Send_Serial_Data(void);

static void EcgDiagnosticModeStartMeasurementReq(void);
static void EcgDiagnosticModeStopMeasurementReq(void);
static void EcgDiagnosticModeNewDataReadyInd(void);

void vfnEnable_AFE (void);

#ifdef SEND_SINE_WAVE
static void TimerSendDummyData_Event(void);
#endif

/*****************************************************************************
 * Constant and Macro's - None
 *****************************************************************************/
const pFunc_t Ecg_Events[] = 
{
	NULL,									//EVENT_ECG_NONE,
	NULL,	//EVENT_ECG_HEART_RATE_MEASUREMENT_COMPLETE_OK,
	NULL,		//EVENT_ECG_HEART_RATE_MEASUREMENT_ERROR,
	NULL,	//EcgHeartBeatOccurredInd,				//EVENT_ECG_HEART_BEAT_OCCURRED,
	EcgDiagnosticModeNewDataReadyInd		//EVENT_ECG_DIAGNOSTIC_MODE_NEW_DATA_READY	
};


const pFunc_t EcgDsc_Events[] = 
{
	NULL,									//EVENT_ECG_NONE,
	NULL,	//EVENT_ECG_HEART_RATE_MEASUREMENT_COMPLETE_OK,
	NULL,		//EVENT_ECG_HEART_RATE_MEASUREMENT_ERROR,
	NULL,	//EcgHeartBeatOccurredInd,				//EVENT_ECG_HEART_BEAT_OCCURRED,
	EcgDiagnosticModeNewDataReadyInd		//EVENT_ECG_DIAGNOSTIC_MODE_NEW_DATA_READY	
};



const pFunc_t ExecuteCommandReq[] =
{
	NULL,				//GLU_START_MEASUREMENT,
	NULL,				//GLU_ABORT_MEASUREMENT,
	NULL,				//GLU_START_CALIBRATION,
	NULL,								//GLU_BLOOD_DETECTED,
	NULL,								//GLU_MEASUREMENT_COMPLETE_OK,
	NULL,								//GLU_CALIBRATION_COMPLETE_OK,
	
	NULL,				//BPM_START_MEASUREMENT,
	NULL,				//BPM_ABORT_MEASUREMENT,
	NULL,								//BPM_MEASUREMENT_COMPLETE_OK,
	NULL,								//BPM_MEASUREMENT_ERROR,
										  
	NULL,				//BPM_START_LEAK_TEST,
	NULL,					//BPM_ABORT_LEAK_TEST,
	NULL,								//BPM_LEAK_TEST_COMPLETE,	
	
	NULL,	//ECG_HEART_RATE_START_MEASUREMENT,
	NULL,	//ECG_HEART_RATE_ABORT_MEASUREMENT,
	NULL,								//ECG_HEART_RATE_MEASUREMENT_COMPLETE_OK,
	NULL,								//ECG_HEART_RATE_MEASUREMENT_ERROR,
	NULL,								//ECG_HEART_BEAT_OCCURRED,
	
	EcgDiagnosticModeStartMeasurementReq,	//ECG_DIAGNOSTIC_MODE_START_MEASUREMENT,
	EcgDiagnosticModeStopMeasurementReq, 	//ECG_DIAGNOSTIC_MODE_STOP_MEASUREMENT,
	EcgDiagnosticModeNewDataReadyInd,		//ECG_DIAGNOSTIC_MODE_NEW_DATA_READY,	
		
	NULL,				//TMP_READ_TEMPERATURE,
	NULL,					//HGT_READ_HEIGHT,
	NULL,					//WGT_READ_WEIGHT,

	NULL,	//SprStartMeasurementReq,				//SPR_START_MEASUREMENT,
	NULL,	//SprAbortMeasurementReq,				//SPR_ABORT_MEASUREMENT,
	NULL,								//SPR_MEASUREMENT_COMPLETE_OK,
	NULL,								//SPR_MEASUREMENT_ERROR,

	NULL,								//SPR_DIAGNOSTIC_MODE_START_MEASURMENT,
	NULL,								//SPR_DIAGNOSTIC_MODE_STOP_MEASURMENT,
	NULL,								//SPR_DIAGNOSTIC_MODE_NEW_DATA_READY,
	NULL,								//SPR_DIAGNOSTIC_MODE_MEASUREMENT_COMPLETE_OK,
	NULL,								//SPR_DIAGNOSTIC_MODE_MEASUREMENT_ERROR,
		
	NULL,	//PoxStartMeasurementReq,				//POX_START_MEASUREMENT,
	NULL,	//PoxAbortMeasurementReq,				//POX_ABORT_MEASURMENT,
	NULL,								//POX_MEASURMENT_COMPLETE_OK,
	NULL,								//POX_MEASURMENT_ERROR,

	NULL,	//PoxDiagnosticModeStartMeasurementReq,//POX_DIAGNOSTIC_MODE_START_MEASURMENT,
	NULL,	//PoxDiagnosticModeStopMeasurementReq,//POX_DIAGNOSTIC_MODE_STOP_MEASURMENT,
	NULL,								//POX_DIAGNOSTIC_MODE_NEW_DATA_READY
	NULL,	//BPM_SEND_PRESSURE_VALUE_TO_PC,
	NULL,	//SYSTEM_RESTART,
	NULL	//BPM_DATA_READY = 0xFF,
};

const UINT8 AmpGain[] = 
{
 Gain2,
 Gain3,
 Gain4,
 Gain5,
 Gain6,
 Gain7,
 Gain8,
 Gain9,
 Gain10,
 Gain11,
 Gain12,
 Gain13,
 Gain14,
 Gain15,
 Gain16,
 Gain17,
 Gain18
};

/****************************************************************************
 * Global Variables
 ****************************************************************************/
 /*****************************************************************************
 * Global Functions Prototypes
 *****************************************************************************/
void TestApp_Init(void);

/*****************************************************************************
 * Local Variables
 *****************************************************************************/
#ifdef _MC9S08JS16_H
#pragma DATA_SEG APP_DATA
#endif
/* Virtual COM Application start Init Flag */
static volatile boolean start_app = FALSE;
/* Virtual COM Application Carrier Activate Flag */
static volatile boolean start_transactions = FALSE;
/* Receive Buffer */
static uint_8 g_curr_recv_buf[DATA_BUFF_SIZE];
/* Send Buffer */
static uint_8 g_curr_send_buf[DATA_BUFF_SIZE];
/* Receive Data Size */
static uint_8 g_recv_size;
/* Send Data Size */
static uint_8 g_send_size;

static UINT16 PacketIdNumber = 0;
static UINT8 ActualMeasurement;

static UINT8 u8Gain_index = 9;
static UINT16 u16Debounce = 0;

#ifdef SEND_SINE_WAVE
static TIMER_OBJECT TimerSendDummyData;
static UINT8  TimerSendDummyDataIndex;
#endif

unsigned int Prueba;
/*****************************************************************************
 * Local Functions
 *****************************************************************************/

 /******************************************************************************
 *
 *   @name        TestApp_Init
 *
 *   @brief       This function is the entry for the Virtual COM Loopback app
 *
 *   @param       None
 *
 *   @return      None
 *****************************************************************************
 * This function starts the Virtual COM Loopback application
 *****************************************************************************/

void TestApp_Init(void)
{
    uint_8   error;

    /*******************************
    Initialize things for Ekg
    *******************************/   
    DAC12_Vin_SWtrig();//set DAC to ouput 1.6 base from 3.3v reference
    ADC1_Init16b();
    ADC0_Init16b();
    opamp1_gp_mode() ;
    opamp2_noninverting_mode(Gain6);
    TRIAMP1C0_HighMode();
    TRIAMP2C0_HighMode();
    vfnEnable_AFE ();
    
    /*******************************
    Initialize Buttons and LEDs
    *******************************/     
    GPIOS_CLOCK_INIT
    INIT_ALL_PBS    
    LED1_INIT
    LED2_INIT

#if (defined ECG_FIR)    
    FTM0_Init();
#elif (defined ECG_DSC)
    EcgDsc_Init();
#endif
      
    Ecg_Init();	

#ifdef SEND_SINE_WAVE
    TimerSendDummyData.msCount = 64;
	  TimerSendDummyData.pfnTimerCallback = TimerSendDummyData_Event;
#endif
    /*******************************/
     
    g_recv_size = 0;
    g_send_size= 0;
    DisableInterrupts;		
    #if (defined _MCF51MM256_H) || (defined _MCF51JE256_H)
     usb_int_dis();
    #endif
    /* Initialize the USB interface */
    error = USB_Class_CDC_Init(CONTROLLER_ID,USB_App_Callback,
                                NULL,USB_Notify_Callback);
    if(error != USB_OK)
    {
        /* Error initializing USB-CDC Class */
        return;
    }
    EnableInterrupts;
	#if (defined _MCF51MM256_H) || (defined _MCF51JE256_H)
     usb_int_en();
    #endif

#if (defined SERIAL_TEST)  /* Start measurement if Serial connection is used */
EcgDiagnosticModeStartMeasurementReq();
#endif
}

/******************************************************************************
 *
 *   @name        TestApp_Task
 *
 *   @brief       Application task function. It is called from the main loop
 *
 *   @param       None
 *
 *   @return      None
 *
 *****************************************************************************
 * Application task function. It is called from the main loop
 *****************************************************************************/
void TestApp_Task(void)
{

        /* call the periodic task function */
        USB_Class_CDC_Periodic_Task();
        
#if (defined ECG_DSC)
        EcgDsc_PeriodicTask();
#elif (defined ECG_FIR)
        
        if (gu8FTM0_TOF && ActualMeasurement == ECG_MEASUREMENT)   //if 0.5ms has passed
        {
         gu8FTM0_TOF = 0;
         vfnFilter_Periodic_Task();
         
         if(gu8FilterDataReady)
         EcgDiagnosticModeNewDataReadyInd();
        }
#endif        
        Ecg_PeriodicTask();

       /* check whether enumeration is complete or not */
        if((start_app==TRUE))// && (start_transactions==TRUE))
        {
            Virtual_Com_Recv_Serial_Data();
            Virtual_Com_Send_Serial_Data();
        }
        
       /* check if SW1 is press to reduce the gain */
        if(!PB1_STATUS & !u16Debounce)  
        {
          if (u8Gain_index) u8Gain_index--;
          LED1_ON
          opamp2_noninverting_mode(AmpGain[u8Gain_index]);
          u16Debounce = DEBOUNCE_TIME; 
        }
    
        /* check if SW2 is press to increase the gain */
        if((!PB2_STATUS & !u16Debounce))  
        {
          if (u8Gain_index<16) u8Gain_index++;
          LED2_ON
          opamp2_noninverting_mode(AmpGain[u8Gain_index]);
          u16Debounce = DEBOUNCE_TIME;
        }
    
        /* check if SW1 and SW2 are released */
        if(PB1_STATUS && PB2_STATUS && u16Debounce) 
        {
          if (u8Gain_index==0) LED1_ON
          else if (u8Gain_index==16) LED2_ON
          else 
          {
            LED1_OFF
            LED2_OFF
          }
        }
        
       if (u16Debounce) u16Debounce--;	    
}

/******************************************************************************
 *
 *    @name       Virtual_Com_Recv_Serial_Data
 *
 *    @brief      Implements Loopback COM Port
 *
 *    @param      None
 *
 *    @return     None
 *
 *****************************************************************************
 * Receives data from USB Host and process it
 *****************************************************************************/
static void Virtual_Com_Recv_Serial_Data(void)
{
    if(g_recv_size)
    {
        //there is new data
		    if (g_curr_recv_buf[PACKET_TYPE] == REQ)
		    {
			    if (ExecuteCommandReq[g_curr_recv_buf[COMMAND_OPCODE]] != NULL)
			    {
				    //check if OPCCODE is in a valid range
				    if ((g_curr_recv_buf[COMMAND_OPCODE] <= LAST_COMMAND))
				    {
					    ExecuteCommandReq[g_curr_recv_buf[COMMAND_OPCODE]]();
				    }
			    }	
		    }
		//else
		//{
			//packet type is not a request
		//}
		    g_recv_size = 0;
    }
    return;
}

/******************************************************************************
 *
 *    @name       Virtual_Com_Send_Serial_Data
 *
 *    @brief      Send serial data in g_curr_send_buf to USB port
 *
 *    @param      None
 *
 *    @return     None
 *
 *****************************************************************************
 * Send data to USB Host
 *****************************************************************************/
static void Virtual_Com_Send_Serial_Data(void)
{
    uint_8 status = 0;
    if(g_send_size)
    {
        /* Send Data to USB Host*/
        uint_8 size = g_send_size;
        g_send_size = 0;
        
        if(g_send_size > DATA_BUFF_SIZE)
        {
         asm ("NOP");
        }
        
        status = USB_Class_CDC_Interface_DIC_Send_Data(CONTROLLER_ID, &g_curr_send_buf[0], size);
        
        if(status != USB_OK)
        {
            /* Send Data Error Handling Code goes here */
        }
    }

}
/******************************************************************************
 *
 *    @name        USB_App_Callback
 *
 *    @brief       This function handles Class callback
 *
 *    @param       controller_ID    : Controller ID
 *    @param       event_type       : Value of the event
 *    @param       val              : gives the configuration value
 *
 *    @return      None
 *
 *****************************************************************************
 * This function is called from the class layer whenever reset occurs or enum
 * is complete. After the enum is complete this function sets a variable so
 * that the application can start.
 * This function also receives DATA Send and RECEIVED Events
 *****************************************************************************/

static void USB_App_Callback (
    uint_8 controller_ID,   /* [IN] Controller ID */
    uint_8 event_type,      /* [IN] value of the event */
    void* val               /* [IN] gives the configuration value */
)
{
    UNUSED (controller_ID)
    UNUSED (val)
    if(event_type == USB_APP_BUS_RESET)
    {
        start_app=FALSE;
    }
    else if(event_type == USB_APP_ENUM_COMPLETE)
    {
        start_app=TRUE;
    }
    else if((event_type == USB_APP_DATA_RECEIVED))
    //&&            (start_transactions == TRUE))
    {
        /* Copy Received Data buffer to Application Buffer */
        USB_PACKET_SIZE BytesToBeCopied;
        APP_DATA_STRUCT* dp_rcv = (APP_DATA_STRUCT*)val;
        uint_8 index;
        BytesToBeCopied = (USB_PACKET_SIZE)((dp_rcv->data_size > DATA_BUFF_SIZE) ?
                                      DATA_BUFF_SIZE:dp_rcv->data_size);
        for(index = 0; index<BytesToBeCopied ; index++)
        {
            g_curr_recv_buf[index]= dp_rcv->data_ptr[index];
        }
        g_recv_size = index;
        (void)USB_Class_CDC_Interface_DIC_Recv_Data(CONTROLLER_ID, NULL, 0);
    }
    else if((event_type == USB_APP_SEND_COMPLETE)&&
                                              (start_transactions == TRUE))
    {
        /* Previous Send is complete. Queue next receive */
        (void)USB_Class_CDC_Interface_DIC_Recv_Data(CONTROLLER_ID, NULL, 0);
    }

    return;
}

/******************************************************************************
 *
 *    @name        USB_Notify_Callback
 *
 *    @brief       This function handles PSTN Sub Class callbacks
 *
 *    @param       controller_ID    : Controller ID
 *    @param       event_type       : PSTN Event Type
 *    @param       val              : gives the configuration value
 *
 *    @return      None
 *
 *****************************************************************************
 * This function handles USB_APP_CDC_CARRIER_ACTIVATED and
 * USB_APP_CDC_CARRIER_DEACTIVATED PSTN Events
 *****************************************************************************/

static void USB_Notify_Callback (
    uint_8 controller_ID,   /* [IN] Controller ID */
    uint_8 event_type,      /* [IN] PSTN Event Type */
    void* val               /* [IN] gives the configuration value */
)
{
    UNUSED (controller_ID)
    UNUSED (val)
    if(start_app == TRUE)
    {
        if(event_type == USB_APP_CDC_CARRIER_ACTIVATED)
        {
            start_transactions = TRUE;
        }
        else if(event_type == USB_APP_CDC_CARRIER_DEACTIVATED)
        {
            start_transactions = FALSE;
        }
    }
    return;
}

void EcgDiagnosticModeStartMeasurementReq(void)
{
	if (ActualMeasurement == NO_MEASUREMENT)
	{
		//execute command and send confirm
		g_curr_send_buf[g_send_size++] = CFM;
		g_curr_send_buf[g_send_size++] = ECG_DIAGNOSTIC_MODE_START_MEASUREMENT;
		g_curr_send_buf[g_send_size++] = 1;	//data bytes

#ifdef SEND_SINE_WAVE
		g_curr_send_buf[g_send_size++] = ERROR_OK;
		TimerSendDummyDataIndex = AddTimerQ(&TimerSendDummyData);
#else
	#ifdef ECG_DSC
	
		if ( (EcgDsc_DiagnosticModeStartMeasurement() == TRUE) && 
		     (Ecg_DiagnosticModeStartMeasurement() ==TRUE) )
		{
			g_curr_send_buf[g_send_size++] = ERROR_OK;
		}
		else
		{
			g_curr_send_buf[g_send_size++] = ERROR_BUSY;
		}

	#else
		if (Ecg_DiagnosticModeStartMeasurement() == TRUE)
		{
			g_curr_send_buf[g_send_size++] = ERROR_OK;
		}
		else
		{
			g_curr_send_buf[g_send_size++] = ERROR_BUSY;
		}
		
	#endif			
#endif

		Virtual_Com_Send_Serial_Data();			
		
		ActualMeasurement = ECG_MEASUREMENT;
	}

}
void EcgDiagnosticModeStopMeasurementReq(void)
{
	if (ActualMeasurement == ECG_MEASUREMENT)
	{			

#ifdef SEND_SINE_WAVE
		RemoveTimerQ(TimerSendDummyDataIndex);		
#else
	#ifdef ECG_DSC
		EcgDsc_DiagnosticModeStopMeasurement();
		Ecg_DiagnosticModeStopMeasurement();
	#else
		Ecg_DiagnosticModeStopMeasurement();
	#endif
#endif
		
		g_curr_send_buf[g_send_size++] = CFM;
		g_curr_send_buf[g_send_size++] = ECG_DIAGNOSTIC_MODE_STOP_MEASUREMENT;
		g_curr_send_buf[g_send_size++] = 0;	//data bytes
				
		Virtual_Com_Send_Serial_Data();
		ActualMeasurement = NO_MEASUREMENT;
	}
}
void EcgDiagnosticModeNewDataReadyInd(void)
{
	static UINT16 IdNumber = 0;

#ifdef ECG_DSC
	if (ActualMeasurement == ECG_MEASUREMENT)
	{		
		UINT8 i=0;
	
                
		//Send indication
		g_curr_send_buf[g_send_size++] = IND;
		g_curr_send_buf[g_send_size++] = ECG_DIAGNOSTIC_MODE_NEW_DATA_READY;
		g_curr_send_buf[g_send_size++] = DATA_LENGTH_FROM_DSC-1;				//data from DSC (64) + packetID
		
		g_curr_send_buf[g_send_size++] = (UINT8) (IdNumber >> 8);
		g_curr_send_buf[g_send_size++] = (UINT8) (IdNumber & 0x00FF);
		
		IdNumber++;
				
		//copy data from DSC to outbuffer
		i = DATA_START_POSITION;
		
		while (i<DATA_END_POSITION)
		{
			g_curr_send_buf[g_send_size++] = DataFromDsc[i+1];	//swap data bytes
			g_curr_send_buf[g_send_size++] = DataFromDsc[i];
			i+=2;
		}
		
		g_curr_send_buf[g_send_size++] = EcgDsc_HeartRate;
		
		//send data
		Virtual_Com_Send_Serial_Data();
	}
        
#elif (defined ECG_FIR)
        
         if (ActualMeasurement == ECG_MEASUREMENT && gu8FilterDataReady)
	{		
		UINT8 i=0;
                
                gu8FilterDataReady = 0;
		
		//Send indication
		g_curr_send_buf[g_send_size++] = IND;
		g_curr_send_buf[g_send_size++] = ECG_DIAGNOSTIC_MODE_NEW_DATA_READY;
		g_curr_send_buf[g_send_size++] = (FIR_DATA_TO_SEND*2)+ 3; //filtered data number + 2 Ceck packets don't include HR Byte
		
		g_curr_send_buf[g_send_size++] = (UINT8) (IdNumber >> 8);
		g_curr_send_buf[g_send_size++] = (UINT8) (IdNumber & 0x00FF);
		
		IdNumber++;
				
		//copy data from FIR to outbuffer
		i = FIR_DATA_TO_SEND;
		
		while (i)
		{
                  
                  /* Small peak Elimination Routine */
                  /*
                  if (gu16FIR_Data[i-1] > gu16FIR_Data[i] && gu16FIR_Data[i-1] > gu16FIR_Data[i-2])
                    gu16FIR_Data[i-1] = gu16FIR_Data[i-2];
                  if (gu16FIR_Data[i-1] < gu16FIR_Data[i] && gu16FIR_Data[i-1] < gu16FIR_Data[i-2])
                    gu16FIR_Data[i-1] = gu16FIR_Data[i];
                  
                  /* Copy buffer normaly */
			g_curr_send_buf[g_send_size++] = (UINT8)(gu16FIR_Data[i-1]>>8);	//copy ECG data to OutBuffer
			g_curr_send_buf[g_send_size++] = (UINT8)(gu16FIR_Data[i-1]&0x00FF);
                        i--;
		}
		
		//g_curr_send_buf[g_send_size++] = 0;
                g_curr_send_buf[g_send_size++] = (UINT8)(Averaged_HR&0x00FF);
		
                
		//send data
		Virtual_Com_Send_Serial_Data();
	}
        
        
#else 
	if (ActualMeasurement == ECG_MEASUREMENT)
	{		
		UINT8 i=0;
		
		//Send indication
		g_curr_send_buf[g_send_size++] = IND;
		g_curr_send_buf[g_send_size++] = ECG_DIAGNOSTIC_MODE_NEW_DATA_READY;
		g_curr_send_buf[g_send_size++] = ECG_DATA_BUFFER_LENGTH + 2 + 1;				//data from ECG + packetID + ECG_HeartRate
		
		g_curr_send_buf[g_send_size++] = (UINT8) (IdNumber >> 8);			//add packetId for error handling
		g_curr_send_buf[g_send_size++] = (UINT8) (IdNumber & 0x00FF);
		
		IdNumber++;
				
		//copy data from DSC to outbuffer
		i = 0;
		
		while (i<ECG_DATA_BUFFER_LENGTH)
		{
			g_curr_send_buf[g_send_size++] = EcgDataBuffer[i++];	//copy ECG data to OutBuffer
			g_curr_send_buf[g_send_size++] = EcgDataBuffer[i++];
		}
		
		g_curr_send_buf[g_send_size++] = Ecg_HeartRate;
		//send data
		Virtual_Com_Send_Serial_Data();
	}
#endif
}


#ifdef SEND_SINE_WAVE
//this is the timer event for sending a dummy sine wave

void TimerSendDummyData_Event(void)
{
	static const UINT16 SinX[] = 
	{
		1250, 1349, 1448, 1545, 1640, 1733, 1823, 1910, 1992, 2069, 2142, 2208, 2269, 2323, 2371, 2411, 2444, 2470, 2488, 2498, 2500, 2494, 2481, 2459, 2430, 2394, 2350, 2300, 2243, 2179, 2110, 2036, 1956, 1872, 1784, 1693, 1598, 1502, 1404, 1305, 1206, 1107, 1009, 912, 818, 726, 638, 553, 473, 398, 328, 264, 206, 155, 111, 73, 44, 21, 7, 0, 2, 11, 28, 53, 85, 124, 171, 225, 284, 350, 422, 499, 581, 667, 756, 849, 944, 1041, 1140, 1239
	};

	#define SIN_X_LAST_ELEMENT 	79
	static const UINT8 EcgBufferSize = 64;
	static const UINT8 SinXTimerPeriod	= 64;




	if (ActualMeasurement == ECG_MEASUREMENT)
	{		
		static UINT8 SinXArrayActualElement = 0;

		//Send indication
		g_curr_send_buf[g_send_size++] = IND;
		g_curr_send_buf[g_send_size++] = ECG_DIAGNOSTIC_MODE_NEW_DATA_READY;
		g_curr_send_buf[g_send_size++] = EcgBufferSize + 2;				//data bytes
		
		g_curr_send_buf[g_send_size++] = (UINT8) (PacketIdNumber >> 8);
		g_curr_send_buf[g_send_size++] = (UINT8) (PacketIdNumber & 0x00FF);
		
		PacketIdNumber++;
		
		
		while (g_send_size < (EcgBufferSize + DATA_PACKET + 2))
		{
			g_curr_send_buf[g_send_size++] = SinX[SinXArrayActualElement] >> 8;		
			g_curr_send_buf[g_send_size++] = SinX[SinXArrayActualElement] & 0x00FF;
			
			if (SinXArrayActualElement == SIN_X_LAST_ELEMENT)
			{
				SinXArrayActualElement = 0;
			}
			else
			{
				SinXArrayActualElement++;
			}
		}

		Virtual_Com_Send_Serial_Data();
		
		TimerSendDummyDataIndex = AddTimerQ(&TimerSendDummyData);
	}	
	
}
#endif


void SWdelay(void)
{
  UINT16 debounce= 50000;
  while (debounce) debounce--;
}

void vfnEnable_AFE (void)
{
  SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK; //Clock to PORTC
  PORTC_PCR14 = PORT_PCR_MUX(1); //PTC14 as GPIO
  GPIOC_PDDR |= (1<<14); //PTC14 as output
  GPIOC_PCOR |= (1<<14); //AFEs Enabled
}

/* EOF */
