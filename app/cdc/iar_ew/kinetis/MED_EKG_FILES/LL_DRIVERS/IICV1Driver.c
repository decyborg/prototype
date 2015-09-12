/**************************************************************************************************
 *  Services performed by FREESCALE in this matter are performed AS IS and without any warranty.  *
 *  CUSTOMER retains the final decision relative to the total design and functionality of the     *
 *  end product. FREESCALE neither guarantees nor will be held liable by CUSTOMER for the         *
 *  success of this project. FREESCALE disclaims all warranties, express, implied or statutory    *
 *  including, but not limited to, implied warranty of merchantability or fitness for a           *
 *  particular purpose on any hardware, software ore advise supplied to the project by FREESCALE, *
 *  and or any product resulting from FREESCALE services . In no event shall FREESCALE be         *
 *  liable for incidental or consequential damages arising out of this agreement. CUSTOMER        *
 *  agrees to hold FREESCALE harmless against any and all claims demands or actions by anyone     *
 *  on account of any damage, or injury, whether commercial, contractual, or tortuous, rising     *
 *  directly or indirectly as a result of the advise or assistance supplied CUSTOMER in           *
 *  connection with product, services or goods supplied under this Agreement.                     *
 **************************************************************************************************/

/**************************************************************************************************
 *                                   I2C Drivers for Kinetis                                      *
 **************************************************************************************************/

#include "IICV1Driver.h"
//#include "IICV1HAL_K50.h"

volatile IICDrvStatus_t _IICDrvStatus;


void vfnIICV1StateIdle(void);
void vfnIICV1StateStartW(void);
void vfnIICV1StateStartR(void);
void vfnIICV1StateDataW(void);
void vfnIICV1StateDataR(void);
void vfnIICV1StateRestart(void);
void vfnIICV1StateStop(void);

void (*const vfnapIICV1Driver[])(void) = 
{
    vfnIICV1StateIdle,
    vfnIICV1StateStartW,
    vfnIICV1StateStartR,
    vfnIICV1StateDataW,
    vfnIICV1StateDataR,
    vfnIICV1StateRestart,
    vfnIICV1StateStop,
};

UINT8 gu8SlaveAddress;
UINT8 *gpu8DataOut;
UINT8 gu8SizeOut;
UINT8 *gpu8DataIn;
UINT8 gu8SizeIn;
UINT8 gu8Flags;

UINT8 gu8IICV1ActualState = IICV1_IDLE;
UINT8 gu8IICV1PreviousState = IICV1_IDLE;

/*****************************************************************************/

void vfnIICV1Init(void)
{
    SET_PIN_AS_IIC();
    
    IIC_CLOCK_RATE = IIC_CLOCK_RATE_VALUE;          
    IIC_CLOCK_DIV = IIC_CLOCK_DIV_VALUE;

    IIC_CONTROL_REGISTER = IIC_CONTROL_REGISTER_VALUE; //enable IIC with interrupts
    
    ENABLE_IIC_INTERRUPT();
}

void vfnIICV1Write(UINT8 u8SlaveAddress, UINT8 *pu8DataOut, UINT8 u8Size)
{
    gu8SlaveAddress = u8SlaveAddress;
    gpu8DataOut = pu8DataOut;
    gu8SizeOut = u8Size;
    gu8IICV1PreviousState = gu8IICV1ActualState;
    gu8IICV1ActualState = IICV1_START_W;
    u8IICV1DriverStatus = (1<<IIC_FLAGS_BUSY);

}

void vfnIICV1Read(UINT8 u8SlaveAddress, UINT8 *pu8DataIn, UINT8 u8Size) 
{
    gu8SlaveAddress = u8SlaveAddress;
    gpu8DataIn = pu8DataIn;
    gu8SizeIn = u8Size;
    gu8IICV1PreviousState = gu8IICV1ActualState;
    gu8IICV1ActualState = IICV1_START_R;
    u8IICV1DriverStatus = (1<<IIC_FLAGS_BUSY);
}

void vfnIICV1RandomRead(UINT8 u8SlaveAddress, UINT8 *pu8DataOut, UINT8 u8SizeOut,
UINT8 *pu8DataIn, UINT8 u8SizeIn)
{
    gu8SlaveAddress = u8SlaveAddress;
    gpu8DataOut = pu8DataOut;
    gu8SizeOut = u8SizeOut;
    gpu8DataIn = pu8DataIn;
    gu8SizeIn = u8SizeIn;
    gu8Flags |= (1<<FLAG_RANDOMREAD);
    gu8IICV1PreviousState = gu8IICV1ActualState;
    gu8IICV1ActualState = IICV1_START_W;
    u8IICV1DriverStatus = (1<<IIC_FLAGS_BUSY);
}


void vfnIICV1Interrupt(void)
{
    if(IICIF)
    {
        gu8Flags |= (1<<FLAG_TCF);
        IICIF = 1;
        return;
    }
    else if(IIC_ARBL)
    {                                       
        gu8Flags |= (1<<FLAG_ARBL);
        u8IICV1DriverStatus |= (1<<IIC_FLAGS_ERROR);
        IIC_ARBL = 1;
    }
}

/*****************************************************************************/

void vfnIICV1StateIdle(void)
{
    //Do Nothing
}

void vfnIICV1StateStartW(void)
{
    IIC_TX = 1;
    IIC_MST = 1;

    gu8SlaveAddress &= 0xFE;
    IIC_DATA_REGISTER = gu8SlaveAddress;

    gu8IICV1PreviousState = gu8IICV1ActualState;
    gu8IICV1ActualState = IICV1_DATA_W;
}

void vfnIICV1StateStartR(void)
{
    if(gu8IICV1PreviousState == IICV1_IDLE)
    {
        IIC_TX = 1;
        IIC_MST = 1;

        gu8SlaveAddress |= 0x01;
        IIC_DATA_REGISTER = gu8SlaveAddress;
        
        IIC_TXAK = 0;
        gu8IICV1PreviousState = gu8IICV1ActualState;
    }
    else
    {
        if(gu8Flags & (1<<FLAG_TCF))
        {
            if(!IIC_RXAK)
            {
                gu8Flags &= ~(1<<FLAG_TCF);
                IIC_TX = 0;           
                (void)IIC_DATA_REGISTER;      //dummy read
                gu8SizeIn--;                
                gu8IICV1ActualState = IICV1_DATA_R;              
            }
            else
            {
                gu8Flags |= (1<<FLAG_ERROR);
                u8IICV1DriverStatus |= (1<<IIC_FLAGS_ERROR);
                gu8IICV1ActualState = IICV1_STOP;
            }
        }
    }      
}

void vfnIICV1StateDataW(void)
{
    if(gu8Flags & (1<<FLAG_TCF))
    {
        gu8Flags &= ~(1<<FLAG_TCF);
        if(!IIC_RXAK)
        {
            if(gu8SizeOut)
            {
                IIC_DATA_REGISTER = *gpu8DataOut;
                gu8SizeOut--;
                gpu8DataOut++;
            }
            else
            {
                if(gu8Flags & (1<<FLAG_RANDOMREAD))
                {
                    gu8Flags &= ~(1<<FLAG_RANDOMREAD);
                    gu8IICV1PreviousState = gu8IICV1ActualState;
                    gu8IICV1ActualState = IICV1_RESTART;
                }
                else
                {
                    u8IICV1DriverStatus |= (1<<IIC_FLAGS_WRITE_OK);
                    gu8IICV1PreviousState = gu8IICV1ActualState;
                    gu8IICV1ActualState = IICV1_STOP;
                }
            }
        }                                                     
        else
        {
            gu8Flags |= (1<<FLAG_ERROR);
            u8IICV1DriverStatus |= (1<<IIC_FLAGS_ERROR);
            gu8IICV1PreviousState = gu8IICV1ActualState;
            gu8IICV1ActualState = IICV1_STOP;
        }
    }    
}

void vfnIICV1StateDataR(void)
{
    if(gu8Flags & (1<<FLAG_TCF))
    {
        gu8Flags &= ~(1<<FLAG_TCF);

        IIC_TX = 0;

        if(gu8SizeIn == 1)
        {
            IIC_TXAK = 1;
        }
        else if(gu8SizeIn == 0)
        {
            u8IICV1DriverStatus |= (1<<IIC_FLAGS_READY);
            gu8IICV1PreviousState = gu8IICV1ActualState;
            gu8IICV1ActualState = IICV1_STOP;
            return;
        }

        *gpu8DataIn = IIC_DATA_REGISTER;
        gu8SizeIn--;
        gpu8DataIn++;
    }    
}

void vfnIICV1StateRestart(void)
{
    if(gu8IICV1PreviousState != gu8IICV1ActualState)
    {
        IIC_RSTA = 1;

        gu8SlaveAddress |= 0x01;
        IIC_DATA_REGISTER = gu8SlaveAddress;
        
        IIC_TXAK = 0;
        gu8IICV1PreviousState = gu8IICV1ActualState;
    }
    else
    {
        if(gu8Flags & (1<<FLAG_TCF))
        {
            if(!IIC_RXAK)
            {
                gu8Flags &= ~(1<<FLAG_TCF);
                IIC_TX = 0;           

                if(gu8SizeIn == 1)
                {
                    IIC_TXAK = 1;
                }

                (void)IIC_DATA_REGISTER;      //dummy read
                gu8SizeIn--;
                gu8IICV1ActualState = IICV1_DATA_R;                          
            }
            else
            {
                gu8Flags |= (1<<FLAG_ERROR);
                u8IICV1DriverStatus |= (1<<IIC_FLAGS_ERROR);
                gu8IICV1ActualState = IICV1_STOP;
            }
        }
    }      
}

void vfnIICV1StateStop(void)
{
    if(gu8IICV1PreviousState != IICV1_STOP)
    {
        IIC_TXAK = 0;
        IIC_MST = 0;
        if (gu8IICV1PreviousState == IICV1_DATA_R )
        {
            *gpu8DataIn = IIC_DATA_REGISTER;
        }
        else
        {
            (void)IIC_DATA_REGISTER;
        }
        gu8IICV1PreviousState = IICV1_STOP;        
    }
    else
    {
        if(!IIC_BUSY)
        {
            u8IICV1DriverStatus &= ~(1<<IIC_FLAGS_BUSY);
            gu8IICV1ActualState = IICV1_IDLE;
        }
    }
}

/*****************************************************************************/

