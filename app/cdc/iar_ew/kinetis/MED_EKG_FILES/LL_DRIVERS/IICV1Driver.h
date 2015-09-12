/**
\file
\brief 	This driver uses the IIC module of the S08 MCUs as master. It does not deal with any
		logical address or register manipuulation of the IIC device, just the physical layer.
		This file was created by Freescale. Modified by Samuel Quiroz
\author Freescale.
\date 2009-04-03
*/
#ifndef __IICV1Driver
#define __IICV1Driver

#include "types.h"
#include "IICV1HAL_K50.h" /* include peripheral declarations */


/** Initializes IIC module */
void vfnIICV1Init(void);

/** Write data to IIC device.
	@param u8SlaveAddress 	The device address of the slave. Note that this address is right aligned,
						  	most devices specify the address left aligned, so probably you will need
						  	left shift 1 position the device address.
	@param pu8DataOut 		Pointer to the buffer witht the data to send
	@param u8Size			Number of bytes to write
 */
 
void vfnIICV1Write(UINT8 u8SlaveAddress, UINT8 *pu8DataOut, UINT8 u8Size);

/**
	Reads data from IIC device. It does not specify which address to read.
	@param u8SlaveAddress 	The device address of the slave. Note that this address is right aligned,
						  	most devices specify the address left aligned, so probably you will need
						  	left shift 1 position the device address.
	@param pu8DataIn		Pointer to the buffer where data will be placed.
	@param u8Size			Number of bytes to read
*/
void vfnIICV1Read(UINT8 u8SlaveAddress, UINT8 *pu8DataIn, UINT8 u8Size);

/**
	Reads data from a specific logical address. It first writes bytes, then reads bytes.
	@param u8SlaveAddress 	The device address of the slave. Note that this address is right aligned,
						  	most devices specify the address left aligned, so probably you will need
						  	left shift 1 position the device address.
	@param pu8DataOut 		Pointer to the buffer witht the data to send.
	@param u8SizeOut		Number of bytes to write (this is usually 1).
	@param pu8DataIn		Pointer to the buffer where data will be placed.
	@param u8SizeIn			Number of bytes to read.
*/
void vfnIICV1RandomRead(UINT8 u8SlaveAddress, UINT8 *pu8DataOut, UINT8 u8SizeOut,UINT8 *pu8DataIn, UINT8 u8SizeIn);

/**
	Interrupt service routine.
*/
void vfnIICV1Interrupt(void);


extern void (*const vfnapIICV1Driver[])(void);	/**< Array of function pointers that implements a state machine. */
extern UINT8 gu8IICV1ActualState;				/**< Actual state of the IIC state machine */
extern UINT8 gu8IICV1PreviousState; 			/**< Previous state of the IIC state machine */


/** Returns 1 if the IIC module is busy */
#define IIC_IS_BUSY()           (u8IICV1DriverStatus & (1<<IIC_FLAGS_BUSY) )

/** Returns 1 if the data was read ok */
#define IIC_IS_READ_OK()        (u8IICV1DriverStatus & (1<<IIC_FLAGS_READY) )

/** Returns 1 if the data was written ok */
#define IIC_IS_WRITE_OK()       (u8IICV1DriverStatus & (1<<IIC_FLAGS_WRITE_OK) )

/** Returns 1 if the IIC slave is not returning ACK */
#define IIC_IS_NACK()           (u8IICV1DriverStatus & (1<<IIC_FLAGS_ERROR) )

/** Clears the busy flag */
#define IIC_CLEAR_FLAG_BUSY()      ( u8IICV1DriverStatus &= ~(1<<IIC_FLAGS_BUSY)    )

/** Clears the ReadOk flag */
#define IIC_CLEAR_FLAG_READ_OK()   ( u8IICV1DriverStatus &= ~(1<<IIC_FLAGS_READY)   )

/** Clears the WriteOk flag */
#define IIC_CLEAR_FLAG_WRITE_OK()  ( u8IICV1DriverStatus &= ~(1<<IIC_FLAGS_WRITE_OK))

/** Clears the NACK flag */
#define IIC_CLEAR_FLAG_NACK()      ( u8IICV1DriverStatus &= ~(1<<IIC_FLAGS_ERROR)   )


/** This union has all the IIC status flags. */
typedef union {
  UINT8 Byte;
  struct {
    byte IDLE       :1;		/**< The IIC state machine is idle */
    byte BUSY       :1;     /**< The IIC module is busy */                                  
    byte READ_OK    :1;     /**< Data was read correctly */                                  
    byte WRITE_OK   :1;     /**< Data was written correctly */                                  
    byte ERROR      :1;     /**< There is an ACK error */                                  
    byte            :1;                                       
    byte            :1; 
    byte            :1; 
  } Bits;
  struct {
    byte grpDrvStatus :5;
    byte         :1;
    byte         :1;
    byte         :1;
  } MergedBits;
} IICDrvStatus_t;


extern volatile IICDrvStatus_t		_IICDrvStatus;		/**< This variable is used to know the IIC status */
#define u8IICV1DriverStatus			_IICDrvStatus.Byte	/**< This macro defines the IIC status as a byte, as the original code */

typedef enum
{
  IIC_FLAGS_IDLE,
  IIC_FLAGS_BUSY,
  IIC_FLAGS_READY,
  IIC_FLAGS_WRITE_OK,
  IIC_FLAGS_ERROR
} eIICV1DriverStatus;

typedef enum
{
    IICV1_IDLE,
    IICV1_START_W,
    IICV1_START_R,
    IICV1_DATA_W,
    IICV1_DATA_R,
    IICV1_RESTART,
    IICV1_STOP
} eIICV1States;

typedef enum
{
    FLAG_TCF,
    FLAG_ARBL,
    FLAG_RANDOMREAD,
    FLAG_ERROR
}eIICV1Flags;



#endif
