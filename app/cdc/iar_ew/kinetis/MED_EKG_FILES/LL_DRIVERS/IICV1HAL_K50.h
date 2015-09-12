#ifndef __IICV1HALGB
#define __IICV1HALGB

#include "derivative.h"
#include "types.h"
#include "PublicTypes.h"

//#define USE_I2C0
#define USE_I2C1

#if ((defined USE_I2C0) && (defined USE_I2C1))
#error "Check IICV1HAL_K50.h just one I2C port must be defined"
#endif

/*****************************************************************************
*                          I2C0 Register Redefinitions
******************************************************************************/ 

#if (defined USE_I2C0)

/********************************************
*      Redefinition for I2C0_S Register
*********************************************/

#undef I2C0_S

typedef union{
 byte Byte;
 struct {
   byte RXAK     :1;
   byte I2CIF    :1;
   byte SRW      :1;
   byte RAM      :1;
   byte ARBL     :1;
   byte BUSY     :1;
   byte IAAS     :1;
   byte TCF      :1;
 }Bits;
}I2Cx_S;

__no_init volatile I2Cx_S _I2C0_S @0x40066003;


#define I2C0_S          _I2C0_S.Byte
#define I2C0_S_RXAK     _I2C0_S.Bits.RXAK
#define I2C0_S_I2CIF    _I2C0_S.Bits.I2CIF
#define I2C0_S_SRW      _I2C0_S.Bits.SRW
#define I2C0_S_RAM      _I2C0_S.Bits.RAM
#define I2C0_S_ARBL     _I2C0_S.Bits.ARBL
#define I2C0_S_BUSY     _I2C0_S.Bits.BUSY
#define I2C0_S_IAAS     _I2C0_S.Bits.IAAS
#define I2C0_S_TCF      _I2C0_S.Bits.TCF
/*************************************************/

/********************************************
*      Redefinition for I2C0_C1 Register
*********************************************/

#undef I2C0_C1

typedef union{
 byte Byte;
 struct {
   byte DMAEN    :1;
   byte WUEN     :1;
   byte RSTA     :1;
   byte TXAK     :1;
   byte TX       :1;
   byte MST      :1;
   byte IICIE    :1;
   byte IICEN    :1;
 }Bits;
}I2Cx_C1;

__no_init volatile I2Cx_C1 _I2C0_C1 @0x40066002;

#define I2C0_C1         _I2C0_C1.Byte
#define I2C0_C1_DAMEN   _I2C0_C1.Bits.DMAEN
#define I2C0_C1_WUEN    _I2C0_C1.Bits.WUEN
#define I2C0_C1_RSTA    _I2C0_C1.Bits.RSTA
#define I2C0_C1_TXAK    _I2C0_C1.Bits.TXAK
#define I2C0_C1_TX      _I2C0_C1.Bits.TX
#define I2C0_C1_MST     _I2C0_C1.Bits.MST
#define I2C0_C1_IICIE   _I2C0_C1.Bits.IICIE
#define I2C0_C1_IICEN   _I2C0_C1.Bits.IICEN
/*************************************************/

/********************************************
*      Redefinition for I2C0_F Register
*********************************************/

#undef I2C0_F

typedef union {
  byte Byte;
  struct {
    byte ICR0        :1;                                       /* IIC Clock Rate Bit 0 */
    byte ICR1        :1;                                       /* IIC Clock Rate Bit 1 */
    byte ICR2        :1;                                       /* IIC Clock Rate Bit 2 */
    byte ICR3        :1;                                       /* IIC Clock Rate Bit 3 */
    byte ICR4        :1;                                       /* IIC Clock Rate Bit 4 */
    byte ICR5        :1;                                       /* IIC Clock Rate Bit 5 */
    byte MULT0       :1;                                       /* Multiplier Factor Bit 0 */
    byte MULT1       :1;                                       /* Multiplier Factor Bit 1 */
  } Bits;
  struct {
    byte grpICR  :6;
    byte grpMULT :2;
  } MergedBits;
} I2Cx_F;

__no_init volatile I2Cx_F _I2C0_F @0x40066001;

#define I2C0_F                  _I2C0_F.Byte
#define I2C0_F_ICR0             _I2C0_F.Bits.ICR0
#define I2C0_F_ICR1             _I2C0_F.Bits.ICR1
#define I2C0_F_ICR2             _I2C0_F.Bits.ICR2
#define I2C0_F_ICR3             _I2C0_F.Bits.ICR3
#define I2C0_F_ICR4             _I2C0_F.Bits.ICR4
#define I2C0_F_ICR5             _I2C0_F.Bits.ICR5
#define I2C0_F_MULT0            _I2C0_F.Bits.MULT0
#define I2C0_F_MULT1            _I2C0_F.Bits.MULT1
#define I2C0_F_ICR              _I2C0_F.MergedBits.grpICR
#define I2C0_F_MULT             _I2C0_F.MergedBits.grpMULT
/*************************************************/

/********************************************
*      Redefinition for I2C0_F Register
*********************************************/

#undef I2C0_A1

typedef union {
  byte Byte;
      struct {
        byte             :1; 
        byte AD1         :1;                                       /* Slave Address Bit 1 */
        byte AD2         :1;                                       /* Slave Address Bit 2 */
        byte AD3         :1;                                       /* Slave Address Bit 3 */
        byte AD4         :1;                                       /* Slave Address Bit 4 */
        byte AD5         :1;                                       /* Slave Address Bit 5 */
        byte AD6         :1;                                       /* Slave Address Bit 6 */
        byte AD7         :1;                                       /* Slave Address Bit 7 */
      } Bits;
      struct {
        byte         :1;
        byte grpAD_1 :7;
      } MergedBits;
    } I2Cx_A1;
    
    __no_init volatile I2Cx_A1 _I2C0_A1 @0x40066000;
    
    #define I2C0_A1                    _I2C0_A1.Byte
    #define I2C0_A1_AD1                _I2C0_A1.Bits.AD1
    #define I2C0_A1_AD2                _I2C0_A1.Bits.AD2
    #define I2C0_A1_AD3                _I2C0_A1.Bits.AD3
    #define I2C0_A1_AD4                _I2C0_A1.Bits.AD4
    #define I2C0_A1_AD5                _I2C0_A1.Bits.AD5
    #define I2C0_A1_AD6                _I2C0_A1.Bits.AD6
    #define I2C0_A1_AD7                _I2C0_A1.Bits.AD7
    #define I2C0_A1_REG                _I2C0_A1.MergedBits.grpAD_1

/*************************************************/
    
/********************************************
*      Redefinition for I2C0_D Register
*********************************************/

#undef I2C0_D
    
 typedef union{
   byte Byte;
     struct{
     byte D0  :1;
     byte D1  :1;
     byte D2  :1;
     byte D3  :1;
     byte D4  :1;
     byte D5  :1;
     byte D6  :1;
     byte D7  :1;
     }Bits;
    }I2Cx_D;

    __no_init volatile I2Cx_D _I2C0_D @0x40066004;
    
#define I2C0_D         _I2C0_D.Byte
    
/*************************************************/
    
    

#define IICIF           I2C0_S_I2CIF
#define IIC_ARBL        I2C0_S_ARBL
#define IIC_RXAK        I2C0_S_RXAK
#define IIC_BUSY        I2C0_S_BUSY
#define IIC_IAAS        I2C0_S_IAAS
#define IIC_SLAVE_ADD   I2C0_A1
#define IIC_CLOCK_RATE  I2C0_F_ICR
#define IIC_CLOCK_DIV   I2C0_F_MULT

#define IIC_CLOCK_DIV_VALUE 	0
#define IIC_CLOCK_RATE_VALUE 	0x11	//400kHz
//#define IIC_CLOCK_RATE_VALUE 	0x20	//150kHz

#define IIC_CONTROL_REGISTER I2C0_C1

#define IIC_CONTROL_REGISTER_VALUE (I2C_C1_IICEN_MASK|I2C_C1_IICIE_MASK)

#define IIC_TXAK I2C0_C1_TXAK
#define IIC_TX   I2C0_C1_TX      
#define IIC_MST  I2C0_C1_MST      
#define IIC_RSTA I2C0_C1_RSTA   

#define IIC_DATA_REGISTER  I2C0_D
    
#define SET_PIN_AS_IIC()        SIM_SCGC4 |= SIM_SCGC4_I2C0_MASK;\
                                PORTB_PCR0 = PORT_PCR_MUX(2);\
                                PORTB_PCR1 = PORT_PCR_MUX(2)  
    
#define ENABLE_IIC_INTERRUPT()     NVICICER0|=(1<<24);\
                                   NVICISER0|=(1<<24); 
    
#endif
/*****************************************************************************/
    
/*****************************************************************************
*                          I2C1 Register Redefinitions
******************************************************************************/    
    
#if (defined USE_I2C1)

/********************************************
*      Redefinition for I2C1_S Register
*********************************************/

#undef I2C1_S

typedef union{
 byte Byte;
 struct {
   byte RXAK     :1;
   byte I2CIF    :1;
   byte SRW      :1;
   byte RAM      :1;
   byte ARBL     :1;
   byte BUSY     :1;
   byte IAAS     :1;
   byte TCF      :1;
 }Bits;
}I2Cx_S;

__no_init volatile I2Cx_S _I2C1_S @0x40067003;


#define I2C1_S          _I2C1_S.Byte
#define I2C1_S_RXAK     _I2C1_S.Bits.RXAK
#define I2C1_S_I2CIF    _I2C1_S.Bits.I2CIF
#define I2C1_S_SRW      _I2C1_S.Bits.SRW
#define I2C1_S_RAM      _I2C1_S.Bits.RAM
#define I2C1_S_ARBL     _I2C1_S.Bits.ARBL
#define I2C1_S_BUSY     _I2C1_S.Bits.BUSY
#define I2C1_S_IAAS     _I2C1_S.Bits.IAAS
#define I2C1_S_TCF      _I2C1_S.Bits.TCF
/*************************************************/

/********************************************
*      Redefinition for I2C1_C1 Register
*********************************************/

#undef I2C1_C1

typedef union{
 byte Byte;
 struct {
   byte DMAEN    :1;
   byte WUEN     :1;
   byte RSTA     :1;
   byte TXAK     :1;
   byte TX       :1;
   byte MST      :1;
   byte IICIE    :1;
   byte IICEN    :1;
 }Bits;
}I2Cx_C1;

__no_init volatile I2Cx_C1 _I2C1_C1 @0x40067002;

#define I2C1_C1         _I2C1_C1.Byte
#define I2C1_C1_DAMEN   _I2C1_C1.Bits.DMAEN
#define I2C1_C1_WUEN    _I2C1_C1.Bits.WUEN
#define I2C1_C1_RSTA    _I2C1_C1.Bits.RSTA
#define I2C1_C1_TXAK    _I2C1_C1.Bits.TXAK
#define I2C1_C1_TX      _I2C1_C1.Bits.TX
#define I2C1_C1_MST     _I2C1_C1.Bits.MST
#define I2C1_C1_IICIE   _I2C1_C1.Bits.IICIE
#define I2C1_C1_IICEN   _I2C1_C1.Bits.IICEN
/*************************************************/

/********************************************
*      Redefinition for I2C1_F Register
*********************************************/

#undef I2C1_F

typedef union {
  byte Byte;
  struct {
    byte ICR0        :1;                                       /* IIC Clock Rate Bit 0 */
    byte ICR1        :1;                                       /* IIC Clock Rate Bit 1 */
    byte ICR2        :1;                                       /* IIC Clock Rate Bit 2 */
    byte ICR3        :1;                                       /* IIC Clock Rate Bit 3 */
    byte ICR4        :1;                                       /* IIC Clock Rate Bit 4 */
    byte ICR5        :1;                                       /* IIC Clock Rate Bit 5 */
    byte MULT0       :1;                                       /* Multiplier Factor Bit 0 */
    byte MULT1       :1;                                       /* Multiplier Factor Bit 1 */
  } Bits;
  struct {
    byte grpICR  :6;
    byte grpMULT :2;
  } MergedBits;
} I2Cx_F;

__no_init volatile I2Cx_F _I2C1_F @0x40067001;

#define I2C1_F                  _I2C1_F.Byte
#define I2C1_F_ICR0             _I2C1_F.Bits.ICR0
#define I2C1_F_ICR1             _I2C1_F.Bits.ICR1
#define I2C1_F_ICR2             _I2C1_F.Bits.ICR2
#define I2C1_F_ICR3             _I2C1_F.Bits.ICR3
#define I2C1_F_ICR4             _I2C1_F.Bits.ICR4
#define I2C1_F_ICR5             _I2C1_F.Bits.ICR5
#define I2C1_F_MULT0            _I2C1_F.Bits.MULT0
#define I2C1_F_MULT1            _I2C1_F.Bits.MULT1
#define I2C1_F_ICR              _I2C1_F.MergedBits.grpICR
#define I2C1_F_MULT             _I2C1_F.MergedBits.grpMULT
/*************************************************/

/********************************************
*      Redefinition for I2C1_A1 Register
*********************************************/

#undef I2C1_A1

typedef union {
  byte Byte;
      struct {
        byte             :1; 
        byte AD1         :1;                                       /* Slave Address Bit 1 */
        byte AD2         :1;                                       /* Slave Address Bit 2 */
        byte AD3         :1;                                       /* Slave Address Bit 3 */
        byte AD4         :1;                                       /* Slave Address Bit 4 */
        byte AD5         :1;                                       /* Slave Address Bit 5 */
        byte AD6         :1;                                       /* Slave Address Bit 6 */
        byte AD7         :1;                                       /* Slave Address Bit 7 */
      } Bits;
      struct {
        byte         :1;
        byte grpAD_1 :7;
      } MergedBits;
    } I2Cx_A1;
    
    __no_init volatile I2Cx_A1 _I2C1_A1 @0x40067000;
    
    #define I2C1_A1                    _I2C1_A1.Byte
    #define I2C1_A1_AD1                _I2C1_A1.Bits.AD1
    #define I2C1_A1_AD2                _I2C1_A1.Bits.AD2
    #define I2C1_A1_AD3                _I2C1_A1.Bits.AD3
    #define I2C1_A1_AD4                _I2C1_A1.Bits.AD4
    #define I2C1_A1_AD5                _I2C1_A1.Bits.AD5
    #define I2C1_A1_AD6                _I2C1_A1.Bits.AD6
    #define I2C1_A1_AD7                _I2C1_A1.Bits.AD7
    #define I2C1_A1_REG                _I2C1_A1.MergedBits.grpAD_1

/*************************************************/
    
/********************************************
*      Redefinition for I2C1_D Register
*********************************************/

#undef I2C1_D
    
 typedef union{
   byte Byte;
     struct{
     byte D0  :1;
     byte D1  :1;
     byte D2  :1;
     byte D3  :1;
     byte D4  :1;
     byte D5  :1;
     byte D6  :1;
     byte D7  :1;
     }Bits;
    }I2Cx_D;

    __no_init volatile I2Cx_D _I2C1_D @0x40067004;
    
#define I2C1_D         _I2C1_D.Byte
    
/*************************************************/
    
    

#define IICIF           I2C1_S_I2CIF
#define IIC_ARBL        I2C1_S_ARBL
#define IIC_RXAK        I2C1_S_RXAK
#define IIC_BUSY        I2C1_S_BUSY
#define IIC_IAAS        I2C1_S_IAAS
#define IIC_SLAVE_ADD   I2C1_A1
#define IIC_CLOCK_RATE  I2C1_F_ICR
#define IIC_CLOCK_DIV   I2C1_F_MULT

#define IIC_CLOCK_DIV_VALUE 	0
#define IIC_CLOCK_RATE_VALUE 	0x11	//400kHz
//#define IIC_CLOCK_RATE_VALUE 	0x20	//150kHz

#define IIC_CONTROL_REGISTER I2C1_C1

#define IIC_CONTROL_REGISTER_VALUE (I2C_C1_IICEN_MASK|I2C_C1_IICIE_MASK)

#define IIC_TXAK I2C1_C1_TXAK
#define IIC_TX   I2C1_C1_TX
#define IIC_MST  I2C1_C1_MST
#define IIC_RSTA I2C1_C1_RSTA

#define IIC_DATA_REGISTER  I2C1_D
    
#define SET_PIN_AS_IIC()        SIM_SCGC4 |= SIM_SCGC4_I2C1_MASK;\
                                PORTC_PCR10 = PORT_PCR_MUX(2);\
                                PORTC_PCR11 = PORT_PCR_MUX(2)

#define ENABLE_IIC_INTERRUPT()     NVICICER0|=(1<<25);\
                                   NVICISER0|=(1<<25); 
    
#endif    
/*****************************************************************************/
#endif
