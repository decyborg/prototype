#ifndef __PublicTypes
#define __PublicTypes

typedef unsigned char   UINT8;      //!<unsigned 8 bit definition 
typedef unsigned short  UINT16;     //!<unsigned 16 bit definition
typedef unsigned long   UINT32;     //!<unsigned 32 bit definition
typedef signed char     INT8;       //!<signed 8 bit definition 
typedef int             INT16;      //!<signed 16 bit definition
typedef long int        INT32;      //!<signed 32 bit definition
typedef unsigned char   byte;       //Byte

typedef void (* pFunc_t) (void);			/** Pointer to function that will be used as a callback */

#define WAIT() _asm wait
#define TRUE 	1
#define FALSE	0


#define OUTPUT	1
#define INTPUT	0

#define FLAG_SET(BitNumber, Register)        (Register |=(1<<BitNumber))
#define FLAG_CLR(BitNumber, Register)        (Register &=~(1<<BitNumber))
#define FLAG_CHK(BitNumber, Register)        (Register & (1<<BitNumber))

#define _OUT    1
#define _IN     0
#define ON      0
#define OFF     1


#define _BGND   asm(BGND)
#define _NOP   asm(NOP)

#endif