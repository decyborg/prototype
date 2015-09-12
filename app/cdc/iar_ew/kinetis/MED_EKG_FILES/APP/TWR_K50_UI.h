/******************************************************************************
 *
 * Freescale Semiconductor Inc.
 * (c) Copyright 2004-2011 Freescale Semiconductor, Inc.
 * ALL RIGHTS RESERVED.
 *
 ******************************************************************************
 *
 * THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR 
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  
 * IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************//*!
 *
 * @file TWR_K50_UI.h
 *
 * @author 
 *
 * @version 1.0 for IAR
 *
 * @date Jan-12-2011
 *
 * @brief The file contains K50-TWR Board UI configurations (LEDs and Buttons).
 *
 *****************************************************************************/

#include "derivative.h"
//#include "common.h"

/*****************************************************************************
 *                               Definitions
 *****************************************************************************/
#define GPIOS_CLOCK_INIT     SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK |SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK |SIM_SCGC5_PORTE_MASK);

/*****************************************************************************
 *                             LED1 Definitions 
 *****************************************************************************/

#define LED1_NUMBER_IN_PORT     7
#define LED1_PCR                PORTC_PCR7
#define LED1_DDR_REG            GPIOC_PDDR
#define LED1_SET_REG            GPIOC_PSOR
#define LED1_CLR_REG            GPIOC_PCOR
#define LED1_TGL_REG            GPIOC_PTOR 

#define LED1_INIT        LED1_PCR = PORT_PCR_MUX(1); \
                         LED1_DDR_REG |= (1<<LED1_NUMBER_IN_PORT);\
                         LED1_SET_REG |= (1<<LED1_NUMBER_IN_PORT);

#define LED1_ON          LED1_CLR_REG |= (1<<LED1_NUMBER_IN_PORT); //LED sets with logical 0 on port
#define LED1_OFF         LED1_SET_REG |= (1<<LED1_NUMBER_IN_PORT); //LED resets with logical 1 on port
#define LED1_TOOGLE      LED1_TGL_REG |= (1<<LED1_NUMBER_IN_PORT); //LED toggles writing a logical 1 on TOR REG

/*****************************************************************************
 *                             LED2 Definitions 
 *****************************************************************************/

#define LED2_NUMBER_IN_PORT     8
#define LED2_PCR                PORTC_PCR8
#define LED2_DDR_REG            GPIOC_PDDR
#define LED2_SET_REG            GPIOC_PSOR
#define LED2_CLR_REG            GPIOC_PCOR
#define LED2_TGL_REG            GPIOC_PTOR 

#define LED2_INIT        LED2_PCR = PORT_PCR_MUX(1); \
                         LED2_DDR_REG |= (1<<LED2_NUMBER_IN_PORT);\
                         LED2_SET_REG |= (1<<LED2_NUMBER_IN_PORT);

#define LED2_ON          LED2_CLR_REG |= (1<<LED2_NUMBER_IN_PORT); //LED sets with logical 0 on port
#define LED2_OFF         LED2_SET_REG |= (1<<LED2_NUMBER_IN_PORT); //LED resets with logical 1 on port
#define LED2_TOOGLE      LED2_TGL_REG |= (1<<LED2_NUMBER_IN_PORT); //LED toggles writing a logical 1 on TOR REG

/*****************************************************************************
 *                             LED3 Definitions 
 *****************************************************************************/
/* TWR-K53 does not have LED 3

#define LED3_NUMBER_IN_PORT     29
#define LED3_PCR                PORTA_PCR29
#define LED3_DDR_REG            GPIOA_PDDR
#define LED3_SET_REG            GPIOA_PSOR
#define LED3_CLR_REG            GPIOA_PCOR
#define LED3_TGL_REG            GPIOA_PTOR 

#define LED3_INIT        LED3_PCR = PORT_PCR_MUX(1); \
                         LED3_DDR_REG |= (1<<LED3_NUMBER_IN_PORT);\
                         LED3_SET_REG |= (1<<LED3_NUMBER_IN_PORT);

#define LED3_ON          LED3_CLR_REG |= (1<<LED3_NUMBER_IN_PORT); //LED sets with logical 0 on port
#define LED3_OFF         LED3_SET_REG |= (1<<LED3_NUMBER_IN_PORT); //LED resets with logical 1 on port
#define LED3_TOOGLE      LED3_TGL_REG |= (1<<LED3_NUMBER_IN_PORT); //LED toggles writing a logical 1 on TOR REG

/*****************************************************************************
 *                             LED4 Definitions 
 *****************************************************************************/
/* TWR-K53 does not have LED 4

#define LED4_NUMBER_IN_PORT     10
#define LED4_PCR                PORTA_PCR10
#define LED4_DDR_REG            GPIOA_PDDR
#define LED4_SET_REG            GPIOA_PSOR
#define LED4_CLR_REG            GPIOA_PCOR
#define LED4_TGL_REG            GPIOA_PTOR 

#define LED4_INIT        LED4_PCR = PORT_PCR_MUX(1); \
                         LED4_DDR_REG |= (1<<LED4_NUMBER_IN_PORT);\
                         LED4_SET_REG |= (1<<LED4_NUMBER_IN_PORT);

#define LED4_ON          LED4_CLR_REG |= (1<<LED4_NUMBER_IN_PORT); //LED sets with logical 0 on port
#define LED4_OFF         LED4_SET_REG |= (1<<LED4_NUMBER_IN_PORT); //LED resets with logical 1 on port
#define LED4_TOOGLE      LED4_TGL_REG |= (1<<LED4_NUMBER_IN_PORT); //LED toggles writing a logical 1 on TOR REG

/*****************************************************************************
 *                           Push Button 1 Definitions 
 *****************************************************************************/

#define PB1_NUMBER_IN_PORT     5
#define PB1_PCR                PORTC_PCR5
#define PB1_DDR_REG            GPIOC_PDDR
#define PB1_DIR_REG            GPIOC_PDIR

#define PB1_INIT         PB1_PCR = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;\
                         PB1_DDR_REG &= (~(1 << PB2_NUMBER_IN_PORT));

#define PB1_STATUS      (PB1_DIR_REG & (1<<PB1_NUMBER_IN_PORT))

/*****************************************************************************
 *                           Push Button 2 Definitions 
 *****************************************************************************/

#define PB2_NUMBER_IN_PORT     13
#define PB2_PCR                PORTC_PCR13
#define PB2_DDR_REG            GPIOC_PDDR
#define PB2_DIR_REG            GPIOC_PDIR

#define PB2_INIT         PB2_PCR =  PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK ;\
                         PB2_DDR_REG &= (~(1 << PB2_NUMBER_IN_PORT));

#define PB2_STATUS       (PB2_DIR_REG & (1<<PB2_NUMBER_IN_PORT))

/*****************************************************************************/

#define INIT_ALL_LEDS           LED1_INIT\
                                LED2_INIT//\
                                //LED3_INIT\
                                //LED4_INIT

#define INIT_ALL_PBS            PB1_INIT\
                                PB2_INIT