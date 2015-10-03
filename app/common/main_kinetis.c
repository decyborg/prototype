/******************************************************************************
 *
 * CloverTech
 *
 * @file main.c
 *
 * @author       Manuel Rodriguez
 *
 * @version             0.0
 *
 * @date                3 / Sept / 2015
 *
 * @brief   EKG application
 *****************************************************************************/
#include "types.h"
#include "derivative.h" /* include peripheral declarations */
#include "user_config.h"
#include "RealTimerCounter.h"
#include "Wdt_kinetis.h"
#include "TWR_K50_UI.h"
#include "uart.h"

extern uint_32 ___VECTOR_RAM[];            //Get vector table that was copied to RAM

__ramfunc void set_sys_dividers(unsigned int outdiv1, unsigned int outdiv2, unsigned int outdiv3, unsigned int outdiv4);

void ISR_Tracker (void);  //Erase

/*****************************************************************************
 * Global Functions Prototypes
 *****************************************************************************/
#if MAX_TIMER_OBJECTS
extern uint_8 TimerQInitialize(uint_8 ControllerId);
#endif
extern void TestApp_Init(void);
extern void TestApp_Task(void);

#define BSP_CLOCK_SRC                   (8000000ul)       // crystal, oscillator freq
#define BSP_REF_CLOCK_SRC               (2000000ul)       // must be 2-4MHz

#define BSP_CORE_DIV                    (1)
#define BSP_BUS_DIV                     (1)
#define BSP_FLEXBUS_DIV                 (1)
#define BSP_FLASH_DIV                   (2)

// BSP_CLOCK_MUL from interval 24 - 55
#define BSP_CLOCK_MUL                   (24)    // 48MHz

#define BSP_REF_CLOCK_DIV               (BSP_CLOCK_SRC / BSP_REF_CLOCK_SRC)

#define BSP_CLOCK                       (BSP_REF_CLOCK_SRC * BSP_CLOCK_MUL)
#define BSP_CORE_CLOCK                  (BSP_CLOCK / BSP_CORE_DIV)          // CORE CLK, max 100MHz
#define BSP_SYSTEM_CLOCK                (BSP_CORE_CLOCK)                    // SYSTEM CLK, max 100MHz
#define BSP_BUS_CLOCK                   (BSP_CLOCK / BSP_BUS_DIV)       // max 50MHz
#define BSP_FLEXBUS_CLOCK               (BSP_CLOCK / BSP_FLEXBUS_DIV)
#define BSP_FLASH_CLOCK                 (BSP_CLOCK / BSP_FLASH_DIV)     // max 25MHz

/*****************************************************************************
 * Local Functions Prototypes
 *****************************************************************************/
static void Init_Sys(void);
static unsigned char pll_init();
static void wdog_disable(void);
static void StartKeyPressSimulationTimer(void);
static void KeyPressSimulationTimerCallback(void);

/****************************************************************************
 * Global Variables
 ****************************************************************************/
uint_8 kbi_stat_sim;
volatile uint_8 kbi_stat;	   /* Status of the Key Pressed */

unsigned int Fault;


/*****************************************************************************
 * Global Functions
 *****************************************************************************/
/******************************************************************************
 * @name        main
 *
 * @brief       This routine is the starting point of the application
 *
 * @param       None
 *
 * @return      None
 *
 *****************************************************************************
 * This function initializes the system, enables the interrupts and calls the
 * application
 *****************************************************************************/
void main(void)
{   
    Init_Sys();        /* initial the system */
    
#if MAX_TIMER_OBJECTS
    (void)TimerQInitialize(0);
#endif
    (void)TestApp_Init(); /* Initialize the USB Test Application */
    StartKeyPressSimulationTimer();

    while(TRUE)
    {
    	Watchdog_Reset();
       /* Call the application task */
       TestApp_Task();
    }
}

/*****************************************************************************
 * Local Functions
 *****************************************************************************/

/******************************************************************************
 * @name        StartKeyPressSimulationTimer
 *
 * @brief       This routine is used to start the timer for generating key press events
 *              Used on boards without buttons
 *
 * @param       None
 *
 * @return      None
 *
 *****************************************************************************
 *
 *****************************************************************************/
static void StartKeyPressSimulationTimer(void)
{
  TIMER_OBJECT TimerObject;
			
  TimerObject.msCount = KEY_PRESS_SIM_TMR_INTERVAL;
  TimerObject.pfnTimerCallback = KeyPressSimulationTimerCallback;
  (void)AddTimerQ(&TimerObject);
}

/******************************************************************************
 * @name        KeyPressSimulationTimerCallback
 *
 * @brief       This routine is used to generate simulated key press events
 *              It is called at KeyPressSimulationTimer expire time
 *              Used on boards without buttons
 *
 * @param       None
 *
 * @return      None
 *
 *****************************************************************************
 *
 *****************************************************************************/
static void KeyPressSimulationTimerCallback(void)
{
	 /* generate the button press */
	 kbi_stat_sim = (uint_8)((kbi_stat_sim << 1) & (uint_8)0x0F);
	 
	 if(kbi_stat_sim == 0)
	 {
	  kbi_stat_sim = 1;
	 }
	 
	 kbi_stat = kbi_stat_sim;
	 
	 /* re-trigger the timer */
	 StartKeyPressSimulationTimer();
}

/*****************************************************************************
 *
 *    @name     GPIO_Init
 *
 *    @brief    This function Initializes LED GPIO
 *
 *    @param    None
 *
 *    @return   None
 *
 ****************************************************************************
 * Intializes the GPIO
 ***************************************************************************/
void GPIO_Init()
{

}

/******************************************************************************
 * @name       all_led_off
 *
 * @brief      Switch OFF all LEDs on Board
 *
 * @param	   None
 *
 * @return     None
 *
 *****************************************************************************
 * This function switch OFF all LEDs on Board
 *****************************************************************************/
static void all_led_off(void)
{
	GPIOC_PSOR |= 1 << 8 | 1 << 9;
}

/******************************************************************************
 * @name       display_led
 *
 * @brief      Displays 8bit value on Board LEDs
 *
 * @param	   val
 *
 * @return     None
 *
 *****************************************************************************
 * This function displays 8 bit value on Board LED
 *****************************************************************************/
void display_led(uint_8 val)
{
    uint_8 i = 0;
    all_led_off();

	val &= 0x03;
	if(val & 0x1)
		GPIOC_PCOR |= 1 << 8;
	if(val & 0x2)
		GPIOC_PCOR |= 1 << 9;
}
/*****************************************************************************
 *
 *    @name     Init_Sys
 *
 *    @brief    This function Initializes the system
 *
 *    @param    None
 *
 *    @return   None
 *
 ****************************************************************************
 * Intializes the MCU, MCG, KBI, RTC modules
 ***************************************************************************/
static void Init_Sys(void)
{

	  uint_32 i,n;
	  /* Point the VTOR to the new copy of the vector table */                  
        SCB_VTOR = (uint_32)___VECTOR_RAM;
		
	NVICICER2|=(1<<9);                     //Clear any pending interrupts on USB
	NVICISER2|=(1<<9);                     //Enable interrupts from USB module 
        
        // Configure the TX and RX pins for UART0
	SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
	PORTD_PCR6 = PORT_PCR_MUX(0x03);
	PORTD_PCR7 = PORT_PCR_MUX(0x03);
        
        // Configure the TX and RX pins for UART4
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
	PORTC_PCR16 = PORT_PCR_MUX(0x03);
	PORTC_PCR17 = PORT_PCR_MUX(0x03);

    /* SIM Configuration */
	GPIO_Init();
	pll_init();
        uart_init(TERM_PORT, 48000, 115200);
        send_string("CloverTech", TERM_PORT);
        uart_init(BL_PORT, 48000, 115200);
        send_string("CloverTech", BL_PORT);
        //configure_BL();
        
    MPU_CESR=0x00;
    
    /************* USB Part **********************/
    /*********************************************/   
    // PLL/FLL selected as CLK source
    SIM_CLKDIV2 &= SIM_CLKDIV2_USBFRAC_MASK | SIM_CLKDIV2_USBDIV_MASK;
    SIM_CLKDIV2|= SIM_CLKDIV2_USBDIV(0);
    
    SIM_SCGC4|=(SIM_SCGC4_USBOTG_MASK);             // USB Clock Gating
    SIM_SOPT2  |= SIM_SOPT2_USBSRC_MASK | SIM_SOPT2_PLLFLLSEL_MASK;
    SIM_SOPT1 |= SIM_SOPT1_USBREGEN_MASK;

}

/*****************************************************************************
 * @name     wdog_disable
 *
 * @brief:   Disable watchdog.
 *
 * @param  : None
 *
 * @return : None
 *****************************************************************************
 * It will disable watchdog.
  ****************************************************************************/
static void wdog_disable(void)
{
	/* Write 0xC520 to the unlock register */
	WDOG_UNLOCK = 0xC520;
	
	/* Followed by 0xD928 to complete the unlock */
	WDOG_UNLOCK = 0xD928;
	
	/* Clear the WDOGEN bit to disable the watchdog */
	WDOG_STCTRLH &= ~WDOG_STCTRLH_WDOGEN_MASK;
}
/*****************************************************************************
 * @name     pll_init
 *
 * @brief:   Initialization of the MCU.
 *
 * @param  : None
 *
 * @return : None
 *****************************************************************************
 * It will configure the MCU to disable STOP and COP Modules.
 * It also set the MCG configuration and bus clock frequency.
 ****************************************************************************/
static unsigned char pll_init()
{      
  //This assumes that the MCG is in default FEI mode out of reset.
  
  //K-60 Board Only
  
// First move to FBE mode
    //MCG_C2 = 0;
    
// Enable external oscillator, RANGE=2, HGO=1, EREFS=1, LP=0, IRCS=0
    MCG_C2 = MCG_C2_RANGE(2) | MCG_C2_HGO_MASK;// | MCG_C2_EREFS_MASK;

// after initialization of oscillator release latched state of oscillator and GPIO
    SIM_SCGC4 |= SIM_SCGC4_LLWU_MASK;
    LLWU_CS |= LLWU_CS_ACKISO_MASK;
    
// Select external oscilator and Reference Divider and clear IREFS to start ext osc
// CLKS=2, FRDIV=3, IREFS=0, IRCLKEN=0, IREFSTEN=0
  MCG_C1 = MCG_C1_CLKS(2) | MCG_C1_FRDIV(3);
    
  while (MCG_S & MCG_S_IREFST_MASK){}; // wait for Reference clock Status bit to clear

  while (((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) != 0x2){}; // Wait for clock status bits to show clock source is ext ref clk

// Now in FBE

   MCG_C5 = MCG_C5_PRDIV(0x18);

  // Ensure MCG_C6 is at the reset default of 0. LOLIE disabled, PLL disabled, clk monitor disabled, PLL VCO divider is clear
   MCG_C6 = 0x0;

      // Set system options dividers
      //MCG=PLL, core = MCG, bus = MCG, FlexBus = MCG, Flash clock= MCG/2
      set_sys_dividers(0,0,0,1);
      // Set the VCO divider and enable the PLL for 48MHz, LOLIE=0, PLLS=1, CME=0, VDIV=0
      MCG_C6 = MCG_C6_PLLS_MASK; //VDIV = 0 (x24)

  while (!(MCG_S & MCG_S_PLLST_MASK)){}; // wait for PLL status bit to set

  while (!(MCG_S & MCG_S_LOCK_MASK)){}; // Wait for LOCK bit to set

// Now running PBE Mode

// Transition into PEE by setting CLKS to 0
// CLKS=0, FRDIV=3, IREFS=0, IRCLKEN=0, IREFSTEN=0
  MCG_C1 &= ~MCG_C1_CLKS_MASK;

// Wait for clock status bits to update
  while (((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) != 0x3){};

// Now running PEE Mode

    return 0;
} //pll_init

/* EOF */


void ISR_Tracker (void)
{
  Fault = SCB_SHCSR;
   asm ("NOP");
}

__ramfunc void set_sys_dividers(unsigned int outdiv1, unsigned int outdiv2, unsigned int outdiv3, unsigned int outdiv4)
{
 /*
  * This routine must be placed in RAM. It is a workaround for errata e2448.
  * Flash prefetch must be disabled when the flash clock divider is changed.
  * This cannot be performed while executing out of flash.
  * There must be a short delay after the clock dividers are changed before prefetch
  * can be re-enabled.
  */
  unsigned int temp_reg;
  unsigned char i;
  
  temp_reg = FMC_PFAPR; // store present value of FMC_PFAPR
  
  // set M0PFD through M7PFD to 1 to disable prefetch
  FMC_PFAPR |= FMC_PFAPR_M7PFD_MASK | FMC_PFAPR_M6PFD_MASK | FMC_PFAPR_M5PFD_MASK
             | FMC_PFAPR_M4PFD_MASK | FMC_PFAPR_M3PFD_MASK | FMC_PFAPR_M2PFD_MASK
             | FMC_PFAPR_M1PFD_MASK | FMC_PFAPR_M0PFD_MASK;
  
  // set clock dividers to desired value  
  SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(outdiv1) | SIM_CLKDIV1_OUTDIV2(outdiv2) 
              | SIM_CLKDIV1_OUTDIV3(outdiv3) | SIM_CLKDIV1_OUTDIV4(outdiv4);

  // wait for dividers to change
  for (i = 0 ; i < outdiv4 ; i++)
  {}
  
  FMC_PFAPR = temp_reg; // re-store original value of FMC_PFAPR
  
  return;
} // set_sys_dividers