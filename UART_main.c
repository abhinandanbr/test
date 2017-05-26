//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
//
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//*****************************************************************************
//
// Application Name     - Blinky
// Application Overview - The objective of this application is to showcase the
//                        GPIO control using Driverlib api calls. The LEDs
//                        connected to the GPIOs on the LP are used to indicate
//                        the GPIO output. The GPIOs are driven high-low
//                        periodically in order to turn on-off the LEDs.
// Application Details  -
// http://processors.wiki.ti.com/index.php/CC32xx_Blinky_Application
// or
// docs\examples\CC32xx_Blinky_Application.pdf
//
//*****************************************************************************

//****************************************************************************
//
//! \addtogroup blinky
//! @{
//
//****************************************************************************

// Standard includes
#include <stdio.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "interrupt.h"
#include "hw_apps_rcm.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "gpio.h"
#include "uart.h"
#include "utils.h"
#include "pin.h"

// Common interface includes
#include "gpio_if.h"
#include "uart_if.h"


unsigned int i=0;
unsigned int n=0;
#define APPLICATION_VERSION     "1.1.1"

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
char *enter="Enter a\n";


#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************


//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//*****************************************************************************

static void BoardInit(void);

//*****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS
//*****************************************************************************

//*****************************************************************************

void PinMux(){
    // Enable Peripheral Clocks
    	MAP_PRCMPeripheralClkEnable(PRCM_GPIOA1, PRCM_RUN_MODE_CLK); //Clock for GPIO
    	MAP_PRCMPeripheralClkEnable(PRCM_UARTA0, PRCM_RUN_MODE_CLK); //Clock for UART
    	MAP_PRCMPeripheralClkEnable(PRCM_GPIOA2, PRCM_RUN_MODE_CLK); //Clock for UART

    // Configure PIN_64 for GPIO Output
    	MAP_PinTypeGPIO(PIN_64, PIN_MODE_0, false);
    	MAP_GPIODirModeSet(GPIOA1_BASE, 0x2, GPIO_DIR_MODE_OUT);

    // Configure SW2 for Switch Input
        MAP_PinTypeGPIO(PIN_15, PIN_MODE_0, false);
        MAP_GPIODirModeSet(GPIOA2_BASE, 0x40, GPIO_DIR_MODE_IN);

    //Configure for UART Pin 55 TX
    	MAP_PinTypeUART(PIN_55,PIN_MODE_3);

    //Configure for UART Pin 57 RX
    	MAP_PinTypeUART(PIN_57,PIN_MODE_3);
}

 void UARTInt(){

		char character;
//		char print;
//		unsigned int i=0;
//		i=0;

		MAP_UARTIntClear(UARTA0_BASE,UART_INT_TX);
		MAP_UARTCharGet(UARTA0_BASE);
		MAP_UARTIntClear(UARTA0_BASE,UART_INT_RX);
		/*while(i==0){
		character=MAP_UARTCharGet(UARTA0_BASE);
		if(character=='\r' || character=='\n'){
			MAP_UARTCharPut(UARTA0_BASE,print);
			MAP_UARTIntClear(UARTA0_BASE,UART_INT_TX);
			i=1;
		}
		else
		{
			print=character;
		}
		}*/

}

 void GPIOInt(){


	MAP_GPIOIntClear(GPIOA1_BASE,GPIO_PIN_1);

	MAP_GPIOPinWrite(GPIOA1_BASE,0x2,0x2);


//	Message("Config\n");

}

void Switch(){

	MAP_GPIOIntClear(GPIOA2_BASE,GPIO_PIN_6);
	MAP_GPIOPinWrite(GPIOA1_BASE,0x2,0x0);
	MAP_UARTCharPut(UARTA0_BASE,*"X\n\0");
//    enter[9]='\0';
//    while(enter[n]!='\0')
//    {
//    MAP_UARTCharPut(UARTA0_BASE,enter[n++]); /*Print on Terminal
//    Until null character detected*/ /*This generates UART TX Interrupt*/
//    }
//    n=0;


}

void InterruptConfig(void)
{

	//Enable INT for Switch SW2
	MAP_GPIOIntTypeSet(GPIOA2_BASE,GPIO_PIN_6,GPIO_FALLING_EDGE);
	MAP_GPIOIntRegister(GPIOA2_BASE,Switch);
	MAP_IntEnable(INT_GPIOA2);
	MAP_IntPrioritySet(INT_GPIOA2,INT_PRIORITY_LVL_1);
	MAP_GPIOIntEnable(GPIOA2_BASE,GPIO_PIN_6);
	MAP_GPIOIntClear(GPIOA2_BASE,GPIO_PIN_6);

	//Enable and register Interrupt for UART RX
	MAP_UARTEnable(UARTA0_BASE);
	MAP_UARTIntRegister(UARTA0_BASE,UARTInt);
	MAP_UARTFIFODisable(UARTA0_BASE);
	MAP_IntPrioritySet(INT_UARTA0,INT_PRIORITY_LVL_2);
	MAP_IntEnable(INT_UARTA0);

	MAP_UARTIntClear(UARTA0_BASE,UART_INT_TX); /* Clear Interrupt*/
	MAP_UARTIntEnable(UARTA0_BASE,UART_INT_TX); /* Enable UART Port Interrupt*/


	//Enable and register Interrupt for GPIO  Table 5-14. GPIO Mapping
	MAP_GPIOIntTypeSet(GPIOA1_BASE,GPIO_PIN_1,GPIO_LOW_LEVEL);
	MAP_GPIOIntRegister(GPIOA1_BASE,GPIOInt);
	MAP_IntEnable(INT_GPIOA1);
	MAP_IntPrioritySet(INT_GPIOA1,INT_PRIORITY_LVL_3);
	MAP_GPIOIntClear(GPIOA1_BASE,GPIO_PIN_1);
	MAP_GPIOIntEnable(GPIOA1_BASE,GPIO_INT_PIN_1);


}
//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
    //
    // Set vector table base
    //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif

    //
    // Enable Processor

    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}
//****************************************************************************
//
//! Main function
//!
//! \param none
//!
//! This function
//!    1. Invokes the LEDBlinkyTask
//!
//! \return None.
//
//****************************************************************************
int
main()
{
    // Initialize Board configurations

    BoardInit();

    // Power on the corresponding GPIO port B for 9,10,11.
    // Set up the GPIO lines to mode 0 (GPIO)

    //GPIO_IF_LedConfigure(LED1|LED2|LED3);

    //GPIO_IF_LedOff(MCU_ALL_LED_IND);

    PinMux();

    //Set Sw2 Interrupt
    MAP_GPIOIntTypeSet(GPIOA2_BASE,GPIO_PIN_6,GPIO_FALLING_EDGE);
    //Set LED off
    MAP_GPIOPinWrite(GPIOA1_BASE,0x2,0x2);

    //Initialize Terminal
    MAP_UARTConfigSetExpClk(CONSOLE,MAP_PRCMPeripheralClockGet(CONSOLE_PERIPH),
                          UART_BAUD_RATE, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                           UART_CONFIG_PAR_NONE));

    //Configure Interrupt
    InterruptConfig();


    while(1)
    {

    	//MAP_GPIOPinWrite(GPIOA1_BASE,0x2,0x2);

    }
    // Start the LEDBlinkyRoutine
    //
    //LEDBlinkyRoutine();

}

