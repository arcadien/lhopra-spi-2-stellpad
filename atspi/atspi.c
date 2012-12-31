//*****************************************************************************
//
// uart_echo.c - Example for reading data from and writing data to the UART in
//               an interrupt driven fashion.
//
// Copyright (c) 2012 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 9453 of the EK-LM4F120XL Firmware Package.
//
//*****************************************************************************

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/ssi.h"

#include "utils/uartstdio.h"
#include <stdio.h>
#include <string.h>
//#include "string.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>UART Echo (uart_echo)</h1>
//!
//! This example application utilizes the UART to echo text.  The first UART
//! (connected to the USB debug virtual serial port on the evaluation board)
//! will be configured in 115,200 baud, 8-n-1 mode.  All characters received on
//! the UART are transmitted back to the UART.
//
//*****************************************************************************

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif

readSpi() {

	unsigned long read_value;
	// other ssi status, green flash

	long status = SSIDataGetNonBlocking(SSI0_BASE, &read_value);
	while (status != 0) {
		ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0xFF);
		UARTprintf("> got %c  \n", read_value);

		status = SSIDataGetNonBlocking(SSI0_BASE, &read_value);

	}
	ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
}

//*****************************************************************************
//
// The SSI/SPI interrupt handler.
//
//*****************************************************************************
void SPIIntHandler(void) {

	// interruption is fired when half RX FIFO buffer is filled. As
	// FIFO buffer is 8x16 bits, it triggers when 4 char are transferred.
	// To display them in "real time", a register read must be implemented
	// in the main loop.

//	unsigned long ulStatus;
//
//	ulStatus = ROM_SSIIntStatus(SSI0_BASE, true);
//
//	ROM_SSIIntClear(SSI0_BASE, ulStatus);
//
//	if (ulStatus == SSI_RXOR || ulStatus == SSI_RXTO) {
//
//		// error : 1s red led, 1 sec
//		ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
//		ROM_SysCtlDelay((ROM_SysCtlClockGet() / (1000 * 3)) * 1000);
//		// shut down red led after 1sec
//		ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
//
//	} else {
//
//		while (SSIBusy(SSI0_BASE))
//			; // wait until transfer is pumped in buffer
//
//		readSpi();
//	}

}

//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void USB_UARTIntHandler(void) {
	unsigned long ulStatus;

	ulStatus = ROM_UARTIntStatus(UART0_BASE, true);

//
// Clear the asserted interrupts.
//
	ROM_UARTIntClear(UART0_BASE, ulStatus);

//
// Loop while there are characters in the receive FIFO.
//
	while (ROM_UARTCharsAvail(UART0_BASE)) {
		//
		// Read the next character from the UART and write it back to the UART.
		//
		unsigned long c = ROM_UARTCharGetNonBlocking(UART0_BASE);

		ROM_UARTCharPutNonBlocking(UART0_BASE, c);

		ROM_UARTCharPut(UART1_BASE, c);
		//
		// Blink the  LED to show a character transfer is occuring.
		//
		ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

		//
		// Delay for 1 millisecond.  Each SysCtlDelay is about 3 clocks.
		//
		ROM_SysCtlDelay(SysCtlClockGet() / (1000 * 3));

		//
		// Turn off the LED
		//
		ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

	}
}

void BT_UARTIntHandler(void) {
	unsigned long ulStatus;

	ulStatus = ROM_UARTIntStatus(UART1_BASE, true);

//
// Clear the asserted interrupts.
//
	ROM_UARTIntClear(UART1_BASE, ulStatus);

//
// Loop while there are characters in the receive FIFO.
//
	while (ROM_UARTCharsAvail(UART1_BASE)) {
		//
		// Read the next character from the UART and write it back to the UART.
		//
		ROM_UARTCharPutNonBlocking(UART0_BASE,
				ROM_UARTCharGetNonBlocking(UART1_BASE));

		//
		// Blink the  LED to show a character transfer is occuring.
		//
		ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

		//
		// Delay for 1 millisecond.  Each SysCtlDelay is about 3 clocks.
		//
		ROM_SysCtlDelay(SysCtlClockGet() / (1000 * 3));

		//
		// Turn off the LED
		//
		ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

	}
}

void enablePeripherals() {

	// for leds
	{
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
		ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
		ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
		ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
	}

	// for UART
	{

		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

		ROM_SysCtlDelay(1); // wait 3 clocks to be sure everything is stabilized.

		ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
		ROM_GPIOPinConfigure(GPIO_PA1_U0TX);

		ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

		// interrupts
		ROM_IntEnable(INT_UART0);
		ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
	}

	// for BlueTooth UART
	{
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

		ROM_SysCtlDelay(1); // wait 3 clocks to be sure everything is stabilized.

		ROM_GPIOPinConfigure(GPIO_PB0_U1RX);
		ROM_GPIOPinConfigure(GPIO_PB1_U1TX);
		ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

		// interrupts
		ROM_IntEnable(INT_UART1);
		ROM_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);
	}

	// for SSI/SPI
	{

		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

		ROM_SysCtlDelay(1); // wait 3 clocks to be sure everything is stabilized.

		ROM_GPIOPinConfigure(GPIO_PA2_SSI0CLK);
		ROM_GPIOPinConfigure(GPIO_PA3_SSI0FSS);
		ROM_GPIOPinConfigure(GPIO_PA4_SSI0RX);
		ROM_GPIOPinConfigure(GPIO_PA5_SSI0TX);

		ROM_GPIOPinTypeSSI(GPIO_PORTA_BASE,
				GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);

		// Push-pull with weak pull-up for every SPI lines
		GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_PIN_TYPE_STD_WPU);
		GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_TYPE_STD_WPU);
		GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_PIN_TYPE_STD_WPU);
		GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_PIN_TYPE_STD_WPU);

		// disable DMA
		//ROM_SSIDMADisable(SSI0_BASE, 0xFF);

		// purge buffer
		unsigned long ulTemp;
		while (ROM_SSIDataGetNonBlocking(SSI0_BASE, &ulTemp)) /* empty buffer */
		{
		}

		// enable interrupts on receive
		ROM_SSIIntEnable(SSI0_BASE, SSI_RXFF); /* SSI_TXFF | SSI_RXFF | SSI_RXTO | SSI_RXOR  */
		ROM_IntEnable(INT_SSI0);
	}

}

/*
 * reconfigure clocked peripherals when clock speed changes (free hook)
 */
void reconfigurePeripheralsAfterClockChange() {

	ROM_SSIDisable(SSI0_BASE);

	// configure as of MODE0, Slave, 4Mhz, 8bit length
	ROM_SSIConfigSetExpClk(SSI0_BASE, ROM_SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
			SSI_MODE_SLAVE, 2000000, 8);

	ROM_SSIEnable(SSI0_BASE);

	// UART

	// USB link
	ROM_UARTDisable(UART0_BASE);

	// 115200 bauds, 8 bits 1 none
	ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,
			(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	ROM_UARTEnable(UART0_BASE);

	// Bluetooth link
	ROM_UARTDisable(UART1_BASE);

	// 115200 bauds, 8 bits 1 none
	ROM_UARTConfigSetExpClk(UART1_BASE, ROM_SysCtlClockGet(), 9600,
			(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	ROM_UARTEnable(UART1_BASE);

	UARTStdioInit(0);
}

/*
 *Cycle power on and off for each 3 rgb leds to reset their status
 */
void ledReset() {
	ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x1);
	ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x1);
	ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x1);
	ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
	ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
	ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
}

/*
 * to call before a clock switch
 *
 */
void beforeClockChangeHook() {
	// flush UART buffers, to avoid
	// buggy data to be sent if frequency changes
	// while buffers are not cleared.
	// only if uart is declared buffered
#if defined(UART_BUFFERED)
	UARTFlushTx(false);
#endif
}

void configSpeed(unsigned long ulConfig) {

	UARTprintf("Current speed is %u Mhz\n", ROM_SysCtlClockGet() / 1000000);

	// wait for tranfer to finish before clock alteration
	while (UARTBusy(UART0_BASE)) {
	}

	beforeClockChangeHook();

	ROM_SysCtlClockSet(ulConfig);

	// sleep to let PLL settle
	// ( 3 clock cycles per loop)
	// 30 cycles
	ROM_SysCtlDelay(10);

	// reconfigure peripherals
	reconfigurePeripheralsAfterClockChange();

	UARTprintf("Clock changed to %u Mhz, periph. reconfigured.\n",
			ROM_SysCtlClockGet() / 1000000);
	ROM_SysCtlDelay(10);
}

//*****************************************************************************
//
// This example demonstrates how to send a string of data to the UART.
//
//*****************************************************************************
int main(void) {

// Enable lazy stacking for interrupt handlers.  This allows floating-point
// instructions to be used within interrupt handlers, but at the expense of
// extra stack usage.
//

// clock init to 16Mhz
	ROM_SysCtlClockSet(
			SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN
					| SYSCTL_XTAL_16MHZ);

	ROM_FPUEnable();
	ROM_FPULazyStackingEnable();

	enablePeripherals();

	reconfigurePeripheralsAfterClockChange();

	ROM_IntMasterEnable();

// lib init

	UARTprintf("Init speed: %u Mhz\n", ROM_SysCtlClockGet() / 1000000);

// 40Mhz off PLL
	configSpeed(
			SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ
					| SYSCTL_OSC_MAIN);

// 50Mhz, pll SYSCTL_XTAL_16MHZ
//	configSpeed(
//			SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN
//					| SYSCTL_XTAL_16MHZ);

// full speed test
//	configSpeed(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ);

	ledReset();

	UARTprintf("Enter text: \n");

	while (1) {

		readSpi();
	}
}

