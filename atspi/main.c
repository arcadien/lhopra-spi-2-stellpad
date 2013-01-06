#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ssi.h"
#include "inc/lm4f120h5qr.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/ssi.h"
#include "driverlib/rom.h"

#include "utils/uartstdio.h"

#include <stdio.h>
#include <string.h>

void SPIIntHandler(void) {

	unsigned long ulStatus;

	ulStatus = ROM_SSIIntStatus(SSI0_BASE, true);

	ROM_SSIIntClear(SSI0_BASE, ulStatus);

	if (ulStatus == SSI_RXOR || ulStatus == SSI_RXTO) {

		// error : 1s red led, 1 sec
		ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);

		ROM_SysCtlDelay((ROM_SysCtlClockGet() / (1000 * 3)) * 1000);

		// shut down red led after 1sec
		ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);

	} else {

		unsigned long read_value;

		long status = ROM_SSIDataGetNonBlocking(SSI0_BASE, &read_value);
		if (status != 0) {
			ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0xFF);
			while (status != 0) {

				UARTprintf("> got %c  \n", read_value);

				status = ROM_SSIDataGetNonBlocking(SSI0_BASE, &read_value);

			}
			ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
		}

	}

}

/*
 *Cycle power on and off for each 3 rgb leds to reset their status
 */
void ledReset() {
	ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
	ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
	ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
	ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
	ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
	ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
}

void gpioConfig() {
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
		ROM_GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_PIN_TYPE_STD_WPU);
		ROM_GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_TYPE_STD_WPU);
		ROM_GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_PIN_TYPE_STD_WPU);
		ROM_GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_PIN_TYPE_STD_WPU);

		// purge buffer
		unsigned long ulTemp;
		while (ROM_SSIDataGetNonBlocking(SSI0_BASE, &ulTemp)) /* empty buffer */
		{
		}
	}
}

void itConfig() {
	// enable interrupts on receive

	IntMasterDisable();
	SSIIntDisable(SSI0_BASE, SSI_TXFF | SSI_RXFF | SSI_RXTO | SSI_RXOR);
	SSIIntClear(SSI0_BASE, SSI_TXFF | SSI_RXFF | SSI_RXTO | SSI_RXOR);

	HWREG(SSI0_BASE + SSI_O_CR1) |= SSI_CR1_EOT; /* switch tx interrupt to eot int */

	SSIIntEnable(SSI0_BASE, SSI_TXFF);

	IntEnable(INT_SSI0);

	IntMasterEnable();
}

int main(void) {

	// 40Mhz off PLL
	ROM_SysCtlClockSet(
			SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ
					| SYSCTL_OSC_MAIN);

	// GPIO configuration
	gpioConfig();

	// configure as of MODE0, Slave, 4Mhz, 8bit transfer length
	ROM_SSIConfigSetExpClk(SSI0_BASE, ROM_SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
			SSI_MODE_SLAVE, 2000000, 8);

	// 115200 bauds, 8 bits 1 none
	// wait for tranfer to finish before clock alteration
	while (UARTBusy(UART0_BASE)) {
	}
	ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,
			(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	ROM_SSIEnable(SSI0_BASE);
	ROM_UARTEnable(UART0_BASE);

	UARTStdioInit(0);

	UARTprintf("Clock configured at %u Mhz\n", ROM_SysCtlClockGet() / 1000000);

	ledReset();

	itConfig();

	UARTprintf("Ready.\n");

	for (;;) {
	}
}

