/**
 *
 * Application qui attend des transferts SPI et affiche sur son UART0
 * les caractères transférés. La fin d'attente est matérialisée par une interruption,
 * ce qui permet de libérer la boucle principale.
 *
 */
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

/**
 *
 * La méthode apellée quand un transfert SPI matériel est terminé,
 * via l'interruption SSI_EOT
 *
 */
void SPIIntHandler(void) {

	unsigned long ulStatus;

	ulStatus = ROM_SSIIntStatus(SSI0_BASE, true);

	// nettoyage de l'interruption
	ROM_SSIIntClear(SSI0_BASE, ulStatus);

	// ce n'est pas le type d'interruption attendu! on signale une erreur
	// en allumant la LED rouge 1 seconde
	if (ulStatus == SSI_RXOR || ulStatus == SSI_RXTO) {
		ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
		ROM_SysCtlDelay((ROM_SysCtlClockGet() / (1000 * 3)) * 1000);
		ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);

	} else {

		// un tranfert est terminé, on va chercher l'octet dans le tampon FIFO
		// et on l'envoie dans l'UART pour affichage sur la console

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
 * Réinitialise l'état des LEDS. Fait par prudence mais pas nécessaire!
 */
void ledReset() {
	ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
	ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
	ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
	ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
	ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
	ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
}

/**
 * Configure les GPIO et pins pour l'expérimentation : LED, SPI et UART.
 *
 */
void gpioConfig() {

	//  leds
	{
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
		ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
		ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
		ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
	}

	//  UART
	{

		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

		ROM_SysCtlDelay(1); //attendre 3 clocks pour etre sur que tout est stable

		ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
		ROM_GPIOPinConfigure(GPIO_PA1_U0TX);

		ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	}

	//  SSI/SPI
	{

		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

		ROM_SysCtlDelay(1); //attendre 3 clocks pour etre sur que tout est stable

		ROM_GPIOPinConfigure(GPIO_PA2_SSI0CLK);
		ROM_GPIOPinConfigure(GPIO_PA3_SSI0FSS);
		ROM_GPIOPinConfigure(GPIO_PA4_SSI0RX);
		ROM_GPIOPinConfigure(GPIO_PA5_SSI0TX);

		ROM_GPIOPinTypeSSI(GPIO_PORTA_BASE,
				GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);

		// Push-pull et weak pull-up pour toutes les lignes SPI
		ROM_GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_PIN_TYPE_STD_WPU);
		ROM_GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_TYPE_STD_WPU);
		ROM_GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_PIN_TYPE_STD_WPU);
		ROM_GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_PIN_TYPE_STD_WPU);

		// purge tampon
		unsigned long ulTemp;
		while (ROM_SSIDataGetNonBlocking(SSI0_BASE, &ulTemp))
		{
		}
	}
}

/**
 *
 * Configure l'interruption EOT
 *
 */
void itConfig() {

	// désactive les interruptions CPU
	IntMasterDisable();

	// déconfigure les interruptions SSI eventuelles
	SSIIntDisable(SSI0_BASE, SSI_TXFF | SSI_RXFF | SSI_RXTO | SSI_RXOR);

	// nettoie les interruptions SSI eventuelles
	SSIIntClear(SSI0_BASE, SSI_TXFF | SSI_RXFF | SSI_RXTO | SSI_RXOR);

	// configure SSI_TXFF pour se lever AUSSI dans le cas d'une fin de transfert
	HWREG(SSI0_BASE + SSI_O_CR1) |= SSI_CR1_EOT;

	// active les interruptions voulues (TXFF/EOT et les erreurs)
	SSIIntEnable(SSI0_BASE, SSI_TXFF | SSI_RXTO | SSI_RXOR);

	// active les interruptions du bloc SSI0
	IntEnable(INT_SSI0);

	// active les interruptions CPU
	IntMasterEnable();
}

int main(void) {

	// 40Mhz avec PLL et xtal 16Mhz
	ROM_SysCtlClockSet(
			SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ
					| SYSCTL_OSC_MAIN);

	// configure les ports et les pins pour les périphériques
	// utilisés : LED, SSI0, UART0
	gpioConfig();

	// configure SSI0 avec MODE0, esclave, 4Mhz, 8bit par transfert
	ROM_SSIConfigSetExpClk(SSI0_BASE, ROM_SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
			SSI_MODE_SLAVE, 2000000, 8);

	// configure UART0 avec 115200 bauds, 8 bits 1 none
	ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,
			(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	// active les périphériques
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

