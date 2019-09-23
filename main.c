#include <msp430.h> 
#include "main.h"
#include "hal_LCD.h"
#include "driverlib/driverlib.h"

/**
 * main.c
 */
int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	  Init_LCD();     //Sets up the LaunchPad LCD display
	return 0;
}
