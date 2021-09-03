#ifndef RTC_AVR128DA64_H_
#define RTC_AVR128DA64_H_

#include <stdbool.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <AVR128DA64/UART_1_AVR128DA64.h>

unsigned long millis;

/************************************************************************/
/* Initialization
Before enabling the RTC peripheral and the desired actions (interrupt requests and output events), the source clock
for the RTC counter must be configured to operate the RTC.

Configure the Clock CLK_RTC
To configure the CLK_RTC, follow these steps:
1. Configure the desired oscillator to operate as required, in the Clock Controller (CLKCTRL) peripheral.
2. Write the Clock Select (CLKSEL) bit field in the Clock Selection (RTC.CLKSEL) register accordingly.
The CLK_RTC clock configuration is used by both RTC and PIT functionality.

Configure RTC
To operate the RTC, follow these steps:
1. Set the compare value in the Compare (RTC.CMP) register, and/or the overflow value in the Period
(RTC.PER) register.
2. Enable the desired interrupts by writing to the respective interrupt enable bits (CMP, OVF) in the Interrupt
Control (RTC.INTCTRL) register.
3. Configure the RTC internal prescaler by writing the desired value to the Prescaler (PRESCALER) bit field in
the Control A (RTC.CTRLA) register.
4. Enable the RTC by writing a ‘1’ to the RTC Peripheral Enable (RTCEN) bit in the RTC.CTRLA register.
Note: The RTC peripheral is used internally during device start-up. Always check the Synchronization Busy bits in
the Status (RTC.STATUS) and Periodic Interrupt Timer Status (RTC.PITSTATUS) registers, and on the initial
configuration.
                                                                     
*/
/************************************************************************/

// Time < 7.99 sec, because of prescaler 4.
uint16_t set_rtc(uint16_t time)
{
	return (((time * 32768) / 32000) + 0.5);
}

void RTC_init(uint16_t time)
{
	sei();
	//USART1_sendString("RTC initializing..");
	/* Set period */
	RTC.PER = set_rtc(time);
	/* 32.768kHz Internal Crystal Oscillator (XOSC32K) */
	RTC.CLKSEL = RTC_CLKSEL_OSC32K_gc;

	RTC.CTRLA = RTC_PRESCALER_DIV32_gc /* 4 */
	| RTC_RTCEN_bm /* Enable: enabled */
	| RTC_RUNSTDBY_bm; /* Run In Standby: enabled */

	/* Enable Overflow Interrupt */
	RTC.INTCTRL |= RTC_OVF_bm;
	
	USART1_sendString("RTC set.");
}

/************************************************************************/
/* RTC interrupt.
   Make sure sei() is already called.                                                                     */
/************************************************************************/
ISR(RTC_CNT_vect)
{	
	USART1_sendChar('R');
	RTC_INTFLAGS |= (1 << 0);
}

/************************************************************************/
/* This function set the prescaler and register value according to the time.
   There is a large range value for the time.
   
   parameter:
   time: Time require for RTC.
          unit: ms.                                                                     */
/************************************************************************/
void set_rtc_value(unsigned long time)
{
	sei();
	
	uint16_t Prescaler = 0;
	uint16_t RegisterValue = 0;
	unsigned long temp = time;

	USART1_sendString("\n\nFor time:");
	USART1_sendInt(time);

	do
	{
		temp = (((time) / (pow(2, Prescaler) * 1000)) * 32768);
		
		// 		USART1_sendString("Prescaler test:");
		// 		USART1_sendInt(Prescaler);
		//
		// 		USART1_sendString("Value:");
		// 		USART1_sendInt(temp);
		Prescaler++;
	}while(temp > 65536);

	Prescaler--;

	RegisterValue = temp;

	USART1_sendString("Prescaler: ");
	USART1_sendInt(pow(2,Prescaler));
	
	USART1_sendString("Register value:");
	USART1_sendInt(RegisterValue);
	
	USART1_sendString("\n\nCalculating Error\n");
	temp = ((pow(2,Prescaler) * RegisterValue * 1000) / 32768) + 0.5;

	USART1_sendString("Accurate time period:");
	USART1_sendInt(temp);
	
	USART1_sendString("Entered Time:");
	USART1_sendInt(time);
	
	USART1_sendString("Error(percent):");
	USART1_sendFloat((abs(temp - time) / time) * 100,4);
	
	
	// Setting up RTC .
	RTC.PER = RegisterValue;
	
	/* 32.768kHz Internal Crystal Oscillator (XOSC32K) */
	RTC.CLKSEL = RTC_CLKSEL_OSC32K_gc;
	
	RTC.CTRLA = ((0x00 + Prescaler) << 3) /* variable */
	| RTC_RTCEN_bm /* Enable: enabled */
	| RTC_RUNSTDBY_bm; /* Run In Standby: enabled */
	
	/* Enable Overflow Interrupt */
	RTC.INTCTRL |= RTC_OVF_bm;
	
	USART1_sendString("Checking time after setting RTC register.");
	
	Prescaler = ((int)(RTC.CTRLA & 0b01111000) >> 3);
	USART1_sendString("Prescaler: ");
	USART1_sendInt(pow(2,Prescaler));
	
	
	USART1_sendString("Register value:");
	USART1_sendInt((uint16_t)RTC.PER);

	USART1_sendString("Accurate time period:");
	USART1_sendInt(temp);
	
	
	USART1_sendString("RTC set.");
}

/************************************************************************/
/* Function make controller to sleep.                                                                     */
/************************************************************************/
void GO_TO_SLEEP()
{
	sleep_enable();
	set_sleep_mode(SLEEP_MODE_STANDBY);
	sleep_cpu();
}

/************************************************************************/
/* Initialization
To start using the timer/counter in a basic mode, follow these steps:
1. Write a TOP value to the Period (TCAn.PER) register.
2. Enable the peripheral by writing a ‘1’ to the Enable (ENABLE) bit in the Control A (TCAn.CTRLA) register.
The counter will start counting clock ticks according to the prescaler setting in the Clock Select (CLKSEL) bit
field in TCAn.CTRLA.
3. Optional: By writing a ‘1’ to the Enable Counter Event Input A (CNTAEI) bit in the Event Control
(TCAn.EVCTRL) register, events are counted instead of clock ticks.
4. The counter value can be read from the Counter (CNT) bit field in the Counter (TCAn.CNT) register.                                                                     */
/************************************************************************/
// TCB0 as millis..-------------------------
void set_millis(void)
{
	sei();
	TCB0_CCMP = 3999;				// Write a TOP value to the Compare/Capture (TCBn.CCMP) register
	
	TCB0_CTRLB |= (0x0 << 0);
	TCB0_INTCTRL |= (1<<0);
	
	TCB0_CTRLA |= (1<<0)|(0x0 <<1); // ENABLE bit in the Control A (TCBn.CTRLA) register,
}

/************************************************************************/
/* Interrupt for millis.
   Make sure sei() is already called.                                                                     */
/************************************************************************/
ISR(TCB0_INT_vect)
{
	millis++;
	TCB0_INTFLAGS |= (1<<0);
}

#endif