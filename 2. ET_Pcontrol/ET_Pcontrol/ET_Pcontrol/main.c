/*
 * DVT_PUMP.c
 *
 * Created: 7/22/2021 9:50:45 AM
 * Author : RAJAT MISHRA
 * Hardware : AVR128DA48 - MPRSS0001PG00001C 
 */


#define  F_CPU 4000000UL

#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>
#include "I2C_0_AVR128DA64.h"
#include "UART_1_AVR128DA64.h"
#include "MPRSS_sensor.h"
#include <avr/interrupt.h>

void TCA0_PWM_init(void);
void P_control(void);
void PWM_Condition(void);
void continous_intubation(void);
void TCB_TIMER_init(void);



/* Pressure Sensor Variables */
float current_pressure =0;



/* P_Control Variables */
float Kp_error = 0.0, Kp = 1.5;
float Set_pressure  = 61;
uint32_t dutyCycle = 0; // OUTPUT ON PA0
uint16_t  valve1_timer=0;

bool flag =  false;

bool low_pressure_flag = true;
bool sq_wave_up_press = false;




/****MINIMUM AND MAXIMUM VALUES OF PRESSURE******/
#define MAX_PRESSURE  62
#define MIN_PRESSURE  0


float current_pressure, previous_pressure;






int main(void)
{
  USART1_init(9600);
  Pressure_init();
  TCA0_PWM_init();
  TCB_TIMER_init();
  PORTE.DIR |= (1<<0); //PE0 VALVE 1
  PORTE.DIR |= (1<<1); // PE1 VALVE 2
  PORTE.OUTSET |= (1<<0);
  PORTE.OUTSET |= (1<<1);
    while (1)
    {


 

 //USART1_sendFloat(current_pressure, 2);
 if(!(PORTE.OUT && (1<<0)))
{ continous_intubation();
}
else
{TCA0.SINGLE.CMP0 = 0;
}
 //previous_pressure = current_pressure;
USART1_sendInt(valve1_timer);
    }
}

void TCA0_PWM_init(void)
{
sei();
/* First PWM on PA1 PIN */
/* Direction set and set output to high */
PORTA.DIR |= (1 << 0);
PORTA.PIN0CTRL |= (1 << 3);
//PORTA.OUTSET |= (1 << 0);
TCA0.SINGLE.PER = 3999;//calculation for 1ms

TCA0.SINGLE.CNT = 0;

TCA0.SINGLE.CTRLA |= (0x0 << 1); //Ftca = fclk_per
TCA0.SINGLE.CTRLB |= (1 << 4); // compare 0 enable
TCA0.SINGLE.CTRLB |= (0x3 << 0); // Single slope PWM
TCA0.SINGLE.CTRLA |= (1 << 0); // Enable PWM

/* Second PWM on PA1 PIN */

// PORTA.DIR |= (1 << 1); // Direction set and set output to high
// PORTA.PIN1CTRL |= (1 << 3);
// TCA0.SINGLE.CTRLB |= (1 << 5); // Enable Compare Pa1
// TCA0.SINGLE.CTRLA |= (1 << 1); // Enable PWM PA1
// TCA0.SINGLE.CMP1 = 2500; // Duty cycle

}

void P_control(void)
{
Kp_error = (Set_pressure + 1) - current_pressure;
dutyCycle = dutyCycle + (Kp*Kp_error); //OUTPUT ON PA0 PIN
PWM_Condition();

}

void PWM_Condition(void)
{
if (dutyCycle >3999)
{
dutyCycle  = 3999;
}
else if (dutyCycle < 0)
{
dutyCycle = 0;
}
}

void continous_intubation(void)
{
	current_pressure = Pressure_read();
	USART1_sendFloat(current_pressure , 1);
	P_control();
	
	if (current_pressure <= Set_pressure - 2)
	{
		low_pressure_flag = true;
	}

	if (low_pressure_flag == true)
	{
		dutyCycle = 1999;
		while (current_pressure <= Set_pressure + 1)
		{
			current_pressure =  Pressure_read();

			P_control();
			// USART1_sendFloat(current_pressure , 1);
			
			TCA0.SINGLE.CTRLA |= (1 << 0);
		//	USART1_sendString("Increasing Pressure");
			TCA0.SINGLE.CMP0 = dutyCycle;			//MOTOR PWM
		}
		low_pressure_flag = false;

	}
	while (current_pressure > Set_pressure + 2)
	{
		current_pressure =  Pressure_read();
		// USART1_sendFloat(current_pressure , 1);
		P_control();
		TCA0.SINGLE.CTRLA |= (1 << 0);
		TCA0.SINGLE.CMP0 = 0;//dutyCycle;			//MOTOR PWM
		
	}
	
	while (current_pressure <= (Set_pressure + 2) && current_pressure >= (Set_pressure - 2))
	{
		current_pressure =  Pressure_read();
		// USART1_sendFloat(current_pressure , 1);
        //TCA0.SINGLE.CTRLA &= ~(1 << 0);		// MOTOR OFF
		dutyCycle = 0;
		TCA0.SINGLE.CMP0 = dutyCycle;
	}
}


void TCB_TIMER_init(void)
{
	TCB0.CCMP = 3906;
	TCB0.CTRLA |= 0x05;
	TCB0.INTCTRL |= (1<<0);
}




ISR(TCB0_INT_vect)
{   valve1_timer = valve1_timer+1;
	if(valve1_timer <= 32000)
	{
		if(valve1_timer <= 16000)
		{
			PORTE.OUT |= (1<<0);
		}
		else
		{
			PORTE.OUT &=~ (1<<0);
		}
		
	}
	else
	{
		valve1_timer = 0;
		PORTE_OUT ^= (1<<1); 
	}
	TCB0.INTFLAGS |= (1<<0);
}
