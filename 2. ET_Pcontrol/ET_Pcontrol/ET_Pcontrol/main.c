/*
 * ET_Pcontrol.c
 *
 * Created: 7/22/2021 9:50:45 AM
 * Author : Deepender Kumar
 * Hardware : AVR128DA48 - MPRSS0001PG00001C - L298N - 3V PUMP - 2 12V Solenoid - ET_CUFF Tube
 */


#define  F_CPU 4000000UL

#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>
#include "I2C_0_AVR128DA64.h"
#include "UART_1_AVR128DA64.h"
#include "RTC_AVR128DA64.h"
#include "MPRSS_sensor.h"
void SQ_wave(void);

void TCA0_PWM_init(void);
void P_control(void);
void PWM_Condition(void);
void intubation(int mode);
void continous_intubation(void);
void squarewave_intubation(void);
void sinewave_intubation(void);
void SQ_wave(void);

#define continous 1
#define squarewave 2
#define sinewave 3
#define prssr_vle_hgh 70
#define prssr_vle_lw 20


/* Pressure Sensor Variables */
float current_pressure =0;

bool square_h = true;

/* P_Control Variables */
float Kp_error = 0.0, Kp = 1.5;
float Set_pressure  = 61;
uint32_t dutyCycle = 0; // OUTPUT ON PA0


bool flag =  false;

bool low_pressure_flag = true;
bool sq_wave_up_press = false;
unsigned long currentmillis = 0, temp;



/****MINIMUM AND MAXIMUM VALUES OF PRESSURE******/
#define MAX_PRESSURE  62
#define MIN_PRESSURE  0

#define leakage_limit  50
float current_pressure, previous_pressure, leakage_rate;
unsigned long timmer = 0;

bool start_pump_flag = 0, check_leak = 0, leakage_flag = 0;


void MILLIS_TCB1_init(void);

int main(void)
{
  USART1_init(9600);
  Pressure_init();
  TCA0_PWM_init();
 
 
    while (1)
    {


 
// USART1_sendString("leakage rate =");
 //USART1_sendFloat(current_pressure, 2);
 continous_intubation();
 //previous_pressure = current_pressure;

    }
}

void TCA0_PWM_init(void)
{
//sei();
/* First PWM on PA1 PIN */
/* Direction set and set output to high */
PORTA.DIR |= (1 << 0);
PORTA.PIN0CTRL |= (1 << 3);
//PORTA.OUTSET |= (1 << 0);
TCA0.SINGLE.PER = 3999;//lculation for 1ms

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
			PORTE_OUT &= ~(1 << 0);				// SOL 1 OFF
			PORTE_OUT |= (1 << 1);					// SOL 2 ON
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
		// PORTE_OUT |= (1 << 0);					// SOL 1 ON
		PORTE_OUT &= ~(1 << 1);					// SOL 2 OFF
		TCA0.SINGLE.CTRLA |= (1 << 0);
		TCA0.SINGLE.CMP0 = 0;//dutyCycle;			//MOTOR PWM
		
	}
	
	while (current_pressure <= (Set_pressure + 2) && current_pressure >= (Set_pressure - 2))
	{
		current_pressure =  Pressure_read();
		// USART1_sendFloat(current_pressure , 1);
		PORTE_OUT &= ~(1 << 0);					// SOL 1 OFF
		PORTE_OUT &= ~(1 << 1);					// SOL 2 OFF
		//TCA0.SINGLE.CTRLA &= ~(1 << 0);		// MOTOR OFF
		dutyCycle = 0;
		TCA0.SINGLE.CMP0 = dutyCycle;
	}
}


void MILLIS_TCB1_init(void)
{
TCB1_CCMP = 3999; // Write a TOP value to the Compare/Capture (TCBn.CCMP) register

TCB1_CTRLB |= (0x0 << 0);
TCB1_INTCTRL |= (1<<0);

TCB1_CTRLA |= (1<<0)|(0x0 <<1); // ENABLE bit in the Control A (TCBn.CTRLA) register,
}


ISR(TCB1_INT_vect)
{
// to detect the rate of pressure decrease over time
timmer++;
if (timmer > 2000)
{
USART1_sendString("timmer");
if (!(PORTE.IN & PIN0_bm) && !(PORTE.IN & PIN1_bm))
{
leakage_rate = previous_pressure - current_pressure;
USART1_sendString("leakage rate =");
USART1_sendFloat(leakage_rate, 2);
if (leakage_rate > leakage_limit)
{
USART1_sendString("leakage detected");
}
}
previous_pressure = current_pressure;
timmer = 0;
}


TCB1_INTFLAGS |= (1<<0);

}
