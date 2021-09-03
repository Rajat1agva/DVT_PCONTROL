

#include <AVR128DA64/UART_1_AVR128DA64.h>
#include <AVR128DA64/I2C_0_AVR128DA64.h>

#include <stdio.h>

#define MPRLS 0x18  //address of sensor
#define COUNTS_224 (16777216L)  //2^24 value

void Pressure_init(void);
float Pressure_read(void);



char buffer [sizeof(uint32_t)*8+1]; 


//char out_str[35] = {0};

//////////////////////////MPRSS0001PG00001C

uint32_t T24BitValue = 0; //24bit data sent by sensor

uint8_t sdata=0;

uint16_t _PSI_min=0, _PSI_max=1;

uint32_t _OUTPUT_min, _OUTPUT_max;


float psi = 0,cmh2o = 0;
	
/////////////////////////////////////////
	
char out_str[35] = {0};	

void Pressure_init(void){
	
	PORTA.PIN2CTRL |= (1 << 3);   // SDA pull up //PA2
	PORTA.PIN3CTRL |= (1 << 3);   // SCL pull up //PA3

	_OUTPUT_min=3355443;
	_OUTPUT_max=13421772;
	
	USART1_init(9600);
	I2C_0_master_initialization();
	
}

float Pressure_read(void){

		
		
		I2C_0_sendAddress(MPRLS, 0);
		I2C_0_sendData(0xAA);
		I2C_0_sendData(0x00);
		I2C_0_sendData(0x00);
		I2C_0_stop_transmission();
		_delay_ms(10);
		
		I2C_0_sendAddress(MPRLS, 1);
		sdata = I2C_0_recieveData(); //status byte send by sensor
		T24BitValue =I2C_0_recieveData();
		T24BitValue <<=8;
		T24BitValue |=I2C_0_recieveData();
		T24BitValue <<=8;
		T24BitValue |=I2C_0_recieveData();
		
		TWI0.MCTRLB |= (1 << 2);   // Send NACK
		//I2C_0_stop_transmission();
		_delay_ms(10);
				

		psi = (T24BitValue - _OUTPUT_min) * (_PSI_max - _PSI_min);
		psi /= (_OUTPUT_max - _OUTPUT_min);
		psi += _PSI_min;
		
		cmh2o = psi*70.307;
		
		if (cmh2o > 1000)
		{
			cmh2o = 0;
		}
		
		USART1_sendFloat(cmh2o, 1);
		return cmh2o;
		
}