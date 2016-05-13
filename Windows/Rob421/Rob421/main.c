
#define TRUE 1
#define FALSE 0
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>

//****************************************************************************************************/
//************Global variables************************************************************************/
 uint8_t PWM_Y = 0;
 uint8_t PWM_X = 0;
 uint8_t cw1 = 2;
 uint8_t cw2 = 3;
 uint8_t ccw1 = 4;
 uint8_t ccw2 = 5; 
 uint8_t Ystate = 1;
 uint8_t Xstate = 1;
 uint8_t x = 1;
 int32_t Xcount = 0;
 int32_t Ycount = 0;

//****************************************************************************************************/
//************DEBOUNCE**Takes in button**Returns true if pressed***************************************/
 
  int8_t debounce(uint8_t button) {
    static uint16_t state = 0; //holds present state 
    state = (state << 1) | ( bit_is_clear(PIND, button))| 0xE000;
    if (state == 0xF000) return 1;
    return 0;
  }//Debounce
 
//******************************************************************************************************/
//*********TIMER0_INIT****CLOCK***********************************************************************/
  
  void timer0_init(void){             //timer counter 0 setup, running off 32768 clock
     TCCR0 |= (1<<WGM21) | (1<<WGM20);  //fast PWM
     TCCR0 |= (1<<COM21) ;              //PB7 output, non Inverted
     TCCR0 |= (1<<CS22) | (1<<CS20);    //1024 prescale (H_bridge, 100k switching max) 16M/1024=15K
     OCR0 = 0xFF;                       //TOP (will spike @ MAX+1 when set to zero)
     TIMSK |= (1<<TOIE0);
  }//Timer0
//****************************************************************************************************/
//*********TIMER2_INIT****PWM_Y_AXIS*******************************************************************/
  
  void timer2_init(void){             
    TCCR2 |= (1<<WGM21) | (1<<WGM20);  //fast PWM
    TCCR2 |= (1<<COM21) ;              //PB7 output, non Inverted
    TCCR2 |= (1<<CS22) | (1<<CS20);    //1024 prescale (H_bridge, 100k switching max) 16M/1024=15K
    OCR2 = 0xFF;                       //TOP (will spike @ MAX+1 when set to zero) 
    TIMSK |= (1<<TOIE2);               //Enable interupt on overflow ISR(TIMER2_OVF){update OCR2}
  }//Timer2

//void timer3_init(void){
//	TCCR3B |= (1<<WGM31) | (1<<WGM30); //fast pwm
//	TCCR3B |= (1<<CS32) | (1<<CS30)    //1024 prescale 15k
//	TCCR3B |= (1<<)
//	
//	
//}
//*****************************************************************************************************/
//******PORT INIT**************************************************************************************/

 void port_init(void){

  DDRB |= (1<<PB6) | (1<<PB5) | (1<<PB1) | (1<<PB0) | (1<<PB2);
  DDRD |= 0x00;
  PORTD = 0xFF;
  DDRC = 0x00;
  
 }//Port_init
//****************************************************************************************************/

/////////////////////Encoder Init//////////////////////////
void enc_init(void){
	DDRE &= ~(1<<PD7);
	PORTE |= (1<<PD7);
	EICRB = (1 << ISC70);
	EIMSK |= (1 << INT7);
	DDRE &= ~(1<<PD6);
	PORTE |= (1<<PD6);
	EICRB = (1 << ISC60);
	EIMSK |= (1 << INT6);
	DDRE &= ~(1<<PD5);
	PORTE |= (1<<PD5);
	EICRB = (1 << ISC50);
	EIMSK |= (1 << INT5);
	DDRE &= ~(1<<PD4);
	PORTE |= (1<<PD4);
	EICRB = (1 << ISC40);
	EIMSK |= (1 << INT4);
}

//****************************************************************************************************/
//*************ISR**CLOCK*****************************************************************************/
  ISR(TIMER0_OVF_vect){
     if(PWM_X > 0xC4)
		PWM_X = 0xC4;
     OCR0 = PWM_X;
}//ISR
//*****************************************************************************************************/
//*************ISR************************************************************************************/
  ISR(TIMER2_OVF_vect){     //update OCR2 
      OCR2 = PWM_Y;
  
  }//ISR
//*****************************************************************************************************/



//*******ENCODER INTERRUPTS*********************************************************************************/
ISR(INT7_vect)
{
// 	if(Ycount < 65535)
// 		Ycount++;
// 	else
// 		Ycount = 0;
	if(PINE6){
		if(PINE7){
		--Xcount;
		Xstate = ccw1;
		}
		else{
		++Xcount;
		Xstate = cw1;
		}
	}
	else{
		if (PINE7){
		Xstate = cw1;
		++Xcount;
		}
		else{
		Xstate = ccw1;
		--Xcount;
		}
	}
}

ISR(INT6_vect)
{
	if(PINE7){
		if(PINE6){
			++Xcount;
			Xstate = cw1;
		}
		else{
			--Xcount;
			Xstate = ccw1;
		}
	}
	else{
		if (PINE6){
			Xstate = ccw1;
			--Xcount;
		}
		else{
			Xstate = cw1;
			++Xcount;
		}
	}
}

ISR(INT5_vect){
	if(PINE4){
		if(PINE5){
			--Ycount;
			Ystate = ccw2;
		}
		else{
			++Ycount;
			Ystate = cw2;
		}
	}
	else{
		if (PINE5){
			Ystate = cw2;
			++Ycount;
		}
		else{
			Ystate = ccw2;
			--Ycount;
		}
	}
}

ISR(INT4_vect){
	if(PINE5){
		if(PINE4){
			++Ycount;
			Ystate = cw2;
		}
		else{
			--Ycount;
			Ystate = ccw2;
		}
	}
	else{
		if (PINE4){
			Ystate = ccw2;
			--Ycount;
		}
		else{
			Ystate = cw2;
			++Ycount;
		}
	}
}
//**************************************************MAIN************************************************/ 
int main(){

  timer2_init();         //PWM Y_Axis
  timer0_init();         //Real Time clock
  //spi_init();            //SPI init
  //adc_init();            //ADC init for PF0
  port_init();           //PORT INIT 
  enc_init();
  PORTB |= (1<<PB5);
  //External interrupt init
  
  sei();
 
  while (1) {
	   
///////////////////////////  Y-AXIS  ////////////////////////////////////////
	 if(debounce(0)){//IF button S1 is pressed, Y-Axis goes RIGHT
		 PORTB |= (1<<PB0);  //turn on LED1, puts high on standby pin on H-bridge (enable)
		 PORTB |= (1<<PB5);  //pin5 to Ain1 (Y-axis)
		 PORTB &= ~(1<<PB6); //pin6 to Ain2 (Y-axis)
		 PWM_Y = 0xFE;       //Duty cycle (Y-axis)
		 DDRB |= (1<<PB7);   //Turn on PWM (Y-axis)
	 } else if(debounce(1)){//IF button S2 is pressed, Y-Axis goes LEFT
		 PORTB |= (1<<PB0);  //turn on LED1, puts high on standby pin on H-bridge (enable)
		 PORTB &= ~(1<<PB5); //pin5 to Ain1 (Y-axis)
		 PORTB |= (1<<PB6);  //pin6 to Ain2 (Y-axis)
		 PWM_Y = 0xFE;       //Duty cycle (Y-axis)
		 DDRB |= (1<<PB7);   //Turn on PWM (Y-axis)
	 } else if(debounce(2)){//IF button S3 is pressed, Y-Axis STOP
		 PORTB &= ~(1<<PB0);  //turn off LED1, puts low on standby pin on H-bridge (disenable)
		 PORTB &= ~(1<<PB5); //pin5 to Ain1 (Y-axis)
		 PORTB &= ~(1<<PB6);  //pin6 to Ain2 (Y-axis)
		 PWM_Y = 0x00;        //Duty cycle (Y-axis)
		 DDRB &= ~(1<<PB7);   //Turn off PWM (Y-axis)
	 }
///////////////////////////////  X-AXIS  ////////////////////////////////////////
	 if(debounce(3)){//IF button S4 is pressed, X-Axis goes RIGHT
		 PWM_X = 0x40;       //Duty cycle (X-axis)
		 DDRB |= (1<<PB4);   //Turn on PWM (X-axis)
		 PORTB |= (1<<PB2);  //puts high on DIR2
	 } else if(debounce(4)){//IF button S5 is pressed, X-Axis goes LEFT
		 PWM_X = 0x40;       //Duty cycle (X-axis)
		 DDRB |= (1<<PB4);   //Turn on PWM (X-axis)
		 PORTB &= ~(1<<PB2);  //puts low on DIR2
	 } else if(debounce(5)){//IF button S6 is pressed, X-Axis STOP
		 PWM_X = 0x00;        //Duty cycle (X-axis)
		 DDRB &= ~(1<<PB4);   //Turn off PWM (X-axis)
	 }
////////////////////////////// Test Code /////////////////////////////////////
	if(Xcount >= 1000){
		PWM_X = 0;
		DDRB &= ~(1<<PB4);
	}
	if(Ycount >= 10000){
		PWM_Y = 0;
		DDRB &= ~(1<<PB7);
	}
 }//while
}//main
//********************************************************************************************************
  
