
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


  }





//**************************************************MAIN************************************************/ 
int main(){

  timer2_init();         //PWM Y_Axis
  timer0_init();         //Real Time clock
  //spi_init();            //SPI init
  //adc_init();            //ADC init for PF0
  port_init();           //PORT INIT 
  PORTB |= (1<<PB5);
  //External interrupt init
  DDRE &= ~(1<<PD7);
  PORTE |= (1<<PD7);
  EICRB = (1 << ISC70);
  EIMSK |= (1 << INT7);
  sei();
 
  while (1) {
	   
     if (debounce(0)){
		 PORTB |= (1<<PB0);
		 PORTB |= (1<<PB5);
		 PORTB &= ~(1<<PB6);
		 PWM_Y = 0xFE;
		 DDRB |= (1<<PB7);
	 }
	 else if ()
  }//while
}//main
//********************************************************************************************************
  
