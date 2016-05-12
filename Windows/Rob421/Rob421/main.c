
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
 uint16_t EncoderCount = 0;

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
     ASSR |= (1<<AS0);                //External Crystal
     TIMSK |= (1<<TOIE0);             //enable interrupts
     TCCR0 |= (1<<CS02) | (1<<CS00);  //normal mode, prescale by 128
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
//****************************************************************************************************/
//*******SPI_INIT**************************************************************************************/
  
  void spi_init(void){
    
    DDRB |= (1<< PB0) | (1<< PB1) | (1<<PB2); 
    SPCR |= (1<< SPE) | (1<< MSTR);
    SPSR |= (1<< SPI2X);
  }//SPI
//*****************************************************************************************************/
//******PORT INIT**************************************************************************************/

 void port_init(void){

  DDRB |= (1<<PB7) | (1<<PB6) | (1<<PB5); //PB7; PWM, 6; Ain2, 5; Ain1
  PORTB |= (1<<PB7);
  DDRB |= (1<<PB0); //LED4 as output 
  DDRD = 0x00;       //set buttons for inputs.  
  PORTD = 0xFF;      //Set pullups for buttons
  
 }//Port_init
//****************************************************************************************************/
//******ADC INIT**************************************************************************************/

 void adc_init(void){
   ADMUX  |= (1<<REFS0);  //5V reference
   ADMUX  |= (1<<ADLAR);  //Left justify
   ADCSRA |= (1<<ADEN);   //ADC enable
   ADCSRA |= (1<<ADFR);   //Free Running Mode
   ADCSRA |= (1<<ADIE);   //Interupt enable on conversion complete ISR(ADC) {adc_temp = ADCH;}
   ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);   //Prescale 128
   ADCSRA |= (1<<ADSC);
 }//ADC_init
//****************************************************************************************************/
//*************ISR**CLOCK*****************************************************************************/
  ISR(TIMER0_OVF_vect){
     static uint8_t j = 0;
     if(j==9){j=0;}

     switch(j){
       case(0): {
                 break;}

       case(2): {
                 break;}

       case(4): {
                 break;}
       
       case(6): {
                 break;}

       case(8): {
                 break;}
       default:  break;
     }//switch
    j++;
 
}//ISR
//*****************************************************************************************************/
//*************ISR************************************************************************************/
  ISR(TIMER2_OVF_vect){     //update OCR2 
     
      OCR2 = PWM_Y;
  
  }//ISR
//*****************************************************************************************************/
//*************ISR***ADC_conversion******************************************************************/
  ISR(ADC_vect){     //ADC conversion complete
     
          
  }//ISR
  //*************************************************************************************************/
  ISR(INT0_vect)
  {
	if(EncoderCount < 65535)
		EncoderCount++;
	else
		EncoderCount = 0;
  }
//**************************************************MAIN************************************************/ 
//***************************************************MAIN***********************************************/
//****************************************************MAIN**********************************************/
int main(){

  timer2_init();         //PWM Y_Axis
  //timer0_init();         //Real Time clock
  //spi_init();            //SPI init
  //adc_init();            //ADC init for PF0
  port_init();           //PORT INIT 
  sei();                 //enable interupts
  PORTB |= (1<<PB5);
  //portd init for external interrupt PORTD.0 ->> int0 in EIRCA
  PORTD |= 0x00;
  DDRD |= 0x00;
  //External interrupt init
  EICRA |= 0x03;  //rising edge
  EIMSK |= 0x01;  //INT0
  EIFR |= 0x01;   //flag shit
 
  while (1) {

    if(debounce(0)){//IF button S1 is pressed
       PORTB |= (1<<PB0);  //turn on LED1, puts high on standby pin on H-bridge (enable)
       PORTB ^= (1<<PB5);
       PORTB ^= (1<<PB6);
       PWM_Y = 0xFF;
    }//if

    if(debounce(1)){//IF button S2 is pressed
       PORTB &= ~(1<<PB0);  //turn off LED1, puts low on standby pin on H-bridge (disenable)
       PWM_Y = 0x00;
    }//if
    
     
  }//while
}//main
//********************************************************************************************************
  
