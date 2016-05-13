
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
 uint8_t cw1 =  2;
 uint8_t cw2 =  3;
 uint8_t ccw1 = 4;
 uint8_t ccw2 = 5;
 uint8_t temp = 0;
 uint8_t Ystate = 1;
 uint8_t Xstate = 1;
 uint8_t x =    1;
 int32_t Xcount = 0;
 int32_t Ycount = 0;
//****************************************************************************************************/
//************DEBOUNCE**Takes in button**Returns true if pressed***************************************/
 
  int8_t debounce(uint8_t button) {
    static uint16_t state1 = 0; //holds present state 
    state1 = (state1 << 1) | ( bit_is_clear(PIND, button))| 0xE000;
    if (state1 == 0xF000) return 1;
    return 0;
  }//Debounce
 
//******************************************************************************************************/
//*********TIMER0_INIT****CLOCK***********************************************************************/
  /*
  void timer0_init(void){             //timer counter 0 setup, running off 32768 clock
     ASSR |= (1<<AS0);                //External Crystal
     TIMSK |= (1<<TOIE0);             //enable interrupts
     TCCR0 |= (1<<CS02) | (1<<CS00);  //normal mode, prescale by 128
  }//Timer0
  /*
//****************************************************************************************************/
//*********TIMER2_INIT****PWM_Y_AXIS*******************************************************************/
//***PB7***//  
  void timer2_init(void){             
    TCCR2 |= (1<<WGM21) | (1<<WGM20);  //fast PWM
    TCCR2 |= (1<<COM21) ;              //PB7 output, non Inverted
    TCCR2 |= (1<<CS22) | (1<<CS20);    //1024 prescale (H_bridge, 100k switching max) 16M/15.6K
    OCR2 = 0xFF;                       //TOP (will spike @ MAX+1 when set to zero) 
    TIMSK |= (1<<TOIE0);               //Enable interupt on overflow ISR(TIMER2_OVF){update OCR2}
  }//Timer2


//****************************************************************************************************/
//*********TIMER0_INIT****PWM_X_AXIS*******************************************************************/
//***PB4***//  
  void timer0_init(void){             
    TCCR0 |= (1<<WGM21) | (1<<WGM20);  //fast PWM
    TCCR0 |= (1<<COM21) ;              //PB4 output, non Inverted
    TCCR0 |= (1<<CS22) | (1<<CS20);    //1024 prescale (H_bridge, 100k switching max) 16M/15.6K
    OCR0 = 0xFF;                       //TOP (will spike @ MAX+1 when set to zero) 
    TIMSK |= (1<<TOIE2);               //Enable interupt on overflow ISR(TIMER2_OVF){update OCR2}
  }//Timer0


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

  DDRB |= (1<<PB6) | (1<<PB5); //6; Ain2, 5; Ain1
  DDRB |= (1<<PB1);//testcode
  DDRB |= (1<<PB0); //LED4 as output, standby pin on 1.2 amp H-bridge. High-enable
  DDRD = 0x00;       //set buttons for inputs.  
  PORTD = 0xFF;      //Set pullups for buttons
  DDRE &= ~(1<<PE2);       //set Port E for inputs (encoder Y-Axis)
  DDRE &= ~(1<<PE3);       //set Port E for inputs (encoder Y-Axis)
  DDRE &= ~(1<<PE4);       //set Port E for inputs (encoder X-Axis)
  DDRE &= ~(1<<PE5);       //set Port E for inputs (encoder X-Axis)
  DDRC = 0x00;       //Set Port C for inputs(mini-beam)
  DDRB |= (1<<PB2);  //Set PB2 for output, for DIR2 (X-Axis)
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
   if(PWM_X > 0xC4){PWM_X = 0xC4;} //keep bounds on PWM for saftey!!!!
   OCR0 = PWM_X;

    /* static uint8_t j = 0;
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
   */
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
//*****************************************************************************************************/
//*************ENCODER******************************************************************/
void encoderY(void){
  if((PINE & (1<<PE2)) && !(PINE & (1<<PE3))){
                       if(Ystate == x) {Ystate = cw1;}
                       else if(Ystate == cw2) {Ystate = x;}
                       else if(Ystate == ccw1) {Ystate = ccw2;}
                       
                      }
  if((PINE & (1<<PE3)) && !(PINE & (1<<PE2))){
                       if(Ystate == cw1){Ystate = cw2;}
                       else if(Ystate == x){Ystate = ccw1;}
                       else if(Ystate == ccw2){Ystate = x;}
                       
                      }
  if((PINE & (1<<PE3)) && (PINE & (1<<PE2))){
                       if(Ystate == cw2){Ycount++;}
                       else if(Ystate == ccw2){Ycount--;}
                       Ystate = x;
                      }
  }//encoderY
//*****************************************************************************************************/
//*************ENCODER******************************************************************/
void encoderX(void){
  if((PINE & (1<<PE4)) && !(PINE & (1<<PE5))){
                       if(Xstate == x) {Xstate = cw1;}
                       else if(Xstate == cw2) {Xstate = x;}
                       else if(Xstate == ccw1) {Xstate = ccw2;}
                       
                      }
  if((PINE & (1<<PE5)) && !(PINE & (1<<PE4))){
                       if(Xstate == cw1){Xstate = cw2;}
                       else if(Xstate == x){Xstate = ccw1;}
                       else if(Xstate == ccw2){Xstate = x;}
                       
                      }
  if((PINE & (1<<PE5)) && (PINE & (1<<PE4))){
                       if(Xstate == cw2){Xcount++;}
                       else if(Xstate == ccw2){Xcount--;}
                       Xstate = x;
                      }
  }//encoderY
//**************************************************MAIN************************************************/ 
//***************************************************MAIN***********************************************/
//****************************************************MAIN**********************************************/
int main(){

  timer2_init();         //PWM Y_Axis, PB7
  timer0_init();         //PWM_X_Axis PB4
  //spi_init();            //SPI init
  //adc_init();            //ADC init for PF0
  port_init();           //PORT INIT 
  sei();               //enable interupts
  
  while (1) {
    encoderY();
    encoderX();
///////////////////////////  Y-AXIS  ////////////////////////////////////////
    if(debounce(0)){//IF button S1 is pressed, Y-Axis goes RIGHT
       PORTB |= (1<<PB0);  //turn on LED1, puts high on standby pin on H-bridge (enable)
       PORTB |= (1<<PB5);  //pin5 to Ain1 (Y-axis)
       PORTB &= ~(1<<PB6); //pin6 to Ain2 (Y-axis)
       PWM_Y = 0xFE;       //Duty cycle (Y-axis)
       DDRB |= (1<<PB7);   //Turn on PWM (Y-axis)
    }//if
    
    if(debounce(1)){//IF button S2 is pressed, Y-Axis goes LEFT
       PORTB |= (1<<PB0);  //turn on LED1, puts high on standby pin on H-bridge (enable)
       PORTB &= ~(1<<PB5); //pin5 to Ain1 (Y-axis)
       PORTB |= (1<<PB6);  //pin6 to Ain2 (Y-axis)
       PWM_Y = 0xFE;       //Duty cycle (Y-axis)
       DDRB |= (1<<PB7);   //Turn on PWM (Y-axis)
    }//if
    
    if(debounce(2)){//IF button S3 is pressed, Y-Axis STOP
       PORTB &= ~(1<<PB0);  //turn off LED1, puts low on standby pin on H-bridge (disenable)
       PORTB &= ~(1<<PB5); //pin5 to Ain1 (Y-axis)
       PORTB &= ~(1<<PB6);  //pin6 to Ain2 (Y-axis)
       PWM_Y = 0x00;        //Duty cycle (Y-axis)
       DDRB &= ~(1<<PB7);   //Turn off PWM (Y-axis)
    }//if
///////////////////////////////  X-AXIS  ////////////////////////////////////////
   if(debounce(3)){//IF button S4 is pressed, X-Axis goes RIGHT  
       PWM_X = 0x40;       //Duty cycle (X-axis)
       DDRB |= (1<<PB4);   //Turn on PWM (X-axis)
       PORTB |= (1<<PB2);  //puts high on DIR2
    }//if
    
    if(debounce(4)){//IF button S5 is pressed, X-Axis goes LEFT
       PWM_X = 0x40;       //Duty cycle (X-axis)
       DDRB |= (1<<PB4);   //Turn on PWM (X-axis)
       PORTB &= ~(1<<PB2);  //puts low on DIR2
    }//if
    
    if(debounce(5)){//IF button S6 is pressed, X-Axis STOP   
       PWM_X = 0x00;        //Duty cycle (X-axis)
       DDRB &= ~(1<<PB4);   //Turn off PWM (X-axis)
    }//if
///////////////////////  TEST CODE  ///////////////////////////////////////////////////
    if(Xcount == 3340){PORTB |= (1<<PB1);}// encoder limit test code
    // else(PORTB &= ~(1<<PB1));
   //if(PINC & (1<<PC0)){PWM_X = 0x00;}//mini-beam test code
  if(Xcount < 3339){PWM_X = 0x60;
                    DDRB |= (1<<PB4);   //Turn on PWM (X-axis)
                    PORTB &= ~(1<<PB2);}  //puts high on DIR2
  else if(Xcount > 3341){PWM_X = 0x60;
                    DDRB |= (1<<PB4);   //Turn on PWM (X-axis)
                    PORTB |= (1<<PB2);}  //puts high on DIR2
  else
     DDRB &= ~(1<<PB4);  

  }//while
}//main
//********************************************************************************************************
  
