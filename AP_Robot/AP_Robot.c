#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdlib.h>

#define TRUE 1
#define FALSE 0
#define FishFinder PC7
#define RtWing PC6
#define LftWing PC5
#define OnLine PC4
#define Zsense PC3

//****************************************************************************************************/
//************Global variables************************************************************************/
uint8_t PWM_Y = 0;
uint8_t PWM_X = 0;
uint8_t PWM_Z = 0;
uint8_t temp = 0;
uint8_t drop = 0;
int32_t Xcount = 0;
int32_t Ycount = 0;

//Enumeration for Fishing State Machine
enum fishing
{
	Xtrack,
	Ytrack,
	Catching,
	Checking,
	Dropoff,
	Droppoint,
	Checking2,
	Reset
};

//****************************************************************************************************/
//************DEBOUNCE**Takes in button**Returns true if pressed***************************************/

int8_t debounce(uint8_t button) {
	static uint16_t state1 = 0; //holds present state
	state1 = (state1 << 1) | ( bit_is_clear(PIND, button))| 0xE000;
	if (state1 == 0xF000) return 1;
	return 0;
}//Debounce


//*********TIMER2_INIT****PWM_Y_AXIS*******************************************************************/
//***PB7***//
void timer2_init(void){
	TCCR2 |= (1<<WGM21) | (1<<WGM20);  //fast PWM
	TCCR2 |= (1<<COM21) ;              //PB7 output, non Inverted
	TCCR2 |= (1<<CS22) | (1<<CS20);    //1024 prescale (H_bridge, 100k switching max) 16M/15.6K
	OCR2 = 0xFF;                       //TOP (will spike @ MAX+1 when set to zero)
	TIMSK |= (1<<TOIE2);               //Enable interupt on overflow ISR(TIMER2_OVF){update OCR2}
}//Timer2


//****************************************************************************************************/
//*********TIMER0_INIT****PWM_X_AXIS*******************************************************************/
//***PB4***//
void timer0_init(void){
	TCCR0 |= (1<<WGM21) | (1<<WGM20);  //fast PWM
	TCCR0 |= (1<<COM21) ;              //PB4 output, non Inverted
	TCCR0 |= (1<<CS22) | (1<<CS20);    //1024 prescale (H_bridge, 100k switching max) 16M/15.6K
	OCR0 = 0xFF;                       //TOP (will spike @ MAX+1 when set to zero)
	TIMSK |= (1<<TOIE0);               //Enable interupt on overflow ISR(TIMER0_OVF){update OCR0}
}//Timer0

//****************************************************************************************************/
//*********TIMER3_INIT****PWM_Z_AXIS*******************************************************************/
//***PE3***//
//***16Bit Timer***//
void timer3_init(void){
	DDRE |= (1 << PE3);									//Change only pin3 on DDRE Reg to output
	TCCR3A |= (1<<COM3A1) | (0<<COM3A0) | (1<<WGM30);
	TCCR3B |= (1<<WGM32) | (1<<CS30) | (1<<CS32);
	TCCR3C = 0x00;
	OCR3A = 0x00;
	ETIMSK |= (1<<TOIE3);
}

//****************************************************************************************************/
//******PORT INIT**************************************************************************************/

void port_init(void){

	DDRB |= (1<<PB6) | (1<<PB5); //6; Ain2, 5; Ain1

	DDRB |= (1<<PB0);  //LED4 as output, standby pin on 1.2 amp H-bridge. High-enable
	DDRD = 0x00;       //set buttons for inputs.
	PORTD = 0xFF;      //Set pull ups for buttons
	DDRC = 0x00;       //Set Port C for inputs(mini-beam)
	DDRB |= (1<<PB2);  //Set PB2 for output, for DIR2 (X-Axis) on Motor Controller card

}//Port_init

//******************enc init***********************************************************************/
void enc_init(void){

	DDRE &= ~(1<<PE7);				//PE7 input
	PORTE |= (1<<PE7);				//PE7 Enable
	//EICRB |= (1<<ISC71);          //Rising
	EICRB |= (1<<ISC70);			//Falling
	EIMSK |= (1<<INT7);				//turns on INT7

	DDRE &= ~(1<<PE6);				//PE6 input
	PORTE |= (1<<PE6);				//PE6 Enable
	//EICRB |= (1<<ISC61);			//Rising
	EICRB |= (1<<ISC60);			//Falling
	EIMSK |= (1<<INT6);				//turns on INT6

	DDRE &= ~(1<<PE5);				//PE5 input
	DDRE &= ~(1<<PE4);				//PE5 input
	PORTE |= (1<<PE4);				//PE5 Enable
	EICRB |= (1 << ISC40);			//Falling
	//EICRB |= (1 << ISC41);		//Rising
	EIMSK |= (1 << INT4);			//turns on INT4
}

//****************************************************************************************************/
//*************ISR************************************************************************************/

//Y axis Encoder
ISR (INT4_vect){
	if (bit_is_clear(PINE, PE5) ^ bit_is_clear(PINE, PE4)){
		Ycount--;
	}else{
		Ycount++;
	}
}

//X axis Encoder 1 (quadrature)
ISR (INT6_vect){
	if((PINE & (1<<PE7)))
	{
		if((PINE & (1<<PE6)))
		Xcount--;
		else
		Xcount++;
	}
	else
	{
		if((PINE & (1<<PE6)))
		Xcount++;
		else
		Xcount--;
	}
}

//X Axis Encoder 2 (quadrature)
ISR (INT7_vect){
	if((PINE & (1<<PE7)))
	{
		if((PINE & (1<<PE6)))
		Xcount++;
		else
		Xcount--;
	}
	else
	{
		if((PINE & (1<<PE6)))
		Xcount--;
		else
		Xcount++;
	}
}//isr_int2

//*************ISR**CLOCK*****************************************************************************/
//Timers for PWM Outputs
//X-Axis PWM output
ISR(TIMER0_OVF_vect){
	//keep bounds on PWM for safety, max speed causes damage to machine and people
	if(PWM_X > 0xC4){PWM_X = 0xC4;} 
	OCR0 = PWM_X;
}//ISR
//Y-Axis PWM Output
ISR(TIMER2_OVF_vect){
	OCR2 = PWM_Y;
}//ISR
//Z-Axis PWM Output
ISR(TIMER3_OVF_vect){
	OCR3A = PWM_Z;
}
//****************************************************MAIN**********************************************/
int main(){
	enum fishing status = Xtrack;
	uint8_t loopbreak = 0;
	timer2_init();         //PWM Y_Axis, Output pin PB7
	timer0_init();         //PWM_X_Axis, Output pin PB4
	timer3_init();		   //PWM_Z_Axis, Output pin PE3
	port_init();           //PORT INIT
	enc_init();
	sei();               //enable interrupts

	while (1) {
		//Turn motors outputs off to start
		DDRB &= ~(1<<PB4);
		DDRE &= ~(1<<PE3);
		DDRB &= ~(1<<PB7);
		//While not Kill Switch
		while(!(PIND&(1<<PD0))){
			//Start the Fishing State Machine!
			switch(status){
			//Track on X Axis
			case Xtrack:
				if (Xcount < 5000){
					DDRB |= (1<<PB4);
					PORTB &= ~(1<<PB2);
					PWM_X = 0x39;
				}else if (Xcount > 20000){
					DDRB |= (1<<PB4);
					PORTB |= (1<<PB2);
					PWM_X = 0x58;
				}
				//If Fish Finder found a fish find Y location
				if(PINC & (1<<FishFinder)){
					PWM_X = 0x00;
					DDRB &= ~(1<<PB4);
					status = Ytrack;
				}
				break;
			//Track on Y Axis for Fish
			case Ytrack:
				if (Ycount < 5000){
					PORTB |= (1<<PB0);
					PORTB &= ~(1<<PB5);
					PORTB |= (1<<PB6);
					PWM_Y = 0xFE;
					DDRB |= (1<<PB7);
				}else if (Ycount>100000){
					PORTB |= (1<<PB0);
					PORTB |= (1<<PB5);
					PORTB &= ~(1<<PB6);
					PWM_Y = 0xFE;
					DDRB |= (1<<PB7);
				}
				//Find a fish at Fish Finder and at Lft Wing Y Sensor
				if ((PINC&(1<<LftWing))&&(PINC&(1<<FishFinder))){
					PWM_Y = 0;
					DDRB &= ~(1<<PB7);
					status = Catching;
				}
				break;
			//"Fish" With Z Axis. (RtWing sees fish and tells motor to "go" to attempt to catch the fish
			case Catching:
				//Wait for reel to be at maximum height
				if(PINC & (1<<Zsense)){
					//Swap directions of Z motor due to Zsense limiter
					DDRE &= ~(1<<PE3);
					//Reset PWM Speed
					PWM_Z = 0x78;
					//Check RtWing Sensor for possible fish coming
					if (PINC & (1<<RtWing)){
						//Check that motor is not currently running
						if(bit_is_clear(PORTB, PB3)){
							loopbreak++;
							//Attempt to catch fish
							DDRE |= (1<<PE3);
							PORTB |= (1<<PB3);
						}else{
							loopbreak++;
							PORTB &= ~(1<<PB3);
							DDRE |= (1<<PE3);
						}
					}
					/
					_delay_ms(150);
					status = Checking;
				}
				break;
			//Check if fish was caught
			case Checking:
				//Force reset after 10 attempts so we dont fish one spot forever
				if (loopbreak > 10){
					loopbreak = 0;
					status = Reset;
					break;
				}
				//Check rod is at maximum and disable motor
				if (PINC&(1<<Zsense))
					DDRE &= ~(1<<PE3);
				//If rod is at max AND online sensor sees a fish, go to droppoint
				if((PINC&(1<<OnLine)) && (PINC&(1<<Zsense))){
					status = Droppoint;
				//Else try to fish again
				}else if(~(PINC&(1<<OnLine)) && (PINC&(1<<Zsense))){
					status = Catching;
				}
				//Restart Case if there is no no fish and not zsense (rod is not at top yet)
				else
					status = Checking;
				break;
			//Bring fish to drop point
			case Droppoint:
				//Make sure z axis motor is at max, change direction of z axis motor
				if (PINC&(1<<Zsense))
					DDRE &= ~(1<<PE3);
				//Move to known Droppoint
				if ((Xcount < 150)){
					DDRB |= (1<<PB4);
					PORTB &= ~(1<<PB2);
					PWM_X = 0x40;
				}else if ((Xcount > 50)){
					DDRB |= (1<<PB4);
					PORTB |= (1<<PB2);
					PWM_X = 0x40;
				//Stop in drop point window
				}else
					DDRB &= ~(1<<PB4);
				//Move Y axis to known drop point
				if (Ycount < 100){
					PORTB |= (1<<PB0);
					PORTB &= ~(1<<PB5);
					PORTB |= (1<<PB6);
					PWM_Y = 0xFE;
					DDRB |= (1<<PB7);
				}else if (Ycount>500){
					PORTB |= (1<<PB0);
					PORTB |= (1<<PB5);
					PORTB &= ~(1<<PB6);
					PWM_Y = 0xFE;
					DDRB |= (1<<PB7);
				//Stop in drop point window
				}else
					DDRB &= ~(1<<PB7);
				//Make sure we made it to the drop point succesfully
				if ((Xcount>50) && (Xcount<150) && (Ycount>100) && (Ycount < 500))
					status = Dropoff;
				break;
			
			//Release fish into drop point
			case Dropoff:
				//Make sure X and Y axis are off
				DDRB &= ~(1<<PB4);
				DDRB &= ~(1<<PB7);
				//DDRE &= ~(1<<PE3);
				//Check for top of z axis again
				if((PINC&(1<<Zsense))){
					PWM_Z = 0x70;
					//Enable Z axis motor
					if(bit_is_clear(PORTB, PB3)){
						DDRE |= (1<<PE3);
						PORTB |= (1<<PB3);
					}else{
						PORTB &= ~(1<<PB3);
						DDRE |= (1<<PE3);
					}
					_delay_ms(150);
					//Check to see if we released the fish succesfully
					status = Checking2;
				}
				break;

			//Make sure fish was released
			case Checking2:
				//If rod at top, stop the motor
				if (PINC&(1<<Zsense))
					DDRE &= ~(1<<PE3);
				//Check Fish On Sensor if on attempt dropoff
				if((PINC&(1<<OnLine)) && (PINC&(1<<Zsense))){
					status = Dropoff;
				//If dropped start fish tracking again
				}else if(~(PINC&(1<<OnLine)) && (PINC&(1<<Zsense)))
					status = Xtrack;
				//If unknown or rod not at top check again later
				else
					status = Checking2;
				break;

			//Reset on loopbreak to known good position
			case Reset:
				if((PINC&(1<<Zsense)))
					DDRE &= ~(1<<PE3);
				DDRB &= ~(1<<PB4);
				DDRB &= ~(1<<PB7);
				if ((Xcount < 100)){
					DDRB |= (1<<PB4);
					PORTB &= ~(1<<PB2);
					PWM_X = 0x60;
				}else if ((Xcount > 2000)){
					DDRB |= (1<<PB4);
					PORTB |= (1<<PB2);
					PWM_X = 0x60;
				}else
					DDRB &= ~(1<<PB4);
				if (Ycount < 2000){
					PORTB |= (1<<PB0);
					PORTB &= ~(1<<PB5);
					PORTB |= (1<<PB6);
					PWM_Y = 0xFE;
					DDRB |= (1<<PB7);
				}else if (Ycount>500){
					PORTB |= (1<<PB0);
					PORTB |= (1<<PB5);
					PORTB &= ~(1<<PB6);
					PWM_Y = 0xFE;
					DDRB |= (1<<PB7);
				}else
					DDRB &= ~(1<<PB7);
				if ((Xcount > 0)&&(Xcount <2000)&&(Ycount > 0)&&(Ycount<2000)){
					status = Xtrack;
					DDRB &= ~(1<<PB7);
					DDRB &= ~(1<<PB4);
				}
			}
		}
	}//while
}//main
