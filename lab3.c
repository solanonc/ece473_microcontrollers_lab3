// lab3.c 
// Cruz M. Solano-Nieblas
// 10.27.21

//#define DEBUG

#define TRUE 1
#define FALSE 0
#define SEGNUMS 4
#define COLONPOS 2
#define BUTTONS 8
#include <avr/io.h>
#include <util/delay.h>

//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5] = {0xFF};

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[12] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 
			   0x82, 0xF8, 0x80, 0x98, 0xFF, 0x07}; //0, 1, 2, 3, 4, 5, 6, 7, 8, 9, (blank), (colon blank)

enum encoder_state{IDLE, STATE01, DETENT, STATE10};  // four states for the encoder. STATE01 and STATE10 are in between IDLE and DETENT states



//******************************************************************************
//				spi_init
//                     Initializes spi operation 				
//

void spi_init(void){

  DDRB |= (1<<PB0 | 1<<PB1 | 1<<PB2); //output mode for SS, MOSI, SCLK
  SPCR |= (1<<SPE | 1<<MSTR); //master mode, clk low on idle, leading edge sample
  SPSR |= 1<<SPI2X; //choose double speed operation

 }//spi_init

//******************************************************************************
//				spi_read
//          Reads data from MISO pin connected to the encoders 				
//

uint8_t spi_read(void){

	PORTE &= ~(1<<PE6); // parallel load encoder pins
	_delay_us(100); //need a delay for buffer to change states and PORTA to read the buttons
	PORTE |= 1<<PE6; // disable parallel load to enable serial shifting
	_delay_us(100); //need a delay for buffer to change states and PORTA to read the buttons

	SPDR = 0x00; // dummy transmission to start receive
	while (bit_is_clear(SPSR, SPIF)){} // spin until transmission is complete

	return SPDR;


 }//spi_read

//******************************************************************************
//				spi_write
//          Writes data to MOSI pin connected to the bar graph 				
//

void spi_write(uint8_t data){
	
	SPDR = data;
	while (bit_is_clear(SPSR, SPIF)){} // spin until transmission is complete
	PORTD |= 1<<PD2;
	PORTD &= ~(1<<PD2);
	
}//spi_write


//******************************************************************************
//			      chk_encoders
//Check the state of the encoder.
//Adapted from Ganssel's "Guide to Debouncing"            


int8_t chk_encoders(uint8_t encoder){

	static uint16_t state = {0};
	uint8_t pinA, pinB;

	pinA = ((encoder & 1) == 0) ? 0 : 1; 
	pinB = ((encoder & 2) == 0) ? 0 : 1; 

	state = (state << 1) | pinA | 0xE000;

	if (state == 0xF000){return (pinB ? 1 : 0);}
	else{return -1;}

}//chk_encoders

//******************************************************************************
//                            chk_buttons                                      
//Checks the state of the button number passed to it. It shifts in ones till   
//the button is pushed. Function returns a 1 only once per debounced button    
//push so a debounce and toggle function can be implemented at the same time.  
//Adapted to check all buttons from Ganssel's "Guide to Debouncing"            
//Expects active low pushbuttons on PINA port.  Debounce time is determined by 
//external loop delay times 12. 
//

uint8_t chk_buttons(uint8_t button) {
static uint16_t states[8] = {0}; // an array to store the states of all buttons on the button board
			// states[0] corresponds to S1 on the board and states[7] corresponds to S8
states[button] = (states[button]<<1 | (! bit_is_clear(PINA, button)) | 0xE000); //first extract the bit that corresponds to the button
									      //then shift the state back to the 1's place
if (states[button] == 0xF000) {return TRUE;}
return FALSE;

}//chk_buttons

//******************************************************************************
//                                   segment_sum                                    
//takes a 16-bit binary input alue and places the appropriate equivalent 4 digit 
//BCD segment code in the array segment_data for display.                       
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
void segsum(uint16_t sum) {
  int i; //for loop variable
  //determine how many digits there are 
  int digits = 0; //stores the number of digits in sum
  uint8_t digit = 0; //stores a digit in sum
  uint16_t number = sum;
  if (number == 0){digits = 1;}
  else{
  while (number != 0) //divide number out until you get zero
  {
	number /= 10;
	digits++; // increase digits count after every loop iteration

  } 

  }

  //break up decimal sum into 4 digit-segments
  for (i = 0; i < digits; i++)
  {
	digit = sum % 10; //extract least significant digit from the sum
	segment_data[i] = dec_to_7seg[digit]; //convert digit to BCD code and store in segment_data array
	sum /= 10; //remove last digit;

  }	

  //blank out leading zero digits 
  if (digits < SEGNUMS) //if there are less digits than segment numbers
  {
	for (i = digits; i < SEGNUMS; i++)
	{
		segment_data[i] = dec_to_7seg[10]; //blank them out	
	
	}

  }

  //now move data to right place for misplaced colon position
  for (i = SEGNUMS; i > COLONPOS; i--)
  {
	segment_data[i] = segment_data[i-1];

  } 
  segment_data[COLONPOS] = dec_to_7seg[11];

}//segment_sum
//***********************************************************************************


//***********************************************************************************
uint8_t main()
{
//encoder test code
#ifdef DEBUG
  DDRE |= 1<<PE6;
  PORTE |= 1<<PE6;
  spi_init();
  DDRD |= 1<<PD2;
  uint8_t bar_graph = 10;

#endif

uint8_t i; //for loop variable
uint16_t count = 0; //display count

//set port B bits 4-7 as outputs
DDRB |= 1<<PB4 | 1<<PB5 | 1<<PB6 | 1<<PB7;
PORTB = 0x00; //init Port B

// bar graph and encoder init
/*DDRE |= 1<<PE6;
PORTE |= 1<<PE6;
spi_init();
DDRD |= 1<<PD2;
uint8_t bar_graph = 10;*/

while(1){
//encoder test code
#ifdef DEBUG
	bar_graph = spi_read();
	spi_write(bar_graph);

#endif

  //make PORTA an input port with pullups 
  DDRA = 0x00; //inputs
  PORTA = 0xFF; //pullups enabled

  //enable tristate buffer for pushbutton switches
  PORTB |= 1<<PB4 | 1<<PB5 | 1<<PB6; //decoder outputs logic low DEC7 to active low tri state buffer

  _delay_us(0.1); //need a delay for buffer to change states and PORTA to read the buttons
  //now check each button and increment the count as needed
  
  for (i = 0; i < BUTTONS; i++){
  	if (chk_buttons(i))
  	{
		count += 1<<i; //shifting is equivalent to 2^(# of shifts)

  	}

  }

 //disable tristate buffer for pushbutton switches
  PORTB &= ~(1<<PB4); //decoder outputs logic high and disables tri state buffer

  //bound the count to 0 - 1023
  if (count < 0){count = 0;}
  else if (count > 1023){count = 0;} 

  //break up the disp_value to 4, BCD digits in the array: call (segsum)
  segsum(count);

  //make PORTA an output
  DDRA = 0xFF;

  //bound a counter (0-4) to keep track of digit to display 
  PORTB = 0x00; //first digit
  for (i = 0; i < SEGNUMS+1; i++)
  {
	PORTA = segment_data[i]; //send 7 segment code to LED segments
	_delay_ms(1);

	//send PORTB the next digit to display
	PORTB += 0x10;

  }
     

  }//while
}//main
