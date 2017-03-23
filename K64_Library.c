//Ben Wedemire
//3515624
//
//A compliation of usefull functions for the MK64FN1M0VLL12
//
//2017/2/16

#include "fsl_device_registers.h"
#include "K64_Library.h"
#include "string.h"

//UART0 Functions
void UART0_Init(void);
void UART0_Putchar(char t);
int UART0_Getchar(void);
void UART0_PutString(char *s);

//ADC0 Functions
void ADC0_Init(void);
int ADC0_Convert(void);

//Switch Functions
void Switch_Init(void);
void Switch_Input(void);

//LED Functions
void LED_Init(void);

//Buffer Functions
rpm_struc *InitBuf(rpm_struc *array);
int AddBuf(int temp, rpm_struc *array);
int RemoveBuf(rpm_struc *array);
void check_array(rpm_struc *array);

//FTM Functions
void flextimer0_init(void);
unsigned int Measure_Pulse(void);

//Global Variables
volatile int sw2;

void UART0_Init(void){
	SIM_SCGC4 |= SIM_SCGC4_UART0_MASK; //enable clock for UART0
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK; //enable clock for PORTB

	PORTB_PCR16 |= PORT_PCR_MUX(3); //Connect UART0 Rx to PortB bits
	PORTB_PCR17 |= PORT_PCR_MUX(3); //Connect UART0 Tx to PortB bits

	UART0_C2 &=	~(UART_C2_TE_MASK | UART_C2_RE_MASK); // Disable transmit enable and receive enable

	UART0_C1 =0; //Set UART0 to 8 bits, no parity

	UART0_BDH = 0;
	UART0_BDL = 0x88; //Set UART0 BAUD RATE = 9600

	UART0_C2 |= UART_C2_TE_MASK; //enable transmit
	UART0_C2 |= UART_C2_RE_MASK; //enable receive
} //end UARTx_Interface_Init

void UART0_Putchar(char t){
	volatile int TDRE = 0;
	TDRE = UART0_S1 & 0b10000000; //MASK TDRE BIT to see if Tx buffer is full
	while(!TDRE){ //do nothing while buffer is full
		TDRE = UART0_S1 & 0b10000000; //MASK TDRE BIT to see if Tx buffer is full
	} //end while
	UART0_D = t; //buffer is no longer full, send char
} //end UART0_Putchar

int UART0_Getchar(void){
	volatile int RDRF = 0;
	volatile int r = 0;
	RDRF = UART0_S1 & 0b00100000; //MASK RDRF bit to see if Rx buffer is full
	while(!RDRF){ //do nothing while buffer is full
		RDRF = UART0_S1 & 0b00100000; //MASK RDRF bit to see if Rx buffer is full
	} // end while
	r = UART0_D;
	return r;
} //end UART0_Getchar

void UART0_PutString(char *s){
	while(*s){
	UART0_Putchar(*s);
	s++;
	}
} //end UART0_PutString

void ADC0_Init(void){
	SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK; //enable clock for ADC0
	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK; //enable clock for PORTE

	PORTE_PCR24 |= 0x2; //enable pulldown resistor
	PORTE_PCR25 |= 0x2; //enable pulldown resistor

	ADC0_CFG1 = 0xC; //ADC0 -> REGULAR RATE CLK, SHORT SAMPLE, MODE - 12 BIT, BUS CLOCK
	ADC0_CFG2 = 0; //ADC0 -> NORMAL CONVERSION SEQUENCE, NO MUX SELECT
	ADC0_SC2 &= 0XFFFFFF80; //MASK AND LEAVE LEAST SIG 7 BITS AS 0 (trigger, compare, DMA, Vref)
	ADC0_SC3 = 0; //DO NOT START CALIBRATION, ONE CONVERSION
	ADC0_SC1A &= 0XFFFFFF92; //select & mask with COCO flag
} //end ADC0_Init

int ADC0_Convert(void){
 	int data = 0;
	//ADC0_SC1A = 18 & ADC_SC1_ADCH_MASK;
	while(!(ADC0_SC1A&0x80)){
	}
	data = ADC0_RA;
	return data;
} //end ADC0_Convert

void Switch_Init(void){
  //Enable clock on Ports B
  SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTE_MASK;

  //PORT CONTROL FOR SW2
  PORTC_PCR6 |= PORT_PCR_MUX(1); //SW2

  //SELECT DATA DIRECTION FOR SW2
  GPIOC_PDDR = (0x0 << 6); // INPUT enable pin 6 --SW2
} //end Switch_Init

void Switch_Input(void){
	sw2 = (GPIOC_PDIR) & 0x40; //READ SW2, active low
	if(sw2){
		GPIOB_PSOR |=  0x600000; //TOGGLE R and B LED off
		GPIOE_PSOR |=  0x4000000; //TOGGLE Green LED off
	}
	else{
		GPIOB_PCOR |=  0x600000;  //TOGGLE R and B LED on
		GPIOE_PCOR |=  0x4000000; //TOGGLE Green LED on
	}
} //end Switch_Input

void LED_Init(void){
	//PORT CONTROL FOR LEDS
	PORTB_PCR22 |= PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK; //Red LED
	PORTB_PCR21 |= PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK; //Blue LED
	PORTE_PCR26 |= PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK; //Green LED

	//SELECT DATA DIRECTION FOR LEDs & SW2
	GPIOB_PDDR |= (0x3 << 21); // output enable pin 21, 22 -- BLUE & RED LED
	GPIOE_PDDR |= (0x01 << 26); // output enable pin 26 -- LED green

	GPIOB_PSOR |=  0x600000; //TOGGLE R & B LED to turn off
	GPIOE_PSOR |=  0x4000000; //TOGGLE Green LED off
} //end LED_Init

rpm_struc *InitBuf(rpm_struc *array) { //initializes the list
	array = malloc(MAXSIZE * sizeof(rpm_struc));
  return array;
} //end InitBuf

void check_array(rpm_struc *array) {
    int temp;
    for(int i = 0; i < MAXSIZE; i++) {
        temp = array[i].rpm_fb;
        //printf("%d\n", temp);
    } //end form
} //end check_array

int AddBuf(int temp, rpm_struc *array) { //Adds the value to the quene if it is not full
    int exit_val = 0;
    //check if the quene is full
    if (is_full) exit_val = 0;
    else { //if it is not incriment front and check if front equals back or front equals MAXSIZE
        if ((front + 1) == back) {
            is_full = 1; // is front equal to back
            exit_val = 0;
        } //end if
        else if (front >= MAXSIZE && back) { //check that front can be reset
            front = 0;
            array[front].rpm_fb = temp;
            exit_val = 1;
        } //end else if
        else if (front >= MAXSIZE && !back) { //check that front cannot be reset
            is_full = 1;
            exit_val = 0;
        } //end else if
        else {
            array[front].rpm_fb = temp;
            front++;
            exit_val = 1;
        } //end else
    } //end else
    return exit_val;
} //end AddBuf

int RemoveBuf(rpm_struc *array) { //Removes a value from the list if it is not empty
    if (!is_full && front == back) { //if the list is empty error out
        front = 0;
        back = 0;
        return 0;
    } // end if

    if (is_full) is_full = 0;

    int temp = 0;
    if (back >= MAXSIZE) {
        back = 0;
    } //end if
    temp = array[back].rpm_fb; //if it is not remove the first value and incriment back
    back++;

    //if (back > front) back = front; // check if back is greater than front

    if (back > MAXSIZE) { //check if back is equal to MAXSIZE
        back = 0;
    } //end if
    return temp;
} //end RemoveBuf

void flextimer0_init(void){
  SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK; //initialize clock to port C
  SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK; //initialize clock to the FTM0

  PORTC_PCR1 |= PORT_PCR_MUX(4); //connect FTM0_Ch0
  PORTC_PCR2 |= PORT_PCR_MUX(4); //connect FTM0_Ch1

  FTM0_MODE |= FTM_MODE_WPDIS_MASK; //Disable the write protect
  FTM0_OUTINIT |= 0x00000002; //enable input on CH1
  FTM0_MODE |= 0x00000002; //lock the I/O selection the channels
  FTM0_CNT = 0x0000; //initialize the counter to 0
  FTM0_MOD = 0xFFFF; //initialize the module reg to 0xffff
  FTM0_SC |= 0x000000F; //enable the system clock to the FTM and pre-scales the clock by 128 times
  FTM0_C1SC |= 0x0000000C; //On channel 1 select rising and falling edge detection and input capture
  FTM0_MODE &= 0xFFFFFFFB; //Enable the write protect
} //end flextimer0_init

unsigned int Measure_Pulse(void){
	int pulse_time = 0;
	int init_val = 0;
	int final_val = 0;
	int i = 1;
	FTM0_C1SC &= 0xFFFFFF7F; //clear channel flag
	while(!(FTM0_C1SC & 0x00000080));
		init_val = FTM0_C1V;  //get captured value
		FTM0_C1SC &= 0xFFFFFF7F; //clear channel flag
		FTM0_SC &= 0xFFFFFF7F; //reset the overflow counter
		while(!(FTM0_C1SC & 0x00000080)) {
			if ((FTM0_SC & 0x00000080)) {
				FTM0_SC &= 0xFFFFFF7F; //reset the overflow counter
				if (i == 1) {
					pulse_time = 0xFFFF - init_val;
					i = 0;
				}//end if
				else pulse_time += 0xFFFF;
			} //end if
		} //end while
		final_val = FTM0_C1V; //get captured value
		FTM0_C1SC &= 0xFFFFFF7F; //clear channel flag
	if (pulse_time == 0) pulse_time = final_val - init_val;
	else pulse_time += final_val; //count how long the button is pressed for
	return pulse_time;
} //end Measure_Pulse
