/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Things that need to be updated
 * -set_fan_RPM slope must be checked and set.
 * -set_fan_RPM startup temperature must be set.
 * -conversion between data from temperature sensors and actual temperature must
 * be made.
*/

/* Change Log
 *
 * 17/3/9 Ben
 * -DAC function built and tested with the temperature sensors.
 * -The ADC was modified to work with 12 bits rather than 8.
 * -The function eaw_handler, PID_control, get_fan_RPM, and set_fan_RPM were
 * created.
 * -eaw_handler has been started but will need to be modified to include any
 * other warning and errors.
 * -The PID controller has been started.
 *
 * 17/3/14 Ben
 * -Added functions for the buffer.
 * -Changed some descriptions in the library file.
 *
 * 17/3/21 Ben
 * -Flextimer functions were added into the library file K64_Library.
 * -All integers in the queue program were changed to unsigned integers.
 * -Added initialization of the queue.
 * -Added some checking for queue statues and breakpoint sections.
 * -Added unsigned integers for total and total number of items in queue.
 * -Wrote more for the get_fan_RPM function, the function has not been tested.
 * -Added total_queue and total_items_queue for integral control.
 * -Added more into function get_fan_RPM.
 *
 * 17/3/22 Ben
 *  -Changed the queue to use a structure instead of unsigned integers
 *  -Changed the K64_Library.c to handle the change to structures
 *  -Changed the K64_Library.h to handle the change to structures
 */

//Ben Wedemire & Dan Pollard
//3515624 & 3196468
//
//Built on: 2017/3/7
//ECE 3232
//PD temperature controlled fan project

//Includes
#include "fsl_device_registers.h"
#include "K64_Library.h"

//Defines
#define MAXSIZE 100 //set this to be the size of the queue we want to create

//Void Function Declarations
void DAC0_Init(void);
void DAC0_Output(int data);
void eaw_handler(void); //stands for error and warning handler
void set_fan_RPM(int device_temp, int ambient_temp); //set the fan's desired RPM from the temperature input

//Returning Function Declarations
int get_device_temp(void);
int get_ambient_temp(void);
int get_fan_RPM(void);
int PID_control(int set_RPM); //control the output voltage based on the fan's current and wanted RPM

//Global Variables
int danger_flag = 0; //if set to 1 then danger is present, goto danger function
int front = 0; //setup the front of the queue
int back = 0; //setup the back of the queue
int is_full = 0; //setup flag for if the queue is full
int command_RPM = 0; //setup of the requested RPM

int main(void){

	//Hardware INITIALIZATIONS
	UART0_Init();
	ADC0_Init();
	DAC0_Init();

	//SOFTWARE INITIALZATIONS
	unsigned int *array;
	unsigned int = trash_val = 0; //oldest value which was removed from queue
	array = InitBuf(array); //initialize the queue

	//VARIABLES
	int device_temp = 0; //heatsink temperature
	int ambient_temp = 0; //ambient air temperature
	int DAC_out = 0; //output from the DAC
	int exit_flag_add_buf = 0; //is 1 if the insert was a sucess
	unsigned int RPM_fb = 0; //fan RPM feedback
	unsigned int prev_RPM_fb = 0; //previous fan RPM feedback
	unsigned int total_queue = 0; //the total of all values in the queue
	unsigned int total_items_queue = 0; //the total number of values in the queue

	//START DOING STUFF
	while(1){
		device_temp = get_device_temp(); //get heatsink temperature
		ambient_temp = get_ambient_temp(); //get ambient_temperature

		if(is_full){
			trash_val = RemoveBuf(array); //remove the earliest value
			total_queue = -= trash_val; //subtract the trash_val
			total_items_queue--; //decrement the # of items in the queue
		} //end if
		else trash_val = 0;

		prev_RPM_fb = RPM_fb;
		RPM_fb = get_fan_RPM(); //get the RPM of the fan
		exit_flag_AddBuf = AddBuf(RPM_fb, array); //attempt to add value to queue
		if(exit_flag_AddBuf){ //check if value was added successfully
			int stop_here = 1; //this should always have a breakpoint
		} //end if
		else{ //if the value was added then,
			total_queue += RPM_fb; //add the vaule to total_queue
			total_items_queue++; //and increment total_items_queue
		}//end else

		DAC_out = device_temp; //this is a placeholder
		DAC0_Output(DAC_out); //send output voltage to the DAC
	} //end while
} //end main

int get_device_temp(void){
	int device_temp = 0;
 	ADC0_SC1A &= 0XFFFFFF92; //select ADC0_SE18 & mask with COCO flag
 	ADC0_SC1A = 18 & ADC_SC1_ADCH_MASK; //and the register with 17 to clear all flags
	device_temp = ADC0_Convert(); //get temp
	if ((device_temp == 0x7ff) | (device_temp == 0x000)){ //written for 12-bits
		danger_flag = 1;
		eaw_handler();
	} //end if
	return device_temp;
} //end get_device_temp

int get_ambient_temp(void){
	int ambient_temp = 0;
 	ADC0_SC1A &= 0XFFFFFF91; //select ADC0_SE17 & mask with COCO flag
 	ADC0_SC1A = 17 & ADC_SC1_ADCH_MASK; //and the register with 17 to clear all flags
	ambient_temp = ADC0_Convert(); //get temp
	return ambient_temp;
	if (ambient_temp == 0){ //written for 12-bits
		//write warning for no ambient temp
	} //end if
} //end get_ambient_temp

void DAC0_Init(void){
	SIM_SCGC2 |= 0x1000; //enable clock for DAC0
	DAC0_C0 |= 0xC0; //enable the DAC
} //end Init_DAC0

void DAC0_Output(int data){
	DAC0_DAT0L = (data & 0xFF); //set the low register
	data = (data >> 8);
	DAC0_DAT0H = (data & 0xF); //set the high register
} //end DAC0_Output

void eaw_handler(void){
	if(danger_flag == 1){
		int DAC_error_out = 0xFFF;
		DAC0_Output(DAC_error_out);
		//also finish the device shutdown program
		while((get_device_temp() == 0xFFF) | (get_device_temp() == 0x000));
	} //end if
} //end eaw_handler

int PID_control(int set_RPM){
	//check if there is a need to calculate anything
	if (set_RPM == 0) return 0;

	//variables
	int kp = 0; //this will have to be tuned later
	int kd = 0; //this will have to be tuned later
	int ki = 0; //this will have to be tuned later
	int slope = 0;
	int scale_factor = 0;
	int current_time = 0;
	int prev_time = 0;
	int p = 0;
	int d = 0;
	int i = 0;
	int PID_RPM = 0;
	int PID = 0;

	//roughing out P controller
	p = (set_RPM - RPM_fb)*kp; //this calculates the proportional part of the PD controller

	//roughing out the D controller
	//RPM_fb = get_fan_RPM();// this should be done with an interrupt

	//get how long it has been since last time this was called
	slope = (RPM_fb - prev_RPM_fb)/(current_time - prev_time);
	d = slope * kd;

	//roughing out the I controller

	//i think that averaging over 100 - 1000 items is plenty

	//how long?

	//how to track the outputs?

	//summing the values
	PID_RPM = p + d + i; //this give a fan rpm but a voltage level is needed.
	PID = PID_RPM * scale_factor; //the scale_factor will convert the RPM to the voltage wanted
	return PID;
} //end PD_control

unsigned int get_fan_RPM(void){ //this could be done with an interrupt
	unsigned int = pulse_time = 0;
	unsigned int = RPM_fb = 0;
	pulse_time = Measue_Pulse();
	RPM_fb = prev_RPM_fb;

	//convert the pulse time to rev/sec. Also times pulse_time by 2 since two
	//pulses per rev and total by 60 to convert rev/sec to RPM.
	RPM_fb = 60/(pulse_time * 2);
	return RPM_fb;
} //end get_fan_RPM

void set_fan_RPM(int device_temp, int ambient_temp){
	if(device_temp <= 0x0000){ //this checks if the device temp is below a set temp
		command_RPM = 0;
	} //end if
	else{
		//the 17.78 comes from a temperature range of 20 to 65 degrees and a fan
		//speed of 1000 RPM to 1800 RPM
		command_RPM = 17.78 * device_temp;
	} //end else
} //end set_fan_RPM
