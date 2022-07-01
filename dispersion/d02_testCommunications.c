// Study of the coverage area problem within a robotic swarm
// Alessia Loi 2022


//-------------------------------------------------------------------------------
// DESCRIPTION
//-------------------------------------------------------------------------------

// d02_testCommunication. 
// Each kilobots sends messages continuously, led cyan = message sent correctly = no collisions detected.
// Robots close by receive the messages, led yellow = message received.
// Test this program with only 2 robots.




//-------------------------------------------------------------------------------
// IMPORTS
//-------------------------------------------------------------------------------

#include <kilombo.h>

#ifdef SIMULATOR
#include <jansson.h>
json_t *json_state();
//#include <stdio.h> 		// for printf
#else
#include <stdlib.h>
//#include <avr/io.h>  	// for microcontroller register defs
//  #define DEBUG          	// for printf to serial port
//  #include "debug.h"
#endif

#include "dispersion.h"	// defines the USERDATA structure
#include "simulation.h"
REGISTER_USERDATA(USERDATA)




//-------------------------------------------------------------------------------
// CONSTANTS
//-------------------------------------------------------------------------------

uint32_t const kticks_straightWalk = 500;
uint32_t const kticks_reorientationWalk = 500;




//-------------------------------------------------------------------------------
// MESSAGES
//-------------------------------------------------------------------------------

// message transmission callback : returns the address of the message (transmit_msg) we declared
message_t *message_tx() {
	return &mydata->transmit_msg;
}

// successful transmission callback
void message_tx_success() {
	mydata->flag_messageSent = 1;	// flag=1 means that a new message has been sent correctly
}

//-------------------------------------------------------------------------------

void message_rx(message_t *msg, distance_measurement_t *dist) {
	mydata->rcvd_message = *msg;
	mydata->flag_newMessage = 1; 	// flag=1 means that a new message has arrived
}




//-------------------------------------------------------------------------------
// FONCTIONS
//-------------------------------------------------------------------------------

void runAndTumbleWalk() {
	
	spinup_motors();
	
	if(kilo_ticks % (mydata->startingTime + (kticks_straightWalk + kticks_reorientationWalk)) == 0) {
		mydata->lastReset = kilo_ticks;
		mydata->currentDirection = rand_soft() % 2;
	}
	
	if (kilo_ticks < mydata->lastReset + kticks_reorientationWalk) {
		if (mydata->currentDirection == 0){
			set_motors(0, kilo_turn_right);
		} else {
			set_motors(kilo_turn_left, 0);
		}
	} else {
		set_motors(kilo_straight_left, kilo_straight_right);
	}
}




//-------------------------------------------------------------------------------
// SETUP, LOOP, MAIN
//-------------------------------------------------------------------------------

void setup() {

	mydata->flag_messageSent = 0; // boolean
	mydata->flag_newMessage = 0; // boolean

	// Initialize transmit_msg
	mydata->transmit_msg.type = NORMAL;
	mydata->transmit_msg.data[0] = 0;
	mydata->transmit_msg.crc = message_crc(&mydata->transmit_msg);

	// Initialization variables
	mydata->lastReset = 0;
	mydata->startingTime = rand_hard();
	printf("startingTime : %d\n", mydata->startingTime);
	mydata->currentDirection = 1;
}


void loop() {

	// Check if the message has been successfully sent
	if (mydata->flag_messageSent) {
		mydata->flag_messageSent = 0;
		set_color(RGB(0,3,3)); 	// cyan
	}

	// Check if a new message has arrived
	if (mydata->flag_newMessage){
		mydata->flag_newMessage = 0;
		set_color(RGB(3,3,0)); 	// yellow
	}

	runAndTumbleWalk();
}


int main() {
	// Initialize kilobot hardware
	kilo_init();
	
	// Register the message functions (transition and reception) with the kilobot library
	kilo_message_tx = message_tx;
	kilo_message_tx_success = message_tx_success;
	kilo_message_rx = message_rx;

	// Start kilobot event loop
	kilo_start(setup, loop);
	
	// Simulator settings
	SET_CALLBACK(global_setup, global_setup);
	SET_CALLBACK(botinfo, botinfo);
	SET_CALLBACK(reset, setup);
	SET_CALLBACK(json_state, json_state); // Saving bot state as json. Not for use in the real bot, only in the simulator.
	SET_CALLBACK(obstacles, obstacles_walls);
	
	return 0;
}

