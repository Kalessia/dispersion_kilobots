// Study of the coverage area problem within a robotic swarm
// Alessia Loi 2022


//-------------------------------------------------------------------------------
// DESCRIPTION
//-------------------------------------------------------------------------------

// d03_runAndTumbleAvoider.
// Each kilobot broadcasts messages continuosly.
// During the runAndTumble walk (d01), if the current kilobot meets another kilobot
// at dist < max_authorized_distance, then the current kilobot turns on the right
// to avoid its neighbor. 'dist' is obtained with the estimate_distance fonction
// on received messages.

// Leds color code :
// Led red : reorientation walk
// Led green : straight walk
// Led blue : turn right to avoid neighbor kilobot

// Recommended parameters (circular arena disk.csv) :
// kticks_straightWalk = 500;
// kticks_reorientationWalk = 500;
// max_authorized_distance = 35;

// Adaptation for real kilobots ideas : 
//	- max_authorized_distance : min distance to detect the neighbor and start turning




//-------------------------------------------------------------------------------
// IMPORTS
//-------------------------------------------------------------------------------

#include <kilombo.h>
#include <stdio.h> 		// for printf

#ifdef SIMULATOR
#include <jansson.h>
json_t *json_state();
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

const uint32_t kticks_straightWalk = 500;
const uint32_t kticks_reorientationWalk = 500;
const uint8_t max_authorized_distance = 35;	// exprimé en mm




//-------------------------------------------------------------------------------
// MESSAGES
//-------------------------------------------------------------------------------

// message transmission callback : returns the address of the message (transmit_msg) we declared
message_t *message_tx() {
	return &mydata->transmit_msg;
}

//-------------------------------------------------------------------------------

void message_rx(message_t *msg, distance_measurement_t *d) {
	mydata->rcvd_message = *msg;
	mydata->dist_measure = *d;
	mydata->flag_newMessage = 1; 	// flag=1 means that a new message has arrived
}




//-------------------------------------------------------------------------------
// FONCTIONS
//-------------------------------------------------------------------------------

void runAndTumbleWalk() {
	
	spinup_motors();
	
	if ((kilo_ticks > mydata->lastReset + kticks_straightWalk + kticks_reorientationWalk)) {
		mydata->lastReset = kilo_ticks - 1;
		mydata->currentDirection = rand_soft() % 2;
	}
	
	if (kilo_ticks < mydata->lastReset + kticks_reorientationWalk) {
		// Turn right or turn left
		set_color(RGB(3,0,0)); 	// red
		if (mydata->currentDirection == 0){
			set_motors(0, kilo_turn_right);
		} else {
			set_motors(kilo_turn_left, 0);
		}
	} else {
		// Go straight
		set_color(RGB(0,3,0)); 	// green
		set_motors(kilo_straight_left, kilo_straight_right);
	}
}


// If the current kilobot meets another kilobot at dist < max_authorized_distance,
// then the current kilobot turns right
void avoider() {
	set_color(RGB(0,0,3));	// blue
	spinup_motors();
	set_motors(0, kilo_turn_right);
}




//-------------------------------------------------------------------------------
// SETUP, LOOP, MAIN
//-------------------------------------------------------------------------------

void setup() {

	// Verbose : set flag_verbose=1 if you want to see execution details on terminal, flag_verbose=0 if not.
	// mydata->flag_verbose = 1; // not used
	
	// Initialize the random generator
    while(get_voltage() == -1);
    rand_seed(rand_hard() + kilo_uid);

	// Initialize transmit_msg
	mydata->transmit_msg.type = NORMAL;
	mydata->transmit_msg.data[0] = 0;	// first byte of the message
	mydata->transmit_msg.crc = message_crc(&mydata->transmit_msg);

	// Initialization variables
	mydata->flag_newMessage = 0;
	mydata->lastReset = rand_soft(); // starting time
	mydata->currentDirection = 1;
	mydata->dist = 100;
}


void loop() {

	// Check if a new message has arrived
	if (mydata->flag_newMessage){
		mydata->flag_newMessage = 0;
		mydata->dist = estimate_distance(&mydata->dist_measure);
	}

	if (mydata->dist < max_authorized_distance) {
		avoider();
	} else {
		runAndTumbleWalk();
	}
}


int main() {
	// Initialize kilobot hardware
	kilo_init();
	
	// Register the message functions (transition and reception) with the kilobot library
	kilo_message_tx = message_tx;
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

