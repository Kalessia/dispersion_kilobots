// Study of the coverage area problem within a robotic swarm
// Alessia Loi 2022


//-------------------------------------------------------------------------------
// DESCRIPTION
//-------------------------------------------------------------------------------

// d03_runAndTumbleAvoider.
// Each kilobot broadcasts messages continuosly.
// During the runAndTumble walk (d01), if the current kilobot meets another kilobot
// at dist < max_authorized_distance, then the current kilobot turns on the right (led blue)
// to avoid its neighbor. dist is obtained with the estimate_distance fonction
// on received messages).

// Led red : turn to the right or to the left (random choice)
// Led green : straight
// Led blue : turn right to avoid neighbor kilobot

// Adaptation for real kilobots ideas : 
//	- max_authorized_distance : min distance to detect the neighbor et react turning of 45°




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

const uint32_t kticks_straightWalk = 500;
const uint32_t kticks_reorientationWalk = 500;
const uint8_t max_authorized_distance = 40;	// exprimé en mm




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
	
	if(kilo_ticks % (mydata->startingTime + (kticks_straightWalk + kticks_reorientationWalk)) == 0) {
		mydata->lastReset = kilo_ticks;
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
// then the current kilobot turns right (led blue)
void avoider() {
	set_color(RGB(0,0,3));	// blue
	spinup_motors();
	set_motors(0, kilo_turn_right);
}




//-------------------------------------------------------------------------------
// SETUP, LOOP, MAIN
//-------------------------------------------------------------------------------

void setup() {

	mydata->flag_newMessage = 0; 	// boolean

	// Initialize transmit_msg
	mydata->transmit_msg.type = NORMAL;
	mydata->transmit_msg.data[0] = 0;	// first byte of the message
	mydata->transmit_msg.crc = message_crc(&mydata->transmit_msg);

	// Initialization variables
	mydata->lastReset = 0;
	mydata->startingTime = rand_hard();
	printf("startingTime : %d\n", mydata->startingTime);
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

