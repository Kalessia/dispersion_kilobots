// Study of the coverage area problem within a robotic swarm
// Alessia Loi 2022


//-------------------------------------------------------------------------------
// DESCRIPTION
//-------------------------------------------------------------------------------



// Recommended parameters :




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
//#include <avr/io.h>  		// for microcontroller register defs
//  #define DEBUG          	// for printf to serial port
//  #include "debug.h"
#endif

#include "dispersion.h"		// defines the USERDATA structure
#include "simulation.h"
REGISTER_USERDATA(USERDATA)




//-------------------------------------------------------------------------------
// CONSTANTS
//-------------------------------------------------------------------------------

// periods expressed in kiloticks
const uint32_t kticks_straightWalk = 500;
const uint32_t kticks_reorientationWalk = 500;
const uint32_t kticks_max_authorizedNeighborAge = 2000; // a neighborg has this age or less
const uint32_t kticks_max_timeToAnchor = 2100;

// distances expressed in mm
const uint8_t dist_max_runAvoiderBehavior = 40;
const uint8_t dist_min_between2Kbots = 55;




//-------------------------------------------------------------------------------
// MESSAGES
//-------------------------------------------------------------------------------

// message transmission callback : returns the address of the message (transmit_msg) we declared
message_t *message_tx() {
	if (mydata->flag_iAmAnchor) {
		return &mydata->transmit_msg1;
	} else {
		return &mydata->transmit_msg2;
	}
}

//-------------------------------------------------------------------------------
	
// Initialize the messages to send : each kilobots sends its ID (uint16_t) and its flag_iAmAnchor value
void setMsg_myId_iAmAnchor() {
	mydata->transmit_msg1.type = NORMAL;
	mydata->transmit_msg1.data[0] = kilo_uid & 0xff;	// 0 low ID
	mydata->transmit_msg1.data[1] = kilo_uid >> 8;	// 1 high ID
	mydata->transmit_msg1.data[2] = 1;	
	mydata->transmit_msg1.crc = message_crc(&mydata->transmit_msg1);
}

void setMsg_myId_iAmNotAnchor() {
	mydata->transmit_msg2.type = NORMAL;
	mydata->transmit_msg2.data[0] = kilo_uid & 0xff;	// 0 low ID
	mydata->transmit_msg2.data[1] = kilo_uid >> 8;	// 1 high ID
	mydata->transmit_msg2.data[2] = 0;	
	mydata->transmit_msg2.crc = message_crc(&mydata->transmit_msg2);
}

//-------------------------------------------------------------------------------

void message_rx(message_t *msg, distance_measurement_t *d) {
	mydata->rcvd_msg_id = msg->data[0] | msg->data[1] << 8; // pointed value
	mydata->rcvd_msg_isItAnchor = msg->data[2];
	mydata->dist_measure = *d;
	mydata->dist = estimate_distance(&mydata->dist_measure);
	mydata->flag_newMessage = 1; 	// flag=1 means that a new message has arrived
}




//-------------------------------------------------------------------------------
// MOTION FONCTIONS
//-------------------------------------------------------------------------------

void runAndTumbleWalk() {
	
	spinup_motors();
	
	if(kilo_ticks % (mydata->startingTime + (kticks_straightWalk + kticks_reorientationWalk)) == 0) {
		mydata->lastReset = kilo_ticks;
		mydata->currentDirection = rand_soft() % 2;
	}
	
	if (kilo_ticks < mydata->lastReset + kticks_reorientationWalk) {
		// Turn right or turn left
		if (mydata->currentDirection == 0) {
			set_motors(0, kilo_turn_right);
		} else {
			set_motors(kilo_turn_left, 0);
		}
	} else {
		// Go straight
		set_motors(kilo_straight_left, kilo_straight_right);
	}
}

//-------------------------------------------------------------------------------

// If the current kilobot meets another kilobot at dist < dist_max_runAvoiderBehavior,
// then the current kilobot turns right
void avoiderBehavior() {
	spinup_motors();
	set_motors(0, kilo_turn_right);
}

//-------------------------------------------------------------------------------

void keepWalking() {
	if (mydata->dist < dist_max_runAvoiderBehavior) {
		avoiderBehavior();
	} else {
		runAndTumbleWalk();
	}
}




//-------------------------------------------------------------------------------
// NEIGHBORS MANAGEMENT FONCTIONS
//-------------------------------------------------------------------------------





//-------------------------------------------------------------------------------
// SETUP, LOOP, MAIN
//-------------------------------------------------------------------------------

void setup() {

	// d01
	mydata->lastReset = 0;
	mydata->startingTime = rand_hard();
	mydata->currentDirection = 1;

	// d03
	mydata->dist = 100;

	// d04
	mydata->nbNeighbors = 0;
	mydata->flag_neighborAlreadyAdded = 0;
	mydata->flag_correctNbNeighbors = 0;	// flag=1 means that nbNeighbors = DESIRED_NBNEIGHBORS
	mydata->lastReset_timeToAnchor = 0;
	mydata->flag_timeToAnchorIsRunning = 0;
	mydata->flag_firstIteTimeToAnchor = 1;
	mydata->flag_iAmAnchor = 0;

	//d05
	mydata->flag_iAmAnchor = 0;
	amIAnchor();

	// messages d05
	mydata->flag_newMessage = 0;
	setMsg_myId_iAmAnchor();
	setMsg_myId_iAmNotAnchor();
}

//-------------------------------------------------------------------------------

void loop() {

	if (mydata->flag_iAmAnchor) {
		// Keep the current position
		set_color(RGB(3,0,3)); // magenta
		set_motors(0,0);
		return;
	}

	// Keep walking to find a better zone
	set_color(RGB(3,3,3)); // white
	keepWalking();
	
	
	if (mydata->flag_newMessage) {
		printf("[Kilobot ID%d] : New message ! Kilobot ID%d has been detected at distance %d. isItAnchor = %d.\n", kilo_uid, mydata->rcvd_msg_id, mydata->dist, mydata->rcvd_msg_isItAnchor);

		if ((mydata->rcvd_msg_isItAnchor) && (mydata->dist > dist_min_between2Kbots)) {
			mydata->flag_iAmAnchor = 1;
			printf("[Kilobot ID%d] : I am anchor !\n", kilo_uid);
		}

		mydata->flag_newMessage = 0;
	}

}

//-------------------------------------------------------------------------------

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

