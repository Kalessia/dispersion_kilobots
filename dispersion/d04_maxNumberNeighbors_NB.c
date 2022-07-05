// Study of the coverage area problem within a robotic swarm
// Alessia Loi 2022


//-------------------------------------------------------------------------------
// DESCRIPTION
//-------------------------------------------------------------------------------

// d04_maxNumberNeighbors_NB.
// Each kilobot moves with a runAndTumble walk (d01), until it finds a position in the arèna
// where the number of its neighbors is less than max_authorized_nbNeighbors.

// Led white : the kilobot is looking for a good position and executes the runAndTumbleWalk
// Led magenta : the kilobot has found a good position with nbNeighbors < max_authorized_nbNeighbors




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
const uint32_t kticks_max_authorized_age = 5000;
const uint8_t max_authorized_distance = 40;		// exprimée en mm




//-------------------------------------------------------------------------------
// MESSAGES
//-------------------------------------------------------------------------------

// message transmission callback : returns the address of the message (transmit_msg) we declared
message_t *message_tx() {
	//printf("myID : %d ; nouveau msg envoyé: %d_%d\n", kilo_uid, mydata->transmit_msg.data[0], mydata->transmit_msg.data[1]);
	return &mydata->transmit_msg;
}

//-------------------------------------------------------------------------------
	
// Initialize the message to send : each kilobots sends its ID (uint16_t)
void setMsg_sendMyId() {
	mydata->transmit_msg.type = NORMAL;
	mydata->transmit_msg.data[0] = kilo_uid & 0xff;	// 0 low ID
	mydata->transmit_msg.data[1] = kilo_uid >> 8;	// 1 high ID	
	mydata->transmit_msg.crc = message_crc(&mydata->transmit_msg);
}

//-------------------------------------------------------------------------------

void message_rx(message_t *msg, distance_measurement_t *d) {
	mydata->rcvd_msg = msg->data[0] | msg->data[1] << 8; // pointed value
	mydata->dist_measure = *d;
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
		//set_color(RGB(3,0,0)); 	// red
		if (mydata->currentDirection == 0){
			set_motors(0, kilo_turn_right);
		} else {
			set_motors(kilo_turn_left, 0);
		}
	} else {
		// Go straight
		//set_color(RGB(0,3,0)); 	// green
		set_motors(kilo_straight_left, kilo_straight_right);
	}
}

//-------------------------------------------------------------------------------

// If the current kilobot meets another kilobot at dist < max_authorized_distance,
// then the current kilobot turns right (led blue)
void avoider() {
	//set_color(RGB(0,0,3));	// blue
	spinup_motors();
	set_motors(0, kilo_turn_right);
}

//-------------------------------------------------------------------------------

void keepWalking() {
	if (mydata->dist < max_authorized_distance) {
		avoider();
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

	mydata->flag_newMessage = 0; 	// boolean

	setMsg_sendMyId();

	// d01
	mydata->lastReset = 0;
	mydata->startingTime = rand_hard();
	//printf("startingTime : %d\n", mydata->startingTime);
	//mydata->currentDirection = 1;

	// d03
	mydata->dist = 100;

	// d04
	mydata->flag_correctNbNeighbors = 0;	// flag=1 means that nbNeighbors > max_authorized_nbNeighbors
	mydata->nbNeighbors = 0;
	mydata->flag_neighborAlreadyAdded = 0;
}

//-------------------------------------------------------------------------------

void loop() {
	uint8_t i;
	
	if ((mydata->nbNeighbors > 0) && ((kilo_ticks - mydata->list_neighborsAges[0]) > kticks_max_authorized_age)) {
		printf("myID : %d ; >>>>>>>>>>>>>>>>>>>>>>>>>>>TIMEOUT pour %d\n", kilo_uid, mydata->list_neighborsIds[i]);
		for (i = 0; i < mydata->nbNeighbors - 1; i++){		
			mydata->list_neighborsIds[i] = mydata->list_neighborsIds[i+1];
			mydata->list_neighborsAges[i] = mydata->list_neighborsAges[i+1];
			printf("myID : %d ; REFRESHING myTab at position %d : %d\n", kilo_uid, i, mydata->list_neighborsIds[i]);
		}
		mydata->list_neighborsIds[i] = NULL;
		mydata->list_neighborsAges[i] = NULL;

		mydata->nbNeighbors = mydata->nbNeighbors - 1;
	}

	// Check if a new message has arrived
	if (mydata->flag_newMessage){
		printf("\nmyID : %d ; new neighbor detected : %d\n", kilo_uid, mydata->rcvd_msg);
		mydata->dist = estimate_distance(&mydata->dist_measure);

		mydata->flag_neighborAlreadyAdded = 0;
		for (i = 0; i < mydata->nbNeighbors; i++){
			printf("myID : %d ; myTab at position %d : %d\n", kilo_uid, i, mydata->list_neighborsIds[i]);
			if (mydata->rcvd_msg == mydata->list_neighborsIds[i]) {
				mydata->flag_neighborAlreadyAdded = 1;
				printf("myID : %d ; neighbor %d IsAlreadyAdded\n", kilo_uid, mydata->rcvd_msg);
				//break;
			}
		}
		printf("flag_neighborAlreadyAdded: %d\n", mydata->flag_neighborAlreadyAdded);


		if (!mydata->flag_neighborAlreadyAdded) {
			if (mydata->nbNeighbors < MAX_AUTHORIZED_NBNEIGHBORS){
				mydata->list_neighborsIds[mydata->nbNeighbors] = mydata->rcvd_msg;
				mydata->list_neighborsAges[mydata->nbNeighbors] = kilo_ticks;
				mydata->nbNeighbors = mydata->nbNeighbors + 1;
				printf("myID : %d ; adding neighbor %d to tab. NbNeighbors = %d\n", kilo_uid, mydata->rcvd_msg, mydata->nbNeighbors);
			} else {
				//mydata->flag_correctNbNeighbors = 1;
				printf("myID : %d ; no more space in database! mydata->nbNeighbors = %d\n", kilo_uid, mydata->nbNeighbors);
			}
		}
		mydata->flag_newMessage = 0;
	}

	if (mydata->nbNeighbors == MAX_AUTHORIZED_NBNEIGHBORS) {
		mydata->flag_correctNbNeighbors = 1;
	} else {
		mydata->flag_correctNbNeighbors = 0;
	}
	
	switch (mydata->nbNeighbors) {
		case 0 :
			set_color(RGB(3,3,3)); // white
			break;
		case 1 :
			set_color(RGB(3,3,0)); // yellow 
			break;
		case 2 :
			set_color(RGB(0,3,3)); // cyan
			break;
		case 3 :
			set_color(RGB(3,0,3)); // magenta (we want 3 neighbors)
			break;
		default :
			set_color(RGB(0,0,0)); // more than 3
	}

	if (mydata->flag_correctNbNeighbors) {
		// Keep the current position	
		set_motors(0,0);
	} else {
		// Keep walking to find a better zone
		keepWalking();
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

