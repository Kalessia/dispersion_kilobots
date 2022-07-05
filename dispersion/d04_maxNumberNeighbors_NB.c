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

// distances expressed in mm
const uint8_t dist_max_runAvoiderBehavior = 40;
const uint8_t dist_min_between2Kbots = 60;




//-------------------------------------------------------------------------------
// MESSAGES
//-------------------------------------------------------------------------------

// message transmission callback : returns the address of the message (transmit_msg) we declared
message_t *message_tx() {
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
	//set_color(RGB(0,0,3));	// blue
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

void addKbotToNeighborsList() {
	printf("[Kilobot ID%d] : Adding kilobot ID%d to database...\n", kilo_uid, mydata->rcvd_msg);
	if (mydata->nbNeighbors < MAX_AUTHORIZED_NBNEIGHBORS) {
		mydata->list_neighbors[mydata->nbNeighbors].id = mydata->rcvd_msg;
		mydata->list_neighbors[mydata->nbNeighbors].age = kilo_ticks;
		mydata->nbNeighbors = mydata->nbNeighbors + 1;
		printf("[Kilobot ID%d] : Kilobot ID%d has been added to list_neighbors.\n", kilo_uid, mydata->rcvd_msg);
	} else {
		printf("[Kilobot ID%d] : Impossible to add kilobot ID%d to list_neighbors, no more space in database! (nbNeighbors = %d).\n", kilo_uid, mydata->rcvd_msg, mydata->nbNeighbors);
	}
}

//-------------------------------------------------------------------------------

void updateKbotAge(uint16_t kBotId) {
	slideNeighborsListFrom(kBotId);
	addKbotToNeighborsList();
	printf("[Kilobot ID%d] : (The age of the kilobot ID%d has been updated).\n", kilo_uid, kBotId);
}

//-------------------------------------------------------------------------------

void isKbotInNeighborsList() {
	uint8_t i;
	mydata->flag_neighborAlreadyAdded = 0;
	for (i = 0; i < mydata->nbNeighbors; i++) {
		if (mydata->rcvd_msg == mydata->list_neighbors[i].id) {
			mydata->flag_neighborAlreadyAdded = 1;
			printf("[Kilobot ID%d] : Kilobot ID%d is already in list_neighbors.\n", kilo_uid, mydata->rcvd_msg);
			updateKbotAge(mydata->rcvd_msg);
			break;
		}
	}
}

//-------------------------------------------------------------------------------

// kBotId removal and sliding following list_neighbors positions (from the oldest to the younger)
void slideNeighborsListFrom(uint16_t kBotId) {
	uint8_t i;
	for (i = 0; i < mydata->nbNeighbors - 1; i++){	
		if (mydata->list_neighbors[i].id == kBotId) {
			mydata->list_neighbors[i].id = mydata->list_neighbors[i+1].id;
			mydata->list_neighbors[i].age = mydata->list_neighbors[i+1].age;
		}	
	}
	mydata->list_neighbors[i].id = NULL;
	mydata->list_neighbors[i].age = NULL;

	mydata->nbNeighbors = mydata->nbNeighbors - 1;
	printf("[Kilobot ID%d] : Kilobot ID%d has been removed from list_neighbors.\n", kilo_uid, kBotId);
}

//-------------------------------------------------------------------------------

void setNbNeighborsLed() {
	switch (mydata->nbNeighbors) {
		case 0 :
			set_color(RGB(3,3,3)); // white
			break;
		case 1 :
			set_color(RGB(3,3,0)); // yellow 
			break;
		case 2 :
			set_color(RGB(0,3,0)); // green
			break;
		case 3 :
			set_color(RGB(0,3,3)); // cyan
			break;
		case 4 :
			set_color(RGB(0,0,3)); // blue
			break;
		case 5 :
			set_color(RGB(3,0,3)); // magenta 
			break;
		case 6 :
			set_color(RGB(3,0,0)); // red
			break;
		default :
			set_color(RGB(0,0,0)); // off
	}
}

//-------------------------------------------------------------------------------

void showNeighborsList() {
	if (mydata->nbNeighbors > 0) {
		uint8_t i;
		printf("[Kilobot ID%d] : list_neighbors.     \t| POS\t| NEIGHBOR_ID\t| CAPTURE_TIME\t\t(Size = nbNeighbors = %d)\n", kilo_uid, mydata->nbNeighbors);
		for (i = 0; i < mydata->nbNeighbors; i++) {
			printf("\t\t\t\t\t| %d\t| %d\t\t| %d\n", i, mydata->list_neighbors[i].id, mydata->list_neighbors[i].age);
		}	
	}
}




//-------------------------------------------------------------------------------
// SETUP, LOOP, MAIN
//-------------------------------------------------------------------------------

void setup() {

	// messages
	mydata->flag_newMessage = 0;
	setMsg_sendMyId();

	// d01
	mydata->lastReset = 0;
	mydata->startingTime = rand_hard();
	mydata->currentDirection = 1;

	// d03
	mydata->dist = 100;

	// d04
	mydata->nbNeighbors = 0;
	mydata->flag_neighborAlreadyAdded = 0;
	mydata->flag_correctNbNeighbors = 0;	// flag=1 means that nbNeighbors = max_authorized_nbNeighbors
}

//-------------------------------------------------------------------------------

void loop() {
	
	if ((mydata->nbNeighbors > 0) && ((kilo_ticks - mydata->list_neighbors[0].age) > kticks_max_authorizedNeighborAge)) {
		printf("[Kilobot ID%d] : Sliding list_neighbors (Kilobot ID%d is too old : distance = %d).\n", kilo_uid, mydata->list_neighbors[0].id, mydata->list_neighbors[0].age);
		slideNeighborsListFrom(mydata->list_neighbors[0].id);
	}

	// Check if a new message has arrived
	if (mydata->flag_newMessage) {
		printf("[Kilobot ID%d] : New message ! Kilobot ID%d has been detected at distance %d.\n", kilo_uid, mydata->rcvd_msg, mydata->dist);

		isKbotInNeighborsList(); // sets the flag_neighborAlreadyAdded
		if ((!mydata->flag_neighborAlreadyAdded) && (mydata->dist >= dist_min_between2Kbots)) {
			addKbotToNeighborsList(); // sets the nbNeighbors
		}

		mydata->flag_newMessage = 0;
	}

	showNeighborsList();
	
	if (mydata->nbNeighbors == MAX_AUTHORIZED_NBNEIGHBORS) {
		mydata->flag_correctNbNeighbors = 1;
	} else {
		mydata->flag_correctNbNeighbors = 0;
	}
	
	setNbNeighborsLed();

	if (mydata->flag_correctNbNeighbors) {
		// Keep the current position
		set_motors(0,0);
	} else {
		// Keep walking to find a better zone
		keepWalking();
	}

	printf("--------------------------------------------------------------------------------\n");
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

