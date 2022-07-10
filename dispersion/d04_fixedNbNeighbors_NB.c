// Study of the coverage area problem within a robotic swarm
// Alessia Loi 2022


//-------------------------------------------------------------------------------
// DESCRIPTION
//-------------------------------------------------------------------------------

// d04_fixedNbNeighbors_NB.
// Each kilobot moves with a runAndTumble walk (d01), until it finds a position in the arèna
// where the number of its nbNeighbors = DESIRED_NBNEIGHBORS.

// Led white : the kilobot is looking for a good position and executes the runAndTumbleWalk
// Led magenta : the kilobot has found a good position with nbNeighbors = DESIRED_NBNEIGHBORS

// Parameters :
//	- DESIRED_NBNEIGHBORS 3


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
const uint32_t kticks_max_authorizedNeighborAge = 1000; // a neighborg has this age or less
const uint32_t kticks_max_timeToAnchor = 1100;

// distances expressed in mm
const uint8_t dist_max_runAvoiderBehavior = 40;
const uint8_t dist_min_between2Kbots = 55;




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
	mydata->rcvd_msg_id = msg->data[0] | msg->data[1] << 8; // pointed value
	mydata->dist_measure = *d;
	mydata->dist = estimate_distance(&mydata->dist_measure);
	mydata->flag_newMessage = 1; 	// flag=1 means that a new message has arrived
}




//-------------------------------------------------------------------------------
// MOTION FONCTIONS
//-------------------------------------------------------------------------------

void runAndTumbleWalk() {
	
	spinup_motors();
	
	if ((kilo_ticks > mydata->lastReset + kticks_straightWalk + kticks_reorientationWalk)) {
		mydata->lastReset = kilo_ticks - 1;
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

void addKbotToNeighborsList() {
	printf("[Kilobot ID%d] : Adding kilobot ID%d to database...\n", kilo_uid, mydata->rcvd_msg_id);
	if (mydata->nbNeighbors < MAX_AUTHORIZED_NBNEIGHBORS) {
		mydata->list_neighbors[mydata->nbNeighbors].id = mydata->rcvd_msg_id;
		mydata->list_neighbors[mydata->nbNeighbors].age = kilo_ticks;
		mydata->list_neighbors[mydata->nbNeighbors].distance = mydata->dist;
		mydata->nbNeighbors = mydata->nbNeighbors + 1;
		printf("[Kilobot ID%d] : Kilobot ID%d has been added to list_neighbors.\n", kilo_uid, mydata->rcvd_msg_id);
	} else {
		printf("[Kilobot ID%d] : Impossible to add kilobot ID%d to list_neighbors, no more space in database!\n", kilo_uid, mydata->rcvd_msg_id, mydata->nbNeighbors);
	}
}

//-------------------------------------------------------------------------------

void updateKbot(uint16_t kBotId) {
	slideNeighborsListFrom(kBotId); // remove old kBotId
	addKbotToNeighborsList();		// add kBotId with updated values
	printf("[Kilobot ID%d] : (The age of the kilobot ID%d has been updated).\n", kilo_uid, kBotId);
}

//-------------------------------------------------------------------------------

void isKbotInNeighborsList() {
	uint8_t i;
	mydata->flag_neighborAlreadyAdded = 0;
	for (i = 0; i < mydata->nbNeighbors; i++) {
		if (mydata->rcvd_msg_id == mydata->list_neighbors[i].id) {
			mydata->flag_neighborAlreadyAdded = 1;
			printf("[Kilobot ID%d] : Kilobot ID%d is already in list_neighbors.\n", kilo_uid, mydata->rcvd_msg_id);
			break;
		}
	}
}

//-------------------------------------------------------------------------------

// kBotId removal and sliding following list_neighbors positions (from the oldest to the younger)
void slideNeighborsListFrom(uint16_t kBotId) {
	uint8_t i;
	for (i = 0; i < mydata->nbNeighbors - 1; i++){	
		if (mydata->list_neighbors[i].id == kBotId) { // sliding starts from this index i, previous neighbors are inchanged
			while (i < mydata->nbNeighbors - 1) { // sliding
				mydata->list_neighbors[i].id = mydata->list_neighbors[i+1].id;
				mydata->list_neighbors[i].age = mydata->list_neighbors[i+1].age;
				mydata->list_neighbors[i].distance = mydata->list_neighbors[i+1].distance;
				i++;
			}
			break;
		}	
	}
	mydata->list_neighbors[i].id = NULL;
	mydata->list_neighbors[i].age = NULL;
	mydata->list_neighbors[i].distance = NULL;

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
		printf("[Kilobot ID%d] : list_neighbors.     \t| POS\t| NEIGHBOR_ID\t| CAPTURE_TIME\t| DISTANCE\t(Size = nbNeighbors = %d)\n", kilo_uid, mydata->nbNeighbors);
		for (i = 0; i < mydata->nbNeighbors; i++) {
			printf("\t\t\t\t\t| %d\t| %d\t\t| %d\t\t| %d\n", i, mydata->list_neighbors[i].id, mydata->list_neighbors[i].age, mydata->list_neighbors[i].distance);
		}	
	}
}




//-------------------------------------------------------------------------------
// SETUP, LOOP, MAIN
//-------------------------------------------------------------------------------

void setup() {

	// Initialize the random generator
    while(get_voltage() == -1);
    rand_seed(rand_hard() + kilo_uid);

	// messages
	mydata->flag_newMessage = 0;
	setMsg_sendMyId();

	// d01
	mydata->lastReset = rand_soft(); // starting time
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
}

//-------------------------------------------------------------------------------

void loop() {

	// If the current kilobot is at a good distance from every neighbor in the list_neighbors, it stops
	if (mydata->flag_iAmAnchor) {
		set_color(RGB(3,0,3));  // magenta
		return;
	}
	

	//------- usual behavior --------

	// Remove old neighbors and slide list_neighbors positions
	if ((mydata->nbNeighbors > 0) && ((kilo_ticks - mydata->list_neighbors[0].age) > kticks_max_authorizedNeighborAge)) {
		printf("[Kilobot ID%d] : Sliding list_neighbors (Kilobot ID%d is too old).\n", kilo_uid, mydata->list_neighbors[0].id, mydata->list_neighbors[0].id);
		slideNeighborsListFrom(mydata->list_neighbors[0].id);
	}

	// A new message has arrived : a kilobot in the neighborhood notifies its presence
	if (mydata->flag_newMessage) {
		printf("[Kilobot ID%d] : New message ! Kilobot ID%d has been detected at distance %d (min distance required to stay in list_neighbors = %d).\n", kilo_uid, mydata->rcvd_msg_id, mydata->dist, dist_min_between2Kbots+1);

		isKbotInNeighborsList(); // sets the flag_neighborAlreadyAdded
		if (mydata->flag_neighborAlreadyAdded) {
			if (mydata->dist > dist_min_between2Kbots) {
				// Update the current neighbor capture time (age expressed in kilo_ticks)
				updateKbot(mydata->rcvd_msg_id);
			} else {
				// Remove the current neighbor from list_neighbors because now its neighbor is too close
				slideNeighborsListFrom(mydata->rcvd_msg_id);
			}
		} else {
			// Add the current neighbor to list_neighbors 
			if ((!mydata->flag_timeToAnchorIsRunning) && (mydata->dist > dist_min_between2Kbots)) {
				addKbotToNeighborsList(); // sets the nbNeighbors
			}
		}
		mydata->flag_newMessage = 0;
	}

	if (mydata->nbNeighbors == DESIRED_NBNEIGHBORS) {
		mydata->flag_correctNbNeighbors = 1;
	} else {
		mydata->flag_correctNbNeighbors = 0;
	}
	
	if (mydata->flag_correctNbNeighbors) {
		// Keep the current position and try to fix it
		set_motors(0,0);
		if (!mydata->flag_timeToAnchorIsRunning) {
			mydata->flag_timeToAnchorIsRunning = 1;
			mydata->flag_firstIteTimeToAnchor = 1;
		}
	} else {
		// Keep walking to find a better zone
		keepWalking();
		mydata->flag_timeToAnchorIsRunning = 0;
	}

	// flag_timeToAnchorIsRunning treatement (attempt to fix the current kilobot position)
	if (mydata->flag_timeToAnchorIsRunning) {
		if (mydata->flag_firstIteTimeToAnchor) {
			mydata->lastReset_timeToAnchor = kilo_ticks;
			mydata->flag_firstIteTimeToAnchor = 0;
			printf("[Kilobot ID%d] : Time to anchor started at t = %d kiloticks.\n", kilo_uid, mydata->lastReset_timeToAnchor);
		}

		// If at the end of the kticks_max_timeToAnchor period the current kilobot has still
		// DESIRED_NBNEIGHBORS neighbors, than it fixes its position
		if (kilo_ticks > mydata->lastReset_timeToAnchor + kticks_max_timeToAnchor) {
			if (mydata->nbNeighbors == DESIRED_NBNEIGHBORS) {
				mydata->flag_iAmAnchor = 1;
				printf("[Kilobot ID%d] : I am anchor !\n", kilo_uid);
			} else {
				mydata->flag_timeToAnchorIsRunning = 0;
				printf("[Kilobot ID%d] : Anchor failed.\n", kilo_uid);
			}
		} else {
			if (kilo_ticks % 100 == 0) {
				printf("[Kilobot ID%d] : Time to anchor is running.\n", kilo_uid);
			}
		}	
	}

	if (mydata->flag_timeToAnchorIsRunning) {
		set_color(RGB(0,0,3));	// blue : anchoring attempt
	} else {
		setNbNeighborsLed();
	}

	showNeighborsList();
	printf("-------------------------------------------------------------------------------------------\n");
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

