// Study of the coverage area problem within a robotic swarm
// Alessia Loi 2022


//-------------------------------------------------------------------------------
// DESCRIPTION
//-------------------------------------------------------------------------------





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
const uint32_t kticks_max_timeToAnchor = 3000;

// distances expressed in mm
const uint8_t dist_max_runAvoiderBehavior = 35;
const uint8_t dist_min_between2Kbots = 60;




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
	if (mydata->nbNeighbors < DESIRED_NBNEIGHBORS) {
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

void deleteAllNeighbors() {
	uint8_t i;
	for (i = 0; i < mydata->nbNeighbors; i++) {
		mydata->list_neighbors[i].id = NULL;
		mydata->list_neighbors[i].age = NULL;
		mydata->list_neighbors[i].distance = NULL;
	}
	mydata->nbNeighbors = 0;
	printf("[Kilobot ID%d] : All neighbors have been removed from list_neighbors.\n", kilo_uid);
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

void isThereNeighborTooClose() {
	uint8_t i;
	mydata->flag_isThereNeighborTooClose = 0;
	for (i = 0; i < mydata->nbNeighbors; i++) {
		if (mydata->list_neighbors[i].distance <= dist_min_between2Kbots) {
			mydata->flag_isThereNeighborTooClose = 1;
			printf("[Kilobot ID%d] : Kilobot ID%d is too close.\n", kilo_uid, mydata->list_neighbors[i].id);
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
			set_color(RGB(3,2,0)); // light orange
			break;
		case 3 :
			set_color(RGB(3,1,0)); // orange
			break;
		case 4 :
  			set_color(RGB(2,2,0)); // light gold
			break;
		case 5 :
			set_color(RGB(1,1,0)); // dark gold
			break;
		default :
  			set_color(RGB(1,0,0)); // dark brown
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
// 
//-------------------------------------------------------------------------------

void amIAnchor(){
	if (kilo_uid == 0) {
		mydata->flag_iAmAnchor = 1;
	}
}





//-------------------------------------------------------------------------------
// SETUP, LOOP, MAIN
//-------------------------------------------------------------------------------

void setup() {

	// Verbose : set flag_verbose=1 if you want to see execution details on terminal, flag_verbose=0 if not.
	mydata->flag_verbose = 1;
	
	// Initialize the random generator
    while(get_voltage() == -1);
    rand_seed(rand_hard() + kilo_uid);

	// d01
	mydata->lastReset = rand_soft(); // starting time
	mydata->currentDirection = 1;

	// d03
	mydata->dist = 100;

	// d04
	mydata->nbNeighbors = 0;
	mydata->flag_neighborAlreadyAdded = 0;
	mydata->lastReset_timeToAnchor = 0;
	mydata->flag_timeToAnchorIsRunning = 0;
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

	

	// Remove old neighbors and slide list_neighbors positions
	if ((mydata->nbNeighbors > 0) && ((kilo_ticks - mydata->list_neighbors[0].age) > kticks_max_authorizedNeighborAge)) {
		printf("[Kilobot ID%d] : Sliding list_neighbors (Kilobot ID%d is too old).\n", kilo_uid, mydata->list_neighbors[0].id, mydata->list_neighbors[0].id);
		slideNeighborsListFrom(mydata->list_neighbors[0].id);
	}
	

	if (mydata->flag_newMessage) {
		printf("[Kilobot ID%d] : New message ! Kilobot ID%d has been detected at distance %d. isItAnchor = %d.\n", kilo_uid, mydata->rcvd_msg_id, mydata->dist, mydata->rcvd_msg_isItAnchor);

		if (mydata->rcvd_msg_isItAnchor) {
			// Management of a list_neighbors composed by anchors
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
				if (mydata->dist > dist_min_between2Kbots) {
					addKbotToNeighborsList(); // sets the nbNeighbors
				}
			}	
		} 
		mydata->flag_newMessage = 0;
	}

	if ((!mydata->flag_timeToAnchorIsRunning) && (mydata->nbNeighbors == DESIRED_NBNEIGHBORS)) {
		mydata->flag_timeToAnchorIsRunning = 1;
		mydata->flag_firstIteTimeToAnchor = 1;
	}

	// flag_timeToAnchorIsRunning treatement (attempt to fix the current kilobot position)
	if (mydata->flag_timeToAnchorIsRunning) {
		if (mydata->flag_firstIteTimeToAnchor) {
			mydata->lastReset_timeToAnchor = kilo_ticks;
			mydata->flag_firstIteTimeToAnchor = 0;
			printf("[Kilobot ID%d] : Time to anchor started at t = %d kiloticks.\n", kilo_uid, mydata->lastReset_timeToAnchor);
		}
		
		set_color(RGB(0,0,3)); // blue
		set_motors(0,0);

		isThereNeighborTooClose(); // sets flag_isThereNeighborTooClose
		if (mydata->flag_isThereNeighborTooClose) {
			deleteAllNeighbors();
			mydata->flag_timeToAnchorIsRunning = 0;
			printf("[Kilobot ID%d] : Anchor failed.\n", kilo_uid);
		}

		if (kilo_ticks > mydata->lastReset_timeToAnchor + kticks_max_timeToAnchor) {
			if (!mydata->flag_isThereNeighborTooClose) {
				mydata->flag_iAmAnchor = 1;
				printf("[Kilobot ID%d] : I am anchor !\n", kilo_uid);
			}
		} else {
			if (kilo_ticks % 100 == 0) {
				printf("[Kilobot ID%d] : Time to anchor is running.\n", kilo_uid);
			}
		}

	} else {
		// Keep walking to find a better zone
		set_color(RGB(3,3,3)); // white
		keepWalking();
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

