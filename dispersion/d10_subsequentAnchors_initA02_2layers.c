// Study of the coverage area problem within a robotic swarm
// Alessia Loi 2022


//-------------------------------------------------------------------------------
// DESCRIPTION
//-------------------------------------------------------------------------------

// d06_subsequentAnchors.





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
const uint32_t kticks_max_electionTime = 2000;
const uint32_t kticks_min_timeToCollectNeighbors = 1000;

// distances expressed in mm
const uint8_t dist_max_runAvoiderBehavior = 35;
const uint8_t dist_min_between2Kbots = 55;

// others
const uint8_t max_nbDetectableNeighbors = 7;




//-------------------------------------------------------------------------------
// MESSAGES
//-------------------------------------------------------------------------------

// message transmission callback : returns the address of the message (transmit_msg) we declared
message_t *message_tx() {
	if (mydata->flag_iAmAnchor) {
		return &mydata->transmit_msg1;
	} else if (mydata->flag_iAmPreAnchor) {
		return &mydata->transmit_msg2;
	} else {
		return &mydata->transmit_msg3;
	}
}

//-------------------------------------------------------------------------------
	
// Initialize the messages to send : each kilobots sends its ID (uint16_t) and its flag_iAmAnchor value
void setMsg_myId_iAmAnchor() {
	mydata->transmit_msg1.type = NORMAL;
	mydata->transmit_msg1.data[0] = kilo_uid & 0xff; // 0 low ID
	mydata->transmit_msg1.data[1] = kilo_uid >> 8;	 // 1 high ID
	mydata->transmit_msg1.data[2] = 1;
	mydata->transmit_msg2.data[3] = 0;
	mydata->transmit_msg1.crc = message_crc(&mydata->transmit_msg1);
}

void setMsg_myId_iAmPreAnchor() {
	mydata->transmit_msg2.type = NORMAL;
	mydata->transmit_msg2.data[0] = kilo_uid & 0xff; // 0 low ID
	mydata->transmit_msg2.data[1] = kilo_uid >> 8;	 // 1 high ID
	mydata->transmit_msg2.data[2] = 0;	
	mydata->transmit_msg2.data[3] = 1;
	mydata->transmit_msg2.crc = message_crc(&mydata->transmit_msg2);
}

void setMsg_myId_iAmNotPreAnchor() {
	mydata->transmit_msg3.type = NORMAL;
	mydata->transmit_msg3.data[0] = kilo_uid & 0xff; // 0 low ID
	mydata->transmit_msg3.data[1] = kilo_uid >> 8;	 // 1 high ID
	mydata->transmit_msg3.data[2] = 0;
	mydata->transmit_msg3.data[3] = 0;
	mydata->transmit_msg3.crc = message_crc(&mydata->transmit_msg3);
}

//-------------------------------------------------------------------------------

void message_rx(message_t *msg, distance_measurement_t *d) {
	mydata->rcvd_msg_id = msg->data[0] | msg->data[1] << 8; // pointed value
	mydata->rcvd_msg_isItAnchor = msg->data[2];
	mydata->rcvd_msg_isItPreAnchor = msg->data[3];
	mydata->dist_measure = *d;
	mydata->dist = estimate_distance(&mydata->dist_measure);
	mydata->flag_newMessage = 1; 	// flag=1 means that a new message has arrived
	printf("%d, %d, %d, %d\n", msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
}




//-------------------------------------------------------------------------------
// MOTION FONCTIONS
//-------------------------------------------------------------------------------

void runAndTumbleWalk() {
	
	spinup_motors();
	
	if ((kilo_ticks > mydata->lastReset_runAndTumble + kticks_straightWalk + kticks_reorientationWalk)) {
		mydata->lastReset_runAndTumble = kilo_ticks - 1;
		mydata->currentDirection = rand_soft() % 2;
	}
	
	if (kilo_ticks < mydata->lastReset_runAndTumble + kticks_reorientationWalk) {
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
	if (mydata->flag_verbose) { printf("[Kilobot ID%d] : Adding kilobot ID%d to database...\n", kilo_uid, mydata->rcvd_msg_id); }
	if (mydata->nbNeighbors < MAX_AUTHORIZED_NBNEIGHBORS) {
		mydata->list_neighbors[mydata->nbNeighbors].id = mydata->rcvd_msg_id;
		mydata->list_neighbors[mydata->nbNeighbors].age = kilo_ticks;
		mydata->list_neighbors[mydata->nbNeighbors].distance = mydata->dist;
		mydata->list_neighbors[mydata->nbNeighbors].flag_isItAnchor = mydata->rcvd_msg_isItAnchor;
		mydata->list_neighbors[mydata->nbNeighbors].flag_isItPreAnchor = mydata->rcvd_msg_isItPreAnchor;

		mydata->nbAnchors = mydata->nbAnchors + mydata->list_neighbors[mydata->nbNeighbors].flag_isItAnchor;
		mydata->nbNeighbors = mydata->nbNeighbors + 1;

		if (mydata->flag_verbose) { printf("[Kilobot ID%d] : Kilobot ID%d has been added to list_neighbors.\n", kilo_uid, mydata->rcvd_msg_id); }
	} else {
		if (mydata->flag_verbose) { printf("[Kilobot ID%d] : Impossible to add kilobot ID%d to list_neighbors, no more space in database!\n", kilo_uid, mydata->rcvd_msg_id); }
	}
}

//-------------------------------------------------------------------------------

void removeAllNeighbors() {
	uint8_t i;
	for (i = 0; i < mydata->nbNeighbors; i++) {
		mydata->list_neighbors[i].id = NULL;
		mydata->list_neighbors[i].age = NULL;
		mydata->list_neighbors[i].distance = NULL;
		mydata->list_neighbors[i].flag_isItAnchor = NULL;
		mydata->list_neighbors[i].flag_isItPreAnchor = NULL;
	}
	mydata->nbAnchors = 0;
	mydata->nbNeighbors = 0;
	if (mydata->flag_verbose) { printf("[Kilobot ID%d] : All neighbors have been removed from list_neighbors.\n", kilo_uid); }
}

//-------------------------------------------------------------------------------

void isKbotInNeighborsList(uint16_t kBotId) {
	uint8_t i;
	mydata->flag_neighborAlreadyAdded = 0;
	for (i = 0; i < mydata->nbNeighbors; i++) {
		if (kBotId == mydata->list_neighbors[i].id) {
			mydata->flag_neighborAlreadyAdded = 1;
			if (mydata->flag_verbose) { printf("[Kilobot ID%d] : Kilobot ID%d is already in list_neighbors.\n", kilo_uid, kBotId); }
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
			if (mydata->flag_verbose) { printf("[Kilobot ID%d] : Kilobot ID%d is too close.\n", kilo_uid, mydata->list_neighbors[i].id); }
			break;
		}
	}
}

//-------------------------------------------------------------------------------

// kBotId removal and sliding following list_neighbors positions (from the oldest to the younger)
void slideNeighborsListFrom(uint16_t kBotId) {
	uint8_t i;
	uint8_t check = 0;
	for (i = 0; i < mydata->nbNeighbors - 1; i++){	
		if (mydata->list_neighbors[i].id == kBotId) { // sliding starts from this index i, previous neighbors are inchanged
			mydata->nbAnchors = mydata->nbAnchors - mydata->list_neighbors[i].flag_isItAnchor;
			check = 1;
			while (i < mydata->nbNeighbors - 1) { // sliding
				mydata->list_neighbors[i].id = mydata->list_neighbors[i+1].id;
				mydata->list_neighbors[i].age = mydata->list_neighbors[i+1].age;
				mydata->list_neighbors[i].distance = mydata->list_neighbors[i+1].distance;
				mydata->list_neighbors[i].flag_isItAnchor = mydata->list_neighbors[i+1].flag_isItAnchor;
				mydata->list_neighbors[i].flag_isItPreAnchor = mydata->list_neighbors[i+1].flag_isItPreAnchor;
				i++;
			}
			break;
		}	
	}

	if (!check) {
		mydata->nbAnchors = mydata->nbAnchors - mydata->list_neighbors[i].flag_isItAnchor;		
	}
	mydata->list_neighbors[i].id = NULL;
	mydata->list_neighbors[i].age = NULL;
	mydata->list_neighbors[i].distance = NULL;
	mydata->list_neighbors[i].flag_isItAnchor = NULL;
	mydata->list_neighbors[i].flag_isItPreAnchor = NULL;

	mydata->nbNeighbors = mydata->nbNeighbors - 1;
	if (mydata->flag_verbose) { printf("[Kilobot ID%d] : Kilobot ID%d has been removed from list_neighbors.\n", kilo_uid, kBotId); }
}

//-------------------------------------------------------------------------------

void updateKbot(uint16_t kBotId) {
	slideNeighborsListFrom(kBotId); // remove old kBotId
	addKbotToNeighborsList();		// add kBotId with updated values
	if (mydata->flag_verbose) { printf("[Kilobot ID%d] : (The age of the kilobot ID%d has been updated).\n", kilo_uid, kBotId); }
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
		if (mydata->flag_verbose) { printf("[Kilobot ID%d] : list_neighbors.     \t| POS\t| NEIGHBOR_ID\t| CAPTURE_TIME\t| DISTANCE\t| ISITANCHOR\t| ISITPREANCHOR\t(Size = nbNeighbors = %d) \t(nbAnchors = %d)\n", kilo_uid, mydata->nbNeighbors, mydata->nbAnchors); }
		for (i = 0; i < mydata->nbNeighbors; i++) {
			if (mydata->flag_verbose) { printf("\t\t\t\t\t| %d\t| %d\t\t| %d\t\t| %d\t\t| %d\t\t| %d\n", i, mydata->list_neighbors[i].id, mydata->list_neighbors[i].age, mydata->list_neighbors[i].distance, mydata->list_neighbors[i].flag_isItAnchor, mydata->list_neighbors[i].flag_isItPreAnchor); }
		}	
	}
}

//-------------------------------------------------------------------------------

// Management of the list_neighbors : add / update / remove the new neighbor detected in fonction of its distance and its old age.
void manageNeighborsList(uint16_t kBotId, uint8_t dist_min_between2Kbots) {
	isKbotInNeighborsList(kBotId); // sets the flag_neighborAlreadyAdded
	if (mydata->flag_neighborAlreadyAdded) {
		if (mydata->dist > dist_min_between2Kbots) {
			// Update the current neighbor capture time (age expressed in kilo_ticks)
			updateKbot(kBotId);
		} else {
			// Remove the current neighbor from list_neighbors because now its neighbor is too close
			slideNeighborsListFrom(kBotId);
		}
	} else {
		if (mydata->dist > dist_min_between2Kbots) {
			// Add the current neighbor to list_neighbors
			addKbotToNeighborsList(); // sets the nbNeighbors
		}
	}
}




//-------------------------------------------------------------------------------
// ANCHORING FONCTIONS
//-------------------------------------------------------------------------------

void initAnchors_02_plusCrowd() {
if (mydata->flag_electionTimeIsRunning) {
	if (kilo_ticks < mydata->lastReset_runAndTumble + kticks_max_electionTime - 300) { // mydata->lastReset_runAndTumble = starting time

		// During the kticks_max_electionTime period, the current kilobot collects the kilobots in the neighborhood
		if (mydata->flag_newMessage) {
			if (mydata->flag_verbose) { printf("[Kilobot ID%d] : Election algorithm - New message ! Kilobot ID%d has been detected at distance %d. isItAnchor = %d.\n", kilo_uid, mydata->rcvd_msg_id, mydata->dist, mydata->rcvd_msg_isItAnchor); }
			mydata->flag_newMessage = 0;

			// If at least one in the list_neighbors is an anchor, then the current kilobot isn't.
			if (mydata->rcvd_msg_isItAnchor) {
				if (mydata->flag_verbose) { printf("[Kilobot ID%d] : Election algorithm - My neighbor is an anchor, so I'm not an anchor.\n", kilo_uid); }
				set_color(RGB(0,3,0)); // green
				mydata->flag_electionTimeIsRunning = 0;
				removeAllNeighbors(); // initialize list_neighbors before starting the dispersion algorithm
				return;
			}
			manageNeighborsList(mydata->rcvd_msg_id, 0); // collect of all neighbors placed at any distance (from distance=0)
		}

		// At the end of the kticks_min_timeToCollectNeighbors period, if a kilobot has 0 or 1 only neighbor,
		// it becomes an anchor.
		if (kilo_ticks > mydata->lastReset_runAndTumble + kticks_min_timeToCollectNeighbors - 300) {
			if (mydata->nbNeighbors < 2) {
				mydata->flag_iAmAnchor = 1;
				if (mydata->flag_verbose) { printf("[Kilobot ID%d] : Election algorithm - I am anchor !\n", kilo_uid); }
			} else {
				if (mydata->nbNeighbors > max_nbDetectableNeighbors) {
					mydata->flag_iAmAnchor = 1;
					if (mydata->flag_verbose) { printf("[Kilobot ID%d] : Election algorithm - I am anchor because it's crowded !    O______O\n", kilo_uid); }
				}
			}
		}
	} else {
		// Last iteration for remaining kilobots
		mydata->flag_electionTimeIsRunning = 0;
		removeAllNeighbors(); // initialize list_neighbors before starting the dispersion algorithm
		set_color(RGB(3,0,0)); // red
	}
	return;
}


}




//-------------------------------------------------------------------------------
// SETUP, LOOP, MAIN
//-------------------------------------------------------------------------------

void setup() {

	// Verbose : set flag_verbose=1 if you want to see execution details on terminal, flag_verbose=0 if not.
	mydata->flag_verbose = 0;
	
	// Initialize the random generator
    while(get_voltage() == -1);
    rand_seed(rand_hard() + kilo_uid);

	// d01
	mydata->lastReset_runAndTumble = rand_soft(); // starting time
	mydata->currentDirection = 1;

	// d03
	mydata->dist = 100;

	// d04
	mydata->nbNeighbors = 0;
	mydata->flag_neighborAlreadyAdded = 0;
	mydata->lastReset_timeToAnchor = 0;
	mydata->flag_timeToAnchorIsRunning = 0;
	mydata->flag_iAmAnchor = 0;

	// d06
	mydata->flag_electionTimeIsRunning = 1;

	// d09
	mydata->nbAnchors = 0;

	//d10
	mydata->flag_iAmPreAnchor = 0;
	
	// messages d10
	mydata->flag_newMessage = 0;
	setMsg_myId_iAmAnchor();
	setMsg_myId_iAmPreAnchor();
	setMsg_myId_iAmNotPreAnchor();
}

//-------------------------------------------------------------------------------

void loop() {

	if (mydata->flag_iAmAnchor) {
		// Keep the current position
		set_color(RGB(3,0,3)); // magenta
		set_motors(0,0);
		return;
	}



	// -------- Initialisation : anchors'election algorithm --------
	
	if (kilo_ticks < kticks_max_electionTime) {
		initAnchors_02_plusCrowd();
		return;
	}



	// -------- Dispersion algorithm --------

	// Remove old neighbors and slide list_neighbors positions
	if ((mydata->nbNeighbors > 0) && ((kilo_ticks - mydata->list_neighbors[0].age) > kticks_max_authorizedNeighborAge)) {
		printf("[Kilobot ID%d] : Sliding list_neighbors (Kilobot ID%d is too old).\n", kilo_uid, mydata->list_neighbors[0].id, mydata->list_neighbors[0].id);
		slideNeighborsListFrom(mydata->list_neighbors[0].id);
	}
	

	/*

	// A new message has arrived : a kilobot in the neighborhood notifies its presence
	if (mydata->flag_newMessage) {
		if (mydata->flag_verbose) { printf("[Kilobot ID%d] : New message ! Kilobot ID%d has been detected at distance %d (min distance required between neighbors = %d).\n", kilo_uid, mydata->rcvd_msg_id, mydata->dist, dist_min_between2Kbots+1); }
		manageNeighborsList(mydata->rcvd_msg_id, 0);
		if (mydata->rcvd_msg_isItPreAnchor) {
			set_color(RGB(3,0,1));
		}
		mydata->flag_newMessage = 0;
	}






















	if (mydata->nbAnchors == DESIRED_NBNEIGHBORS) {
		// Keep the current position and try to fix it
		set_motors(0,0);
		set_color(RGB(0,0,3)); // blue
		if (!mydata->flag_timeToAnchorIsRunning) {
			mydata->flag_timeToAnchorIsRunning = 1;
			mydata->lastReset_timeToAnchor = kilo_ticks;
			if (mydata->flag_verbose) { printf("[Kilobot ID%d] : Time to anchor started at t = %d kiloticks.\n", kilo_uid, mydata->lastReset_timeToAnchor); }
		}
	} else if (mydata->nbAnchors > 0) {

 		set_color(RGB(0,2,3)); // light blue 
		avoiderBehavior();
	} else {
		if (!mydata->flag_timeToAnchorIsRunning) {
			// Keep walking to find a better zone
			keepWalking();
			set_color(RGB(3,3,3)); // white
		}
	}


	// flag_timeToAnchorIsRunning treatement (attempt to fix the current kilobot position)
	if (mydata->flag_timeToAnchorIsRunning) {
		// If at the end of the kticks_max_timeToAnchor period the current kilobot has still
		// DESIRED_NBNEIGHBORS neighbors, than it fixes its position
		if (kilo_ticks > mydata->lastReset_timeToAnchor + kticks_max_timeToAnchor) {
			if (mydata->nbAnchors == DESIRED_NBNEIGHBORS) {
				mydata->flag_iAmAnchor = 1;
				if (mydata->flag_verbose) { printf("[Kilobot ID%d] : I am anchor !\n", kilo_uid); }
			} else {
				mydata->flag_timeToAnchorIsRunning = 0;
				if (mydata->flag_verbose) { printf("[Kilobot ID%d] : Anchor failed.\n", kilo_uid); }
			}
		} else {
			if (kilo_ticks % 100 == 0) {
				if (mydata->flag_verbose) { printf("[Kilobot ID%d] : Time to anchor is running.\n", kilo_uid); }
			}
		}	
	}


	if (mydata->flag_timeToAnchorIsRunning) {
		set_color(RGB(0,0,3));	// blue : anchoring period
	} else {
		setNbNeighborsLed();
	}
*/

	showNeighborsList();
	if (mydata->flag_verbose) { printf("-------------------------------------------------------------------------------------------\n"); }
}




/*
		isThereNeighborTooClose(); // sets flag_isThereNeighborTooClose
		if (mydata->flag_isThereNeighborTooClose) {
			removeAllNeighbors();
			mydata->flag_timeToAnchorIsRunning = 0;
			printf("[Kilobot ID%d] : Anchor failed.\n", kilo_uid);
		}

		if (kilo_ticks > mydata->lastReset_timeToAnchor + kticks_max_timeToAnchor) {
			if (!mydata->flag_isThereNeighborTooClose) {
				mydata->flag_iAmAnchor = 1;
				printf("[Kilobot ID%d] : I am anchor !\n", kilo_uid);
			}
*/


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

