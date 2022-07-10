// Study of the coverage area problem within a robotic swarm
// Alessia Loi 2022


//-------------------------------------------------------------------------------
// DESCRIPTION
//-------------------------------------------------------------------------------

// d01_runAndTumble. 
// Each kilobot starts moving at a different randomly chosen time, given by startingTime.
// For a kticks_reorientationWalk period, kilobots turn and reorientate themselves,
// randomly on the right or on the left.
// For a kticks_straightWalk period, kilobots walk straight.

// Leds color code :
// Led red = kticks_reorientationWalk period, reorientation walk.
// Led green = kticks_straightWalk period, straight walk.

// Recommended parameters (circular arena disk.csv) :
// kticks_straightWalk = 500;
// kticks_reorientationWalk = 500;

// Adaptation for real kilobots ideas : 
//	- kticks_straightWalk : time required to walk an half arena distance
//	- kticks_reorientationWalk : time required for a 225° turn (opposite side, lateral) 




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
//  #define DEBUG 		// for printf to serial port
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
// FONCTIONS
//-------------------------------------------------------------------------------

void runAndTumbleWalk() {
	
	spinup_motors();
	
	if ((kilo_ticks > mydata->lastReset + kticks_straightWalk + kticks_reorientationWalk)) {
		mydata->lastReset = kilo_ticks - 1;
		mydata->currentDirection = rand_soft() % 2;
	}
	
	if (kilo_ticks < mydata->lastReset + kticks_reorientationWalk) {
		set_color(RGB(3,0,0)); 	// red
		if (mydata->currentDirection == 0) {
			set_motors(0, kilo_turn_right);
		} else {
			set_motors(kilo_turn_left, 0);
		}
	} else {
		set_color(RGB(0,3,0)); 	// green
		set_motors(kilo_straight_left, kilo_straight_right);
	}
}




//-------------------------------------------------------------------------------
// SETUP, LOOP, MAIN
//-------------------------------------------------------------------------------

void setup() {

	// Initialize the random generator
    while(get_voltage() == -1);
    rand_seed(rand_hard() + kilo_uid);

	mydata->lastReset = rand_soft(); // starting time
	printf("startingTime : %d\n", mydata->lastReset);
	mydata->currentDirection = 1;
}


void loop() {
	runAndTumbleWalk();
}


int main() {
	// Initialize kilobot hardware
	kilo_init();
	
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

