// Study of the coverage area problem within a robotic swarm
// Alessia Loi 2022




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





// constants
uint32_t const kiloticks_random_walk_choice = 500;



// tools fonctions

void runAndTumbleWalk() {
	
	spinup_motors();
	if(kilo_ticks % (mydata->startingTime + (kiloticks_random_walk_choice * 2)) == 0) {
		mydata->lastReset = kilo_ticks;
		mydata->currentDirection = rand_soft() % 2;
	}
	
	if (kilo_ticks < mydata->lastReset + kiloticks_random_walk_choice) {
		set_color(RGB(3,0,0)); //red
	if (mydata->currentDirection == 0){
		set_motors(0, kilo_turn_right);
		} else {
			set_motors(kilo_turn_left, 0);
		}
	} else {
		set_color(RGB(0,3,0)); //green
		set_motors(kilo_straight_left, kilo_straight_right);
	}
}


void setup() {
	mydata->lastReset = 0;
	mydata->startingTime = rand_soft();
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

