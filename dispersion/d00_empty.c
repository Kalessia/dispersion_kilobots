// Study of the coverage area problem within a robotic swarm
// Alessia Loi 2022



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




//-------------------------------------------------------------------------------
// FONCTIONS
//-------------------------------------------------------------------------------

void set_motion(motion_t new_motion){
}


void message_rx(message_t *m, distance_measurement_t *d) {
}


void setup_message(void) {
}




//-------------------------------------------------------------------------------
// SETUP, LOOP, MAIN
//-------------------------------------------------------------------------------

void setup() {
}


void loop() {
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

