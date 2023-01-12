#include <kilombo.h>

#ifdef SIMULATOR
#include <jansson.h>
json_t *json_state();
#else
#include <stdlib.h>
//#include <avr/io.h>   // for microcontroller register defs
//  #define DEBUG       // for printf to serial port
//  #include "debug.h"
#endif

#include "simulation.h"
typedef struct 
{
    int target_dist; //in mm

    int current_motion;
    int new_message;
    int dist;
    int nb_messages;
    message_t message;
    uint32_t last_motion_update; 
} USERDATA;
REGISTER_USERDATA(USERDATA)

#define STOP 0
#define FORWARD 1
#define LEFT 2
#define RIGHT 3
#define TH 450 // Time threshold to lower distance from neighbours in seconds

/* Idea: Reduce distance from neighbours over time. */
/* Idea to improve: Once stopped, kilobots keep a list of the distances that they receive messages from, it could help determine if a kilobot is very far away from some and not others. The problem with this is that this might be normal if the opt distance between kilobots is less than 80mm. (might receive comms from other kilobots further. But this may also be unlikely bc 80mm is about the space for max 2 kilobots that would be stuck together close.) */

// Function to handle motion.
void set_motion(int new_motion)
{
    // Only take an action if the motion is being changed.
    if (mydata->current_motion != new_motion)
    {
        mydata->current_motion = new_motion;
        
        if (mydata->current_motion == STOP)
        {
            set_motors(0, 0);
        }
        else if (mydata->current_motion == FORWARD)
        {
            spinup_motors();
            set_motors(kilo_straight_left, kilo_straight_right);
        }
        else if (mydata->current_motion == LEFT)
        {
            spinup_motors();
            set_motors(kilo_turn_left, 0);
        }
        else if (mydata->current_motion == RIGHT)
        {
            spinup_motors();
            set_motors(0, kilo_turn_right);
        }
    }
}

void setup()
{

    mydata->target_dist = 80; //in mm

    mydata->current_motion = STOP;
    mydata->new_message = 0;
    mydata->dist = 0;
    mydata->nb_messages = 0;
    mydata->last_motion_update = 0; 
    // Initialize an empty message.
    mydata->message.type = NORMAL;
    mydata->message.crc = message_crc(&mydata->message);
}

void loop()
{
    if (kilo_ticks % (TH*32) == 0)
    {
        mydata->target_dist = mydata->target_dist - 5;
    }

    if (kilo_ticks > mydata->last_motion_update + 32)
    {
        mydata->last_motion_update = kilo_ticks;
        
        // If a message was received within the last second and less than 50mm away, set a random motion.
        if (mydata->new_message == 1 && mydata->dist < mydata->target_dist)
        {
            mydata->new_message = 0;

            if (mydata->current_motion == STOP) 
            {
                if (mydata->nb_messages < 5)
                {
                    mydata->nb_messages += 1;
                    return;
                }
            }
            
            // Generate an 8-bit random number (between 0 and 2^8 - 1 = 255).
            int random_number = rand_hard();
            
            // Compute the remainder of random_number when divided by 4.
            // This gives a new random number in the set {0, 1, 2, 3}.
            int random_direction = (random_number % 4);
            
            // There is a 50% chance of random_direction being 0 OR 1, in which
            // case set the LED green and move forward.
            if ((random_direction == 0) || (random_direction == 1))
            {
                set_color(RGB(0, 1, 0));
                set_motion(FORWARD);
            }
            // There is a 25% chance of random_direction being 2, in which case
            // set the LED red and move left.
            else if (random_direction == 2)
            {
                set_color(RGB(1, 0, 0));
                set_motion(LEFT);
            }
            // There is a 25% chance of random_direction being 3, in which case
            // set the LED blue and move right.
            else if (random_direction == 3)
            {
                set_color(RGB(0, 0, 1));
                set_motion(RIGHT);
            }
        }
        // If no messages were received within the last second, set the LED white
        // and stop moving.
        else
        {
            set_color(RGB(1, 1, 1));
            set_motion(STOP);
            if (mydata->new_message != 1){
                mydata->nb_messages = 0;
            }
            
        }
    }
}

void message_rx(message_t *m, distance_measurement_t *d)
{
    mydata->new_message = 1;
    mydata->dist = estimate_distance(d);
}

message_t *message_tx()
{
    return &mydata->message;
}

int main()
{
    kilo_init();
    kilo_message_rx = message_rx;
    kilo_message_tx = message_tx;
    kilo_start(setup, loop);
    
    // Simulator settings
    SET_CALLBACK(global_setup, global_setup);
    SET_CALLBACK(botinfo, botinfo);
    SET_CALLBACK(reset, setup);
    SET_CALLBACK(json_state, json_state); // Saving bot state as json. Not for use in the real bot, only in the simulator.
    SET_CALLBACK(obstacles, obstacles_walls);
    

    return 0;
}
