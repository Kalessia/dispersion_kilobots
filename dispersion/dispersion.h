
//-------------------------------------------------------------------------------
// CONSTANTS
//-------------------------------------------------------------------------------

// d04
#ifndef MAX_AUTHORIZED_NBNEIGHBORS
#define MAX_AUTHORIZED_NBNEIGHBORS 3 // authorized values : from 0 to 255 neighbors
#endif

// d
#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884197169399375105820974944
#endif




/*
set_color(RGB(0,0,0)); 	// off
set_color(RGB(0,0,3)); 	// blue
set_color(RGB(3,0,3)); 	// magenta
set_color(RGB(3,0,0));  // red
set_color(RGB(3,3,0)); 	// yellow
set_color(RGB(0,3,0));  // green
set_color(RGB(0,3,3)); 	// cyan
set_color(RGB(3,3,3)); 	// white
*/




//-------------------------------------------------------------------------------
// USERDATA STRUCTURE
//-------------------------------------------------------------------------------

typedef struct {
  uint16_t id;
  uint16_t age;
} neighbor_t;


typedef struct {

  // Messages variables
  message_t transmit_msg;
  uint8_t flag_messageSent;       // boolean
  message_t rcvd_message;
  uint16_t rcvd_msg;
  uint8_t flag_newMessage;        // boolean

  // d01
  uint16_t lastReset;
  uint8_t startingTime;
  uint8_t currentDirection;

  // d03
  distance_measurement_t dist_measure;
  uint8_t dist;
  
  // d04
  uint8_t flag_correctNbNeighbors;  // boolean
  uint8_t flag_neighborAlreadyAdded;   // boolean
  uint8_t nbNeighbors;  // current nb of neighbors added to list_neighborsIds = current list size
  neighbor_t list_neighbors[MAX_AUTHORIZED_NBNEIGHBORS]; // list of detected different neighbors

} USERDATA;






