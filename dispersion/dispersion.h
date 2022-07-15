//-------------------------------------------------------------------------------
// CONSTANTS
//-------------------------------------------------------------------------------

// d04
#ifndef DESIRED_NBNEIGHBORS
#define DESIRED_NBNEIGHBORS 2 // authorized values : from 0 to 255 neighbors
#endif

// d06
#ifndef MAX_AUTHORIZED_NBNEIGHBORS
#define MAX_AUTHORIZED_NBNEIGHBORS 10 // authorized values : from 0 to 255 neighbors
#endif

// d
#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884197169399375105820974944
#endif




//-------------------------------------------------------------------------------
// USERDATA STRUCTURE
//-------------------------------------------------------------------------------

typedef struct {
  uint16_t id;
  uint16_t age;
  uint8_t distance;
} neighbor_t;


typedef struct {

  uint8_t flag_verbose;                 // boolean

  // Messages variables
  message_t transmit_msg;
  message_t transmit_msg1;
  message_t transmit_msg2;
  uint8_t flag_messageSent;             // boolean
  message_t rcvd_message;
  uint16_t rcvd_msg_id;
  uint8_t rcvd_msg_isItAnchor;          // boolean
  uint8_t flag_newMessage;              // boolean

  // d01
  uint32_t lastReset_runAndTumble;
  uint8_t flag_firstIteTimeToReset;    // boolean
  uint8_t currentDirection;

  // d03
  distance_measurement_t dist_measure;
  uint8_t dist;
  
  // d04
  uint32_t lastReset_timeToAnchor;
  uint8_t flag_neighborAlreadyAdded;    // boolean
  uint8_t flag_timeToAnchorIsRunning;   // boolean
  uint8_t flag_iAmAnchor;               // boolean
  uint8_t nbNeighbors;  // current nb of neighbors added to list_neighborsIds = current list size
  neighbor_t list_neighbors[MAX_AUTHORIZED_NBNEIGHBORS]; // list of detected different neighbors

  // d06
  uint8_t flag_isThereNeighborTooClose;   // boolean
  uint8_t flag_electionTimeIsRunning;     // boolean

} USERDATA;




//-------------------------------------------------------------------------------
// MEMO COLORS
//-------------------------------------------------------------------------------

/*
	//set_color(RGB(0,0,0)); // off

  //set_color(RGB(3,3,0)); // yellow <<<<<<<<<<<<<<<<<< secondary additive color
	//set_color(RGB(3,2,0)); // light orange
	//set_color(RGB(3,1,0)); // orange
  //set_color(RGB(2,2,0)); // light gold
	//set_color(RGB(1,1,0)); // dark gold
  //set_color(RGB(2,1,0)); // light brown
	//set_color(RGB(2,1,0)); // brown
  //set_color(RGB(1,0,0)); // dark brown
	//set_color(RGB(2,0,0)); // brown red
	//set_color(RGB(3,0,0)); // red <<<<<<<<<<<<<<<<<<<<< primary additive color
	//set_color(RGB(3,1,1)); // rose salmon
	//set_color(RGB(2,1,1)); // mauve
	//set_color(RGB(3,0,1)); // pink
	//set_color(RGB(2,0,1)); // bordeaux
  
  //set_color(RGB(3,0,3)); // magenta <<<<<<<<<<<<<<<<< secondary additive color
	//set_color(RGB(1,0,1)); // purple
	//set_color(RGB(0,0,1)); // violet
  //set_color(RGB(0,0,2)); // dark blue
	//set_color(RGB(0,0,3)); // blue <<<<<<<<<<<<<<<<<<<< primary additive color
	//set_color(RGB(0,1,3)); // light grey blue
  //set_color(RGB(0,1,2)); // grey blue
 	//set_color(RGB(0,2,3)); // light blue 

	//set_color(RGB(0,3,3)); // cyan <<<<<<<<<<<<<<<<<<<< secondary additive color
	//set_color(RGB(0,3,2)); // aquamarine
	//set_color(RGB(2,3,0)); // light green
	//set_color(RGB(0,3,1)); // light acid green
	//set_color(RGB(1,3,0)); // acid green
	//set_color(RGB(0,2,0)); // dark green
	//set_color(RGB(0,1,0)); // dark military green
	//set_color(RGB(1,2,0)); // light military green
	//set_color(RGB(0,3,0)); // green <<<<<<<<<<<<<<<<<<< primary additive color
	//set_color(RGB(0,2,1)); // light emerald
	//set_color(RGB(0,1,1)); // dark emerald

  //set_color(RGB(1,1,1)); // grey
	//set_color(RGB(3,3,3)); // white
*/



