
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

// declare variables
typedef struct {

  // Messages variables
  message_t transmit_msg;
  uint8_t flag_messageSent; // boolean
  message_t rcvd_message;
  uint8_t flag_newMessage;  // boolean

  // d01
  uint16_t lastReset;
  uint8_t startingTime;
  uint8_t currentDirection;
  
} USERDATA;






