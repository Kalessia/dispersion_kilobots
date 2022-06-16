
#ifndef DISPERSION_H_
#define DISPERSION_H_

#define MAXN 20
//#define MAXN 16
#define RB_SIZE 8   // Ring buffer size. Choose a power of two for faster code

#define REALROBOTS_CUTOFF 85
//#define CUTOFF 85 //neighbors further away are ignored. (mm)
#define CUTOFF 10000 //neighbors further away are ignored. (mm)

#define ENABLE_AVG_AVG_LAMBDA
//#define ENABLE_INIT_REMOVE_SUM_X
#define ENABLE_PRE_DIFFUSION


typedef float base_t;
typedef float ext_t;
typedef float float_t;
#define ABS(x) fabs(x)

extern float initial_x_max_val;


void set_color_from_nb_neighbours();

// declare variables

typedef enum {
    DATA_NULL = 5,  // Cf kilolib/message.h --> range btw 0x02 and 0x79 are available
    DATA_TODO,
} data_type_t;

typedef struct {
    uint16_t ID;
    uint8_t dist;
    uint32_t timestamp;

    ext_t val;
    data_type_t data_type;
} Neighbor_t;

typedef struct {
    message_t msg;
    distance_measurement_t dist;
} received_message_t;



typedef struct {
    Neighbor_t neighbors[MAXN];
    uint8_t N_Neighbors;

    message_t msg;

    received_message_t RXBuffer[RB_SIZE];
    uint8_t RXHead, RXTail;

    data_type_t data_type;

    ext_t val;
} USERDATA;

extern USERDATA *mydata;



#endif

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
