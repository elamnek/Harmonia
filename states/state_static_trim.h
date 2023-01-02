// state_static_trim.h

#ifndef _STATE_STATIC_TRIM_h
#define _STATE_STATIC_TRIM_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


#define K_P_DEPTH 1.0
#define K_I_DEPTH 1.0
#define K_D_DEPTH 1.0

#define MIN_DEPTH_CMD 10

#define K_P_STAT_PITCH 1.0
#define K_I_STAT_PITCH 1.0
#define K_D_STAT_PITCH 1.0

#define MIN_STATIC_PITCH_CMD 10


#define sgn(x) ((x<0) ? -1:((x>0) ? 1:0))

typedef struct {
	float err;
	float errInt;
	float errDer;
} t_control_error;

void update_error(float newErr, float dt, t_control_error* errUpdated);

void adjust_depth(t_control_error depthError);

void adjust_static_trim(t_control_error trimError);

#endif

