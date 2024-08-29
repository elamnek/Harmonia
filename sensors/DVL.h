// DVL.h

#ifndef _DVL_h
#define _DVL_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

String init_dvl();

void read_dvl();

uint8_t crc8Check(uint8_t* message, int message_length);

void start_dvl_deadreckoning();
void calibrate_dvl();
void disable_acoustic_dvl();
void enable_acoustic_dvl();

String CheckMessage();

//dead reckoning report
uint8_t get_wrp_crc8_result();
char* get_wrp_checksum();
String get_wrp_data();
double get_dvldeadreckoning_x();
double get_dvldeadreckoning_y();
double get_dvldeadreckoning_z();
double get_dvldeadreckoning_stdDev();
double get_dvldeadreckoning_roll();
double get_dvldeadreckoning_pitch();
double get_dvldeadreckoning_yaw();
int get_dvldeadreckoning_status();

//velocity report
uint8_t get_wrz_crc8_result();
char* get_wrz_checksum();
String get_wrz_data();
double get_dvlvelocity_x();
double get_dvlvelocity_y();
double get_dvlvelocity_z();
double get_dvlvelocity_alt();
char get_dvlvelocity_valid();

//for future use
typedef struct {

	double x;
	double y;
	double z;

	double roll;
	double pitch;
	double yaw;
	
} dvl_state;


#endif
