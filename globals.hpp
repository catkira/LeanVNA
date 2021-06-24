#pragma once
#include <stdint.h>
#include "common.hpp"

// global variables, to be defined in globals.cpp that can be accessed by all modules.
// DO NOT ADD STATEFUL FUNCTIONS HERE.
// think thrice before adding anything here.

// TODO(gabu-chan): eliminate these variables, make each module provide hooks
// for events that can be acted upon rather than let modules directly
// modify global state.

constexpr int registerSize = 256;
constexpr int registersSizeMask = registerSize - 1;
extern uint8_t registers[registerSize];

extern volatile bool sweep_enabled;

extern int16_t lastsaveid;
extern properties_t current_props;

extern config_t config;

// measured frequency domain data
extern complexf measuredFreqDomain[2][SWEEP_POINTS_MAX];

// measured data, possibly transformed
extern complexf measured[2][SWEEP_POINTS_MAX];


enum EcalStates {
	ECAL_STATE_MEASURING,
	ECAL_STATE_2NDSWEEP,
	ECAL_STATE_DONE
};
extern volatile EcalStates ecalState;

#define frequency0 current_props._frequency0
#define frequency1 current_props._frequency1
#define sweep_points current_props._sweep_points
#define cal_status current_props._cal_status
#define frequencies current_props._frequencies
#define cal_data current_props._cal_data

#define trace current_props._trace
#define domain_mode current_props._domain_mode
#define marker_smith_format current_props._marker_smith_format

extern volatile int MEASUREMENT_NPERIODS_NORMAL;
extern volatile int MEASUREMENT_NPERIODS_CALIBRATING;
extern volatile int MEASUREMENT_ECAL_INTERVAL;
extern volatile int MEASUREMENT_NWAIT_SWITCH;
