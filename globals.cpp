#include "globals.hpp"
#include <board.hpp>

volatile bool sweep_enabled = true;
int16_t vbat;

uint8_t registers[registerSize];

int16_t lastsaveid = 0;

config_t config = {
  .magic =             CONFIG_MAGIC,
  .dac_value =         1922,
  .default_loadcal =   0,
  .harmonic_freq_threshold = 300000000,
  .ui_options =        0,
  .checksum =          0
};

properties_t current_props;

complexf measuredFreqDomain[2][SWEEP_POINTS_MAX] alignas(8);
complexf measured[2][SWEEP_POINTS_MAX] alignas(8);

volatile EcalStates ecalState = ECAL_STATE_MEASURING;

__attribute__((used))
volatile int MEASUREMENT_NPERIODS_NORMAL = BOARD_MEASUREMENT_NPERIODS_NORMAL;
__attribute__((used))
volatile int MEASUREMENT_NPERIODS_CALIBRATING = BOARD_MEASUREMENT_NPERIODS_CALIBRATING;
__attribute__((used))
volatile int MEASUREMENT_ECAL_INTERVAL = BOARD_MEASUREMENT_ECAL_INTERVAL;
__attribute__((used))
volatile int MEASUREMENT_NWAIT_SWITCH = BOARD_MEASUREMENT_NWAIT_SWITCH;

uistat_t uistat = {
 digit: 6,
 digit_mode: 0,
 current_trace: 0,
 value: 0,
 lever_mode: LM_MARKER,
 previous_marker: 0,
 marker_search_mode: MarkerSearchModes::Max,
 marker_tracking: false,
 marker_delta: false
};

