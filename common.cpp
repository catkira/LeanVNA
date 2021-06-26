#include "common.hpp"
#include <string.h>

#define TRUE true
#define FALSE false

// NanoVNA Default settings
static const trace_t def_trace[TRACES_MAX] = {//enable, type, channel, reserved, scale, refpos
    { 1, TRC_LOGMAG, 0, 0, 1.0, 7.0 },
    { 1, TRC_LOGMAG, 1, 0, 1.0, 7.0 },
    { 1, TRC_SMITH,  0, 1, 1.0, 0.0 },
    { 1, TRC_PHASE,  0, 0, 1.0, 4.0 }
};

void properties_t::setFieldsToDefault() {
	magic = CONFIG_MAGIC;
	_frequency0   = 100000000;    // start = 100MHz
	_frequency1   = 900000000;    // end   = 900MHz
	_sweep_points = 101;
	_cal_status   = 0;
	_electrical_delay = 0.0;  
	_avg = 1;
	_adf4350_txPower = 3;
	_si5351_txPower = 1;
	_measurement_mode = MEASURE_MODE_FULL;

	setCalDataToDefault();
	memcpy(_trace, def_trace, sizeof(_trace));
}

void properties_t::setCalDataToDefault() {
  _cal_status = 0;
  do_cal_reset(CAL_LOAD, 0.f);
  do_cal_reset(CAL_OPEN, 1.f);
  do_cal_reset(CAL_SHORT, -1.f);
  do_cal_reset(CAL_THRU, 1.f);
  do_cal_reset(CAL_ISOLN_OPEN, 0.f);
  do_cal_reset(CAL_ISOLN_SHORT, 0.f);
  do_cal_reset(CAL_THRU_REFL, 0.f);
}
void properties_t::do_cal_reset(int calType, complexf val) {
  complexf* arr = _cal_data[calType];
  for(int i=0; i<SWEEP_POINTS_MAX; i++)
    arr[i] = val;
}
