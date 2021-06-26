#pragma once
#include <stdint.h>
#include <complex>
#include <array>

using namespace std;


// common data structures, constants, and convenience functions.
// DO NOT ADD NON-CONSTANT VARIABLES OR STATEFUL FUNCTIONS HERE.


typedef complex<float> complexf;

// port 1 in, port 1 out, port 2 in
typedef array<complexf, 3> VNAObservation;

// a complete observation set for one frequency point
// (measured waves for excitations at each port).
// currently a single VNAObservation, but will be
// an array of 2 observations for a full two port VNA.
typedef VNAObservation VNAObservationSet;

// S11, S21
typedef array<complexf, 2> VNARawValue;

typedef int64_t freqHz_t;


// constants and data types used by all modules.
// DO NOT DECLARE EXTERNAL FUNCTIONS HERE!!!


#define VERSION "git"
#define FIRMWARE_MAJOR_VERSION 1
#define FIRMWARE_MINOR_VERSION 2
#define CH_KERNEL_VERSION "None"
#define PORT_COMPILER_NAME "gcc"
#define PORT_ARCHITECTURE_NAME "arm"
#define PORT_INFO "GD32F303"
#define PORT_CORE_VARIANT_NAME "N/A"
#define PLATFORM_NAME "BARE METAL"

#define FREQUENCY_MIN 10000
#define FREQUENCY_MAX 4400000000
// Frequencies below this are generated by the Si5351, above by the ADF4350
// The ADF4350 lower limit is 137Mhz, so it must be above 137Mhz
static constexpr uint32_t FREQUENCY_CHANGE_OVER	= 140000000;
#define SWEEP_POINTS_MIN 2
#ifndef SWEEP_POINTS_MAX
#define SWEEP_POINTS_MAX 201
#endif

#define TRACES_MAX 4

#define ECAL_PARTIAL

#ifdef ECAL_PARTIAL
#define ECAL_CHANNELS 1
#else
#define ECAL_CHANNELS 3
#endif

#define CAL_ENTRIES 7
#define CAL_LOAD 0
#define CAL_OPEN 1
#define CAL_SHORT 2
#define CAL_THRU 3
#define CAL_ISOLN_OPEN 4
#define CAL_ISOLN_SHORT 5
#define CAL_THRU_REFL 6

#define CALSTAT_LOAD (1<<0)
#define CALSTAT_OPEN (1<<1)
#define CALSTAT_SHORT (1<<2)
#define CALSTAT_THRU (1<<3)
#define CALSTAT_ISOLN (1<<4)
#define CALSTAT_ES (1<<5)
#define CALSTAT_ER (1<<6)
#define CALSTAT_ET (1<<7)
#define CALSTAT_ED CALSTAT_LOAD
#define CALSTAT_EX CALSTAT_ISOLN
#define CALSTAT_APPLY (1<<8)
#define CALSTAT_INTERPOLATED (1<<9)
#define CALSTAT_ENHANCED_RESPONSE (1<<10)

#define ETERM_ED 0 /* error term directivity */
#define ETERM_ES 1 /* error term source match */
#define ETERM_ER 2 /* error term refrection tracking */
#define ETERM_ET 3 /* error term transmission tracking */
#define ETERM_EX 4 /* error term isolation */

/* Determines the measurements the ADC will do. */
enum MeasurementMode {
    MEASURE_MODE_FULL, //Including ECAL, slowest
    MEASURE_MODE_REFL_THRU, //Does not switch the output, use for CW mode
    MEASURE_MODE_REFL_THRU_REFRENCE, //No ecal
};

constexpr uint32_t BOOTLOADER_BOOTLOAD_MAGIC = 0xdeadbabe;
static volatile uint32_t& bootloaderBootloadIndicator = *(uint32_t*)(0x20000000 + 48*1024 - 4);

enum {
  TRC_LOGMAG, TRC_PHASE, TRC_DELAY, TRC_SMITH, TRC_POLAR, TRC_LINEAR, TRC_SWR, TRC_REAL, TRC_IMAG, TRC_R, TRC_X, TRC_Q, TRC_OFF
};

enum SweepParameter {
  ST_START, ST_STOP, ST_CENTER, ST_SPAN, ST_CW
};

typedef struct {
  uint8_t enabled;
  uint8_t type;
  uint8_t channel;
  uint8_t polar;
  float scale;
  float refpos;
} trace_t;

struct alignas(4) properties_t {
  uint32_t magic;
  freqHz_t _frequency0; // start
  freqHz_t _frequency1; // stop
  int16_t _sweep_points;
  uint16_t _cal_status;

  complexf _cal_data[CAL_ENTRIES][SWEEP_POINTS_MAX];
  float _electrical_delay; // picoseconds

  trace_t _trace[TRACES_MAX];
  uint8_t _avg;
  uint8_t _adf4350_txPower; // 0 to 3
  uint8_t _si5351_txPower; // 0 to 3
  uint8_t _measurement_mode; //See enum MeasurementMode.

  uint32_t checksum;

  // overwrite all fields of this instance with factory default values
  void setFieldsToDefault();

  // clear calibration data
  void setCalDataToDefault();

  properties_t() { setFieldsToDefault(); }
  freqHz_t startFreqHz() const {
    if(_frequency1 > 0) return _frequency0;
    return _frequency0 + _frequency1/2;
  }
  freqHz_t stopFreqHz() const {
    if(_frequency1 > 0) return _frequency1;
    return _frequency0 - _frequency1/2;
  }
  freqHz_t stepFreqHz() const {
    if(_sweep_points > 0)
      return (stopFreqHz() - startFreqHz()) / (_sweep_points - 1);
    return 0;
  }

  void do_cal_reset(int calType, complexf val);
};

typedef struct {
  uint32_t magic;
  uint32_t checksum;
} config_t;

#define CONFIG_MAGIC 0x8008123c

static inline bool is_freq_for_adf4350(freqHz_t freq)
{
	return freq > FREQUENCY_CHANGE_OVER;
}

static const struct {
  const char *name;
  uint16_t refpos;
  float scale_unit;
} trace_info[] = {
  { "LOGMAG", 7, 10 },
  { "PHASE",  4, 90 },
  { "DELAY",  4,  1 },
  { "SMITH",  0,  1 },
  { "POLAR",  0,  1 },
  { "LINEAR", 0,  0.125 },
  { "SWR",    0,  1 },
  { "REAL",   4,  0.25 },
  { "IMAG",   4,  0.25 },
  { "R",      0, 100 },
  { "X",      4, 100 },
  { "Q",      0, 10.0 }
};