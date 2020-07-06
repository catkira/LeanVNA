#include "globals.hpp"

volatile bool sweep_enabled = true;
int16_t vbat;

uint8_t registers[registerSize];

int16_t lastsaveid = 0;

config_t config = {
  .magic =             CONFIG_MAGIC,
  .dac_value =         1922,
  .checksum =          0
};


complexf measuredFreqDomain[2][SWEEP_POINTS_MAX] alignas(8);
complexf measured[2][SWEEP_POINTS_MAX] alignas(8);
complexf measuredEcal[ECAL_CHANNELS][USB_POINTS_MAX] alignas(8);

volatile EcalStates ecalState = ECAL_STATE_MEASURING;

