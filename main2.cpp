/*
 * This file is derived from libopencm3 example code.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#define PRNT(x)
#define PRNTLN(x)
#include <mculib/fastwiring.hpp>
#include <mculib/softi2c.hpp>
#include <mculib/si5351.hpp>
#include <mculib/dma_adc.hpp>
#include <mculib/usbserial.hpp>
#include <mculib/printf.hpp>
#include <mculib/printk.hpp>

#include <array>
#include <complex>

#include "main.hpp"
#include <board.hpp>
#include "common.hpp"
#include "globals.hpp"
#include "synthesizers.hpp"
#include "vna_measurement.hpp"
#include "fifo.hpp"
#include "flash.hpp"
#include "calibration.hpp"
#include "command_parser.hpp"
#include "stream_fifo.hpp"
#include "sin_rom.hpp"
#include "gain_cal.hpp"

#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/vector.h>

using namespace mculib;
using namespace std;
using namespace board;

// see https://lists.debian.org/debian-gcc/2003/07/msg00057.html
// this can be any value since we are not using shared libraries.
void* __dso_handle = (void*) &__dso_handle;

static bool outputRawSamples = false;
int cpu_mhz = 8; /* The CPU boots on internal (HSI) 8Mhz */


static int lo_freq = 12000; // IF frequency, Hz
static int adf4350_freqStep = 12000; // adf4350 resolution, Hz

static USBSerial serial;

static const int adcBufSize=1024;	// must be power of 2
static volatile uint16_t adcBuffer[adcBufSize];

static VNAMeasurement vnaMeasurement;
static CommandParser cmdParser;
static StreamFIFO cmdInputFIFO;
static uint8_t cmdInputBuffer[128];


static float gainTable[RFSW_BBGAIN_MAX+1];

struct usbDataPoint {
	VNAObservation value;
	int freqIndex;
};
static usbDataPoint usbTxQueue[256];
static constexpr int usbTxQueueMask = 255;
static volatile int usbTxQueueWPos = 0;
static volatile int usbTxQueueRPos = 0;


static uint16_t ADCValueQueue[8192];
static constexpr int ADCValueQueueMask = 8191;
static volatile int ADCValueQueueWPos = 0;
static volatile int ADCValueQueueRPos = 0;

// periods of a 1MHz clock; how often to call adc_process()
static constexpr int tim1Period = 25;	// 1MHz / 25 = 40kHz

// value is in microseconds; increments at 40kHz by TIM1 interrupt
volatile uint32_t systemTimeCounter = 0;

static FIFO<small_function<void()>, 8> eventQueue;

static volatile bool usbDataMode = false;

static freqHz_t currFreqHz = 0;		// current hardware tx frequency
static int currThruGain = 0;		// gain setting used for this thru measurement

// if nonzero, any ecal data in the next ecalIgnoreValues data points will be ignored.
// this variable is decremented every time a data point arrives, if nonzero.
static volatile int ecalIgnoreValues = 0;
static volatile int collectMeasurementType = -1;
static int collectMeasurementOffset = -1;
static int collectMeasurementState = 0;
static small_function<void()> collectMeasurementCB;

static void adc_process();
static int measurementGetDefaultGain(freqHz_t freqHz);


#define myassert(x) if(!(x)) do { errorBlink(3); } while(1)

template<unsigned int N>
static inline void pinMode(const array<Pad, N>& p, int mode) {
	for(int i=0; i<(int)N; i++)
		pinMode(p[i], mode);
}

static void errorBlink(int cnt) {
	digitalWrite(led, HIGH);
	while (1) {
		for(int i=0;i<cnt;i++) {
			digitalWrite(led, HIGH);
			delay(200);
			digitalWrite(led, LOW);
			delay(200);
		}
		delay(1000);
	}
}

// period is in units of us
static void startTimer(uint32_t timerDevice, int period) {
	// set the timer to count one tick per us
	timer_set_mode(timerDevice, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_prescaler(timerDevice, cpu_mhz-1);
	timer_set_repetition_counter(timerDevice, 0);
	timer_continuous_mode(timerDevice);

	// this doesn't really set the period, but the "autoreload value"; actual period is this plus 1.
	// this should be fixed in libopencm3.

	timer_set_period(timerDevice, period - 1);

	timer_enable_preload(timerDevice);
	timer_enable_preload_complementry_enable_bits(timerDevice);
	timer_enable_break_main_output(timerDevice);

	timer_enable_irq(timerDevice, TIM_DIER_UIE);

	TIM_EGR(timerDevice) = TIM_EGR_UG;
	timer_set_counter(timerDevice, 0);
	timer_enable_counter(timerDevice);
}


static void dsp_timer_setup() {
	rcc_periph_clock_enable(RCC_TIM1);
	rcc_periph_reset_pulse(RST_TIM1);
	// set tim1 to highest priority
	nvic_set_priority(NVIC_TIM1_UP_IRQ, 0x00);
	nvic_enable_irq(NVIC_TIM1_UP_IRQ);
	startTimer(TIM1, tim1Period);
}

extern "C" void tim1_up_isr() {
	TIM1_SR = 0;
	systemTimeCounter += tim1Period;
	adc_process();
}


static int si5351_doUpdate(uint32_t freqHz) {
	// si5351 code seems to give high frequency errors when frequency
	// isn't a multiple of 10Hz. TODO: investigate
	freqHz = (freqHz/10) * 10;
	return synthesizers::si5351_set(freqHz+lo_freq, freqHz);
}

static int si5351_update(uint32_t freqHz) {
	static uint32_t prevFreq = 0;
	int ret = si5351_doUpdate(freqHz);
	if(freqHz < prevFreq)
		si5351_doUpdate(freqHz);
	prevFreq = freqHz;
	return ret;
}



static void adf4350_setup() {
	adf4350_rx.N = 120;
	adf4350_rx.rfPower = 0b00;
	adf4350_rx.sendConfig();
	adf4350_rx.sendN();

	adf4350_tx.N = 120;
	adf4350_tx.rfPower = 0b11;
	adf4350_tx.sendConfig();
	adf4350_tx.sendN();
}

static void adf4350_update(freqHz_t freqHz) {
	freqHz = freqHz_t(freqHz/adf4350_freqStep)*adf4350_freqStep;
	synthesizers::adf4350_set(adf4350_tx, freqHz, adf4350_freqStep);
	synthesizers::adf4350_set(adf4350_rx, freqHz + lo_freq, adf4350_freqStep);
}

/* Powerdown both devices */
static void adf4350_powerdown(void) {
	adf4350_tx.sendPowerDown();
	adf4350_rx.sendPowerDown();
}

static void adf4350_powerup(void) {
	adf4350_tx.sendPowerUp();
	adf4350_rx.sendPowerUp();
}

// automatically set IF frequency depending on rf frequency and board parameters
static void updateIFrequency(freqHz_t txFreqHz) {
	// adf4350 freq step and thus IF frequency must be a divisor of the crystal frequency
	if(xtalFreqHz == 20000000 || xtalFreqHz == 40000000) {
		// 6.25/12.5kHz IF
		if(txFreqHz >= 100000) {
			lo_freq = 12500;
			adf4350_freqStep = 12500;
			vnaMeasurement.setCorrelationTable(sinROM24x2, 48);
		} else {
			lo_freq = 6250;
			adf4350_freqStep = 6250;
			vnaMeasurement.setCorrelationTable(sinROM48x1, 48);
		}
	} else {
		// 6.0/12.0kHz IF
		if(txFreqHz >= 100000) {
			lo_freq = 12000;
			adf4350_freqStep = 12000;
			vnaMeasurement.setCorrelationTable(sinROM25x2, 50);
		} else {
			lo_freq = 6000;
			adf4350_freqStep = 6000;
			vnaMeasurement.setCorrelationTable(sinROM50x1, 50);
		}
	}
}

// set the measurement frequency including setting the tx and rx synthesizers
static void setFrequency(freqHz_t freqHz) {
	currFreqHz = freqHz;
	updateIFrequency(freqHz);
	if(!outputRawSamples)	
		rfsw(RFSW_BBGAIN, RFSW_BBGAIN_GAIN(measurementGetDefaultGain(currFreqHz)));

	// use adf4350 for f > 140MHz
	if(is_freq_for_adf4350(freqHz)) {
		adf4350_update(freqHz);
		rfsw(RFSW_TXSYNTH, RFSW_TXSYNTH_HF);
		rfsw(RFSW_RXSYNTH, RFSW_RXSYNTH_HF);
		vnaMeasurement.nWaitSynth = 10;
	} else {
		int ret = si5351_update(freqHz);
		rfsw(RFSW_TXSYNTH, RFSW_TXSYNTH_LF);
		rfsw(RFSW_RXSYNTH, RFSW_RXSYNTH_LF);
		if(ret == 0)
			vnaMeasurement.nWaitSynth = 18;
		if(ret == 1)
			vnaMeasurement.nWaitSynth = 60;
		if(ret == 2)
			vnaMeasurement.nWaitSynth = 60;
	}
}

static void adc_setup() {
	static uint8_t channel_array[1] = {adc_rxChannel};
	dmaADC.buffer = adcBuffer;
	dmaADC.bufferSizeBytes = sizeof(adcBuffer);
	dmaADC.init(channel_array, 1);

	adc_set_sample_time_on_all_channels(dmaADC.adcDevice, adc_ratecfg);
	dmaADC.start();
}

// read and consume data from the adc ring buffer
static void adc_read(volatile uint16_t*& data, int& len) {
	static uint32_t lastIndex = 0;
	uint32_t cIndex = dmaADC.position();
	uint32_t bufWords = dmaADC.bufferSizeBytes / 2;
	cIndex &= (bufWords-1);

	data = ((volatile uint16_t*) dmaADC.buffer) + lastIndex;
	if(cIndex >= lastIndex) {
		len = cIndex - lastIndex;
	} else {
		len = bufWords - lastIndex;
	}
	lastIndex += len;
	if(lastIndex >= bufWords) lastIndex = 0;
}


static void lcd_and_ui_setup() {
	lcd_spi_init();

	pinMode(ili9341_cs, OUTPUT);
	pinMode(xpt2046_cs, OUTPUT);
	digitalWrite(ili9341_cs, HIGH);	// disable LCD
	digitalWrite(xpt2046_cs, HIGH); // disable touch screen
	
	lcd_spi_fast();
}

static complexf ecalApplyReflection(complexf refl, int freqIndex) {
	#ifdef ECAL_PARTIAL
		return refl - measuredEcal[0][freqIndex];
	#else
		return SOL_compute_reflection(
					measuredEcal[1][freqIndex],
					1.f,
					measuredEcal[0][freqIndex],
					refl);
	#endif
}


static complexf applyFixedCorrections(complexf refl, freqHz_t freq) {
	// These corrections do not affect calibrated measurements
	// and is only there to fix uglyness when uncalibrated and
	// without full ecal.

	// magnitude correction:
	// - Near DC the balun is ineffective and measured refl is
	//   0 for short circuit, 0.5 for load, and 1.0 for open circuit,
	//   requiring a correction of (refl*2 - 1.0).
	// - Above 5MHz no correction is needed.
	// - Between DC and 5MHz we apply something in between, with
	//   interpolation factor defined by a polynomial that is
	//   experimentally determined.

	if(freq < 5000000) {
		float x = float(freq) * 1e-6 * (3./5.);
		x = 1 - x*(0.7 - x*(0.141 - x*0.006));
		refl = refl * (1.f + x) - x;
	}

	// phase correction; experimentally determined polynomial
	// x: frequency in MHz
	// arg = -0.25 * x * (-1.39 + x*(0.35 - 0.022*x));

	if(freq < 7500000) {
		float x = float(freq) * 1e-6;
		float im = -0.8f * x*(0.45f + x*(-0.12f + x*0.008f));
		float re = 1.f;
		refl *= complexf(re, im);
	}
	return refl;
}

static complexf applyFixedCorrectionsThru(complexf thru, freqHz_t freq) {
	float scale = 0.5;
	if(freq > 1900000000) {
		float x = float(freq - 1900000000) / (4400000000 - 1900000000);
		scale *= (1 - 0.8*x*(2 - x));
	}
	return thru * scale;
}


bool serialSendTimeout(const char* s, int len, int timeoutMillis) {
	for(int i = 0; i < timeoutMillis; i++) {
		if(serial.trySend(s, len))
			return true;
		delay(1);
	}
	return false;
}

/*
For a description of the command interface see command_parser.hpp
-- register map:
-- 00: sweepStartHz[7..0]
-- 01: sweepStartHz[15..8]
-- 02: sweepStartHz[23..16]
-- 03: sweepStartHz[31..24]
-- 04: sweepStartHz[39..32]
-- 05: sweepStartHz[47..40]
-- 06: sweepStartHz[55..48]
-- 07: sweepStartHz[63..56]
-- 10: sweepStepHz[7..0]
-- 11: sweepStepHz[15..8]
-- 12: sweepStepHz[23..16]
-- 13: sweepStepHz[31..24]
-- 14: sweepStepHz[39..32]
-- 15: sweepStepHz[47..40]
-- 16: sweepStepHz[55..48]
-- 17: sweepStepHz[63..56]
-- 20: sweepPoints[7..0]
-- 21: sweepPoints[15..8]
-- 22: valuesPerFrequency[7..0]
-- 23: valuesPerFrequency[15..8]
-- 26: dataMode: 0 => VNA data, 1 => raw data, 2 => exit usb data mode
-- 30: valuesFIFO - returns data points; elements are 32-byte. See below for data format.
--                  command 0x14 reads FIFO data; writing any value clears FIFO.
-- 31: RFSW state
	- 0: reference
	- 1: rx port 1
	- 2: rx port 2
-- f0: device variant (01)
-- f1: protocol version (01)
-- f2: hardware revision
-- f3: firmware major version

-- register descriptions:
-- sweepStartHz - Sweep start frequency in Hz.
-- sweepStepHz - Sweep step frequency in Hz.
-- sweepPoints - Number of points in sweep.
-- valuesFIFO - Only command 0x13 supported; returns VNA data.

-- valuesFIFO element data format:
-- bytes:
-- 00: fwd0Re[7..0]
-- 01: fwd0Re[15..8]
-- 02: fwd0Re[23..16]
-- 03: fwd0Re[31..24]
-- 04: fwd0Im[7..0]
-- 05: fwd0Im[15..8]
-- 06: fwd0Im[23..16]
-- 07: fwd0Im[31..24]

-- 08: rev0Re[7..0]
-- 09: rev0Re[15..8]
-- 0a: rev0Re[23..16]
-- 0b: rev0Re[31..24]
-- 0c: rev0Im[7..0]
-- 0d: rev0Im[15..8]
-- 0e: rev0Im[23..16]
-- 0f: rev0Im[31..24]

-- 10: rev1Re[7..0]
-- 11: rev1Re[15..8]
-- 12: rev1Re[23..16]
-- 13: rev1Re[31..24]
-- 14: rev1Im[7..0]
-- 15: rev1Im[15..8]
-- 16: rev1Im[23..16]
-- 17: rev1Im[31..24]

-- 18: freqIndex[7..0]
-- 19: freqIndex[15..8]
-- 1a - 1f: reserved
*/


static void cmdReadFIFO(int address, int nValues) 
{
	if(address == 0x31)
	{
		uint8_t txbuf[2];
		for(int i=0; i<nValues*16;) 
		{
			int rdRPos = ADCValueQueueRPos;
			int rdWPos = ADCValueQueueWPos;
			__sync_synchronize();

			if(rdRPos == rdWPos)  // queue empty
				continue;

			txbuf[0]=uint8_t(ADCValueQueue[rdRPos]  >>0);
			txbuf[1]=uint8_t(ADCValueQueue[rdRPos]  >>8);

			ADCValueQueueRPos = (rdRPos + 1) & ADCValueQueueMask;
			i++;		
			if(!serialSendTimeout((char*)txbuf, sizeof(txbuf), 1500)) // max number of bytes seems to be 0x1f
				return;
		}
		return;
	}

	if(address != 0x30) 
		return;	
	
	for(int i=0; i<nValues;) {
		int rdRPos = usbTxQueueRPos;
		int rdWPos = usbTxQueueWPos;
		__sync_synchronize();

		if(rdRPos == rdWPos) { // queue empty
			continue;
		}

		usbDataPoint& usbDP = usbTxQueue[rdRPos];
		VNAObservation& value = usbDP.value;
		if(usbDP.freqIndex < 0 || usbDP.freqIndex > USB_POINTS_MAX)
			continue;

		value[0] = ecalApplyReflection(value[0] / value[1], usbDP.freqIndex) * value[1];

		int32_t fwdRe = value[1].real();
		int32_t fwdIm = value[1].imag();
		int32_t reflRe = value[0].real();
		int32_t reflIm = value[0].imag();
		int32_t thruRe = value[2].real();
		int32_t thruIm = value[2].imag();

		uint8_t txbuf[32];
		txbuf[0] = uint8_t(fwdRe >> 0);
		txbuf[1] = uint8_t(fwdRe >> 8);
		txbuf[2] = uint8_t(fwdRe >> 16);
		txbuf[3] = uint8_t(fwdRe >> 24);

		txbuf[4] = uint8_t(fwdIm >> 0);
		txbuf[5] = uint8_t(fwdIm >> 8);
		txbuf[6] = uint8_t(fwdIm >> 16);
		txbuf[7] = uint8_t(fwdIm >> 24);

		txbuf[8] = uint8_t(reflRe >> 0);
		txbuf[9] = uint8_t(reflRe >> 8);
		txbuf[10] = uint8_t(reflRe >> 16);
		txbuf[11] = uint8_t(reflRe >> 24);

		txbuf[12] = uint8_t(reflIm >> 0);
		txbuf[13] = uint8_t(reflIm >> 8);
		txbuf[14] = uint8_t(reflIm >> 16);
		txbuf[15] = uint8_t(reflIm >> 24);

		txbuf[16] = uint8_t(thruRe >> 0);
		txbuf[17] = uint8_t(thruRe >> 8);
		txbuf[18] = uint8_t(thruRe >> 16);
		txbuf[19] = uint8_t(thruRe >> 24);

		txbuf[20] = uint8_t(thruIm >> 0);
		txbuf[21] = uint8_t(thruIm >> 8);
		txbuf[22] = uint8_t(thruIm >> 16);
		txbuf[23] = uint8_t(thruIm >> 24);

		txbuf[24] = uint8_t(usbDP.freqIndex >> 0);
		txbuf[25] = uint8_t(usbDP.freqIndex >> 8);

		txbuf[26] = 0;
		txbuf[27] = 0;
		txbuf[28] = 0;
		txbuf[29] = 0;
		txbuf[30] = 0;
		txbuf[31] = 0;

		uint8_t checksum=0b01000110;
		for(int i=0; i<31; i++)
			checksum = (checksum xor ((checksum<<1) | 1)) xor txbuf[i];
		txbuf[31] = checksum;

		if(!serialSendTimeout((char*)txbuf, sizeof(txbuf), 1500)) {
			return;
		}

		__sync_synchronize();
		usbTxQueueRPos = (rdRPos + 1) & usbTxQueueMask;
		i++;
	}
}

// apply usb-configured sweep parameters
static void setVNASweepToUSB() {
	int points = *(uint16_t*)(registers + 0x20);
	int values = *(uint16_t*)(registers + 0x22);

	if(points > USB_POINTS_MAX)
		points = USB_POINTS_MAX;

	vnaMeasurement.sweepStartHz = (freqHz_t)*(uint64_t*)(registers + 0x00);
	vnaMeasurement.sweepStepHz = (freqHz_t)*(uint64_t*)(registers + 0x10);
	vnaMeasurement.sweepDataPointsPerFreq = values;
	vnaMeasurement.sweepPoints = points;
	vnaMeasurement.resetSweep();
}

static void measurementPhaseChanged(VNAMeasurementPhases ph);

static void cmdRegisterWrite(int address) {
	if(address == 0x00 || address == 0x10 || address == 0x20 || address == 0x22) 
		setVNASweepToUSB();
	if(address == 0x00)
	{
		//freqHz_t f = (freqHz_t)*(uint64_t*)(registers + 0x00);
		setFrequency((freqHz_t)*(uint64_t*)(registers + 0x00));
	}
	if(address == 0x26) {
		auto val = registers[0x26];
		if(val == 0) {
			outputRawSamples = false;
		} else if(val == 1) {
			outputRawSamples = true;
		} else if(val == 2) {
			outputRawSamples = false;
		}
	}
	if(address == 0x00 || address == 0x10 || address == 0x20) {
		ecalState = ECAL_STATE_MEASURING;
		vnaMeasurement.ecalIntervalPoints = 1;
		vnaMeasurement.nPeriods = MEASUREMENT_NPERIODS_CALIBRATING;
	}
	if(address == 0x30) {
		ADCValueQueueRPos = ADCValueQueueWPos; // clear ADCValueQueue
		usbTxQueueRPos = usbTxQueueWPos; // clear usbTxQueue
	}
	if(address == 0x31) 
	{
		auto val = registers[0x31];
		if(val == 0)
			measurementPhaseChanged(VNAMeasurementPhases::REFERENCE);
		else if(val == 1)
			measurementPhaseChanged(VNAMeasurementPhases::REFL);
		else if(val == 2)
			measurementPhaseChanged(VNAMeasurementPhases::THRU);
		else if(val == 3)
			measurementPhaseChanged(VNAMeasurementPhases::ECALTHRU);
		else if(val == 4)
			measurementPhaseChanged(VNAMeasurementPhases::ECALLOAD);
		else if(val == 5)
			measurementPhaseChanged(VNAMeasurementPhases::ECALSHORT);
	}
	if(address == 0x32)
	{
		auto val = registers[0x32];
		if(val >= 0 &&  val <= 3)
			rfsw(RFSW_BBGAIN, RFSW_BBGAIN_GAIN(val));
	}
}


static void cmdInit() {
	cmdParser.handleReadFIFO = [](int address, int nValues) {
		return cmdReadFIFO(address, nValues);
	};
	cmdParser.handleWriteFIFO = [](int address, int totalBytes, int nBytes, const uint8_t* data) {};
	cmdParser.handleWrite = [](int address) {
		return cmdRegisterWrite(address);
	};
	cmdParser.send = [](const uint8_t* s, int len) {
		serialSendTimeout((char*) s, len, 1500);
	};
	cmdParser.registers = registers;
	cmdParser.registersSizeMask = registersSizeMask;

	cmdInputFIFO.buffer = cmdInputBuffer;
	cmdInputFIFO.bufferSize = sizeof(cmdInputBuffer);
	cmdInputFIFO.output = [](const uint8_t* s, int len) {
		cmdParser.handleInput(s, len);
	};
}

static int measurementGetDefaultGain(freqHz_t freqHz) {
	if(freqHz > 2500000000)
		return 2;
	else if(freqHz > FREQUENCY_CHANGE_OVER)
		return 1;
	else
		return 0;
}
// callback called by VNAMeasurement to change rf switch positions.
static void measurementPhaseChanged(VNAMeasurementPhases ph) {
	if(!outputRawSamples)
		rfsw(RFSW_BBGAIN, RFSW_BBGAIN_GAIN(measurementGetDefaultGain(currFreqHz)));
	switch(ph) {
		case VNAMeasurementPhases::REFERENCE:
			rfsw(RFSW_REFL, RFSW_REFL_ON);
			rfsw(RFSW_RECV, RFSW_RECV_REFL);
			rfsw(RFSW_ECAL, RFSW_ECAL_OPEN);
			break;
		case VNAMeasurementPhases::REFL:
			rfsw(RFSW_ECAL, RFSW_ECAL_NORMAL);
			break;
		case VNAMeasurementPhases::THRU:
			rfsw(RFSW_ECAL, RFSW_ECAL_NORMAL);
			rfsw(RFSW_REFL, RFSW_REFL_OFF);
			rfsw(RFSW_RECV, RFSW_RECV_PORT2);
			break;
		case VNAMeasurementPhases::ECALTHRU:
			rfsw(RFSW_ECAL, RFSW_ECAL_LOAD);
			rfsw(RFSW_RECV, RFSW_RECV_REFL);
			break;
		case VNAMeasurementPhases::ECALLOAD:
			rfsw(RFSW_REFL, RFSW_REFL_ON);
			rfsw(RFSW_RECV, RFSW_RECV_REFL);
			rfsw(RFSW_ECAL, RFSW_ECAL_LOAD);
			break;
		case VNAMeasurementPhases::ECALSHORT:
			rfsw(RFSW_ECAL, RFSW_ECAL_SHORT);
			break;
	}
}

// callback called by VNAMeasurement when an observation is available.
static void measurementEmitDataPoint(int freqIndex, freqHz_t freqHz, VNAObservation v, const complexf* ecal) {
	digitalWrite(led, vnaMeasurement.clipFlag?1:0);

	v[2] *= gainTable[currThruGain] / gainTable[measurementGetDefaultGain(freqHz)];
	v[2] = applyFixedCorrectionsThru(v[2], freqHz);
	v[0] = applyFixedCorrections(v[0]/v[1], freqHz) * v[1];

	
	// enqueue new data point
	int wrRPos = usbTxQueueRPos;
	int wrWPos = usbTxQueueWPos;
	__sync_synchronize();
	if(((wrWPos + 1) & usbTxQueueMask) == wrRPos) {
		// overflow
		// discard value
	} 
	else {
		usbTxQueue[wrWPos].freqIndex = freqIndex;
		usbTxQueue[wrWPos].value = v;
		__sync_synchronize();
		usbTxQueueWPos = (wrWPos + 1) & usbTxQueueMask;
	}
}


static void measurement_setup() {
	vnaMeasurement.phaseChanged = [](VNAMeasurementPhases ph) {
		measurementPhaseChanged(ph);
	};
	vnaMeasurement.gainChanged = [](int gain) {
		currThruGain = gain;
		rfsw(RFSW_BBGAIN, RFSW_BBGAIN_GAIN(currThruGain));
	};
	vnaMeasurement.emitDataPoint = [](int freqIndex, freqHz_t freqHz, const VNAObservation& v, const complexf* ecal) {
		measurementEmitDataPoint(freqIndex, freqHz, v, ecal);
	};
	vnaMeasurement.frequencyChanged = [](freqHz_t freqHz) {
		setFrequency(freqHz);
	};
	vnaMeasurement.sweepSetupChanged = [](freqHz_t start, freqHz_t stop) {
		if(!is_freq_for_adf4350(stop)) {
			/* ADF4350 can be powered down */
			adf4350_powerdown();
		}
		else {
			adf4350_powerup();
		}
		if(is_freq_for_adf4350(start)) {
			/* Si5351 not needed, power it down? */
		}
	};
	vnaMeasurement.nPeriods = MEASUREMENT_NPERIODS_NORMAL;
	vnaMeasurement.gainMin = 0;
	vnaMeasurement.gainMax = RFSW_BBGAIN_MAX;
	vnaMeasurement.init();
}

static void adc_process() {
	volatile uint16_t* buf;
	int len;
	for(int i=0; i<2; i++) {
		adc_read(buf, len);
		for(int k=0;k<len;k++)
		{
			int wrWPos = ADCValueQueueWPos;
			int wrRPos = ADCValueQueueRPos;
			__sync_synchronize();			
			if(((wrWPos + 1) & ADCValueQueueMask) == wrRPos) 
			{
				// overflow
				// discard value
				break;
			} 			
			else
			{
				ADCValueQueue[ADCValueQueueWPos] = buf[k];
				__sync_synchronize();				
				ADCValueQueueWPos = (ADCValueQueueWPos + 1) & ADCValueQueueMask;				
			}
		}
		if(!outputRawSamples)
			vnaMeasurement.processSamples((uint16_t*)buf, len);
	}
}

/* Return true when FPU is available */
bool cpu_enable_fpu(void)
{
	uint32_t fpuEnable = 0b1111 << 20;
	if((SCB_CPACR & fpuEnable) != fpuEnable) {
		SCB_CPACR |= fpuEnable;
		if((SCB_CPACR & fpuEnable) != fpuEnable) {
			return false;
		}
	}
	return true;
}

int main(void) {
	bool shouldShowDmesg = false;

#ifndef GD32F3_NOFPU
	if(cpu_enable_fpu()) {
		printk1("LIBOPENCM3 DID NOT ENABLE FPU!\n CHECK lib/dispatch/vector_chipset.c\n");
	} else {
		// printk1() does not invoke printf() and does not use fpu

		// if you encounter this error, see:
		// https://www.amobbs.com/thread-5719892-1-1.html
		printk1("FPU NOT DETECTED!\nCHECK GD32F303 BATCH OR REBUILD WITHOUT FPU\n");
		shouldShowDmesg = true;
	}
#endif


	boardInit();

	// set version registers (accessed through usb serial)
	registers[0xf0 & registersSizeMask] = 2;	// device variant
	registers[0xf1 & registersSizeMask] = 1;	// protocol version
	registers[0xf2 & registersSizeMask] = (uint8_t) BOARD_REVISION;
	registers[0xf3 & registersSizeMask] = (uint8_t) FIRMWARE_MAJOR_VERSION;
	registers[0xf4 & registersSizeMask] = (uint8_t) FIRMWARE_MINOR_VERSION;

	// we want all higher priority irqs to preempt lower priority ones
	scb_set_priority_grouping(SCB_AIRCR_PRIGROUP_GROUP16_NOSUB);

	pinMode(led, OUTPUT);
	pinMode(led2, OUTPUT);
	pinMode(RFSW_ECAL, OUTPUT);
	pinMode(RFSW_BBGAIN, OUTPUT);
	pinMode(RFSW_TXSYNTH, OUTPUT);
	pinMode(RFSW_RXSYNTH, OUTPUT);
	pinMode(RFSW_REFL, OUTPUT);
	pinMode(RFSW_RECV, OUTPUT);
	pinMode(USB0_DP, OUTPUT);

	digitalWrite(USB0_DP, LOW);

	digitalWrite(led, HIGH);

	rfsw(RFSW_BBGAIN, RFSW_BBGAIN_GAIN(0));
	rfsw(RFSW_RXSYNTH, RFSW_RXSYNTH_LF);
	rfsw(RFSW_TXSYNTH, RFSW_TXSYNTH_LF);
	rfsw(RFSW_REFL, RFSW_REFL_ON);
	rfsw(RFSW_RECV, RFSW_RECV_REFL);
	rfsw(RFSW_ECAL, RFSW_ECAL_NORMAL);

	delay(500);

	cmdInit();
	serial.setReceiveCallback([](uint8_t* s, int len) {
		cmdInputFIFO.input(s, len);
	});
	// baud rate is ignored for usbserial
	serial.begin(115200);
	pinMode(USB0_DP, INPUT);

	nvic_set_priority(NVIC_USB_HP_CAN_TX_IRQ, 0xf0);

	// set up lcd and hook up UI events
	lcd_and_ui_setup();

	delay(50);
	
	flash_config_recall();


	// show dmesg and wait for user input if there is an important error
	if(shouldShowDmesg) {
		printk1("Touch anywhere to continue...\n");
		//show_dmesg();
	}

	printk("xtal freq %d.%03d MHz\n", (xtalFreqHz/1000000), ((xtalFreqHz/1000) % 1000));

	si5351_i2c.init();
	if(!synthesizers::si5351_setup()) {
		printk1("ERROR: si5351 init failed\n");
		printk1("Touch anywhere to continue...\n");
		//show_dmesg();
	}


	setFrequency(56000000);

	
	// initialize VNAMeasurement
	measurement_setup();
	adc_setup();
	dsp_timer_setup();

	adf4350_setup();

	performGainCal(vnaMeasurement, gainTable, RFSW_BBGAIN_MAX);

	for(int i=0; i<=RFSW_BBGAIN_MAX; i++) {
		printk("BBGAIN %d: %.2f dB\n", i, log10f(gainTable[i])*20.f);
	}

	setVNASweepToUSB();

	while(true) {
		// process any outstanding commands from usb
		cmdInputFIFO.drain();
		
		//if(outputRawSamples)
//			usb_transmit_rawSamples();
	}
}

extern "C" void abort() {
	while (1) {
		for(int i=0;i<3;i++) {
			digitalWrite(led, HIGH);
			delay(100);
			digitalWrite(led, LOW);
			delay(100);
		}
		delay(1000);
	}
}

extern "C" {
	__attribute__((used))
	uintptr_t __stack_chk_guard = 0xdeadbeef;

	__attribute__((used))
	void __cxa_pure_virtual() {
		errorBlink(4);
		while(1);
	}
	__attribute__((used))
	void __stack_chk_fail() {
		errorBlink(5);
		while(1);
	}
	__attribute__((used))
	void _fini() {
		errorBlink(6);
		while(1);
	}
	__attribute__((used))
	void __assert_fail(const char *__assertion, const char *__file,
               unsigned int __line, const char *__function) {
		errorBlink(3);
		while(1);
	}
}


