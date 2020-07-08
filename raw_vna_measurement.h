#pragma once
#include <mculib/small_function.hpp>
#include "common.hpp"
#include "sample_processor.hpp"



// implements sweep, rf switch timing, and dsp for single-receiver
// switched path VNAs (one receiver with switches to select reference,
// reflected, and thru paths).
// given switch & synthesizer controls and adc data feed, emit a stream
// of data points.
class RawVNAMeasurement {
public:
	
	// how many periods to wait after changing rf switches
	static constexpr uint32_t nWaitSwitch = 5;

	uint32_t samplesPerPhase = 512;


	// called when a new data point is available.
	small_function<void()> emitData;

	// called to change rf switch direction;
	// the function may assume the phase progression is always:
	// REFERENCE, REFL1, REFL2, THRU,
	// except that REFERENCE may be switched to at any time and from any phase.
	small_function<void(VNAMeasurementPhases ph)> phaseChanged;

	RawVNAMeasurement();

	void init();
	void sampleProcessor_emitValue(uint16_t* buf, uint32_t len);
	
	struct _emitValue_t {
		RawVNAMeasurement* m;
		void operator()(uint16_t* buf, uint32_t len);
	};

	RawSampleProcessor<_emitValue_t> sampleProcessor;	
	void processSamples(uint16_t* buffer, uint32_t len);

public:
	// state variables
	VNAMeasurementPhases measurementPhase = VNAMeasurementPhases::IDLE;

	// number of periods since changing rf switches
	uint32_t periodCounterSwitch = 0;
	
	void setMeasurementPhase(VNAMeasurementPhases ph);	
	void collectData(uint32_t samplesPerPhase);
private:
	bool startRequest = false;
};
