#pragma once
#include <mculib/small_function.hpp>
#include "common.hpp"
#include "sample_processor.hpp"



// implements sweep, rf switch timing, and dsp for single-receiver
// switched path VNAs (one receiver with switches to select reference,
// reflected, and thru paths).
// given switch & synthesizer controls and adc data feed, emit a stream
// of data points.
template<typename FIFO>
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
	FIFO* ADCValueQueue;	

	// state variables
	VNAMeasurementPhases measurementPhase = VNAMeasurementPhases::IDLE;

	// number of periods since changing rf switches
	uint32_t periodCounterSwitch = 0;
	
	void setMeasurementPhase(VNAMeasurementPhases ph);	
	void collectData(uint32_t samplesPerPhase);
private:
	volatile bool startRequest = false;
	uint16_t sampleBuffer[2048];	
};

template <typename FIFO>
RawVNAMeasurement<FIFO>::RawVNAMeasurement(): sampleProcessor(_emitValue_t {this},sampleBuffer)
{
}

template <typename FIFO>
void RawVNAMeasurement<FIFO>::init() 
{
}

template <typename FIFO>
void RawVNAMeasurement<FIFO>::collectData(uint32_t samplesPerPhase)
{
	this->samplesPerPhase = samplesPerPhase;
	sampleProcessor.accumPeriod = samplesPerPhase;
	__sync_synchronize();
	startRequest = true;
}

template <typename FIFO>
void RawVNAMeasurement<FIFO>::setMeasurementPhase(VNAMeasurementPhases ph) 
{
	phaseChanged(ph);
	measurementPhase = ph;
	periodCounterSwitch = 0;
}

template <typename FIFO>
void RawVNAMeasurement<FIFO>::processSamples(uint16_t* buf, uint32_t len) 
{
	sampleProcessor.process(buf, len);
}

template <typename FIFO>
void RawVNAMeasurement<FIFO>::sampleProcessor_emitValue(uint16_t* buf, uint32_t len) 
{
	if(startRequest)
	{
		startRequest = false;
		ADCValueQueue->clear();
		setMeasurementPhase(VNAMeasurementPhases::REFERENCE);
		return;
	}
	if(measurementPhase == VNAMeasurementPhases::IDLE)
		return;
	
	// throw away nWaitSwitch*samplesPerPhase values after a RF-switch change	
	if(periodCounterSwitch < nWaitSwitch)
	{
		periodCounterSwitch++;
		return;
	}
	
	// need to make sure ADCValueQueue is larger than 3*samplesPerPhase
	for(uint32_t i = 0;i<len;i++)
	{
		ADCValueQueue->enqueue(buf[i]);
	}

	if(measurementPhase == VNAMeasurementPhases::REFERENCE)
		setMeasurementPhase(VNAMeasurementPhases::REFL);
	else if(measurementPhase == VNAMeasurementPhases::REFL)
		setMeasurementPhase(VNAMeasurementPhases::THRU);		
	else if(measurementPhase == VNAMeasurementPhases::THRU)
	{
		emitData();
		measurementPhase = VNAMeasurementPhases::IDLE;
	}
}

template <typename FIFO>
void RawVNAMeasurement<FIFO>::_emitValue_t::operator()(uint16_t* buf, uint32_t len) {
	m->sampleProcessor_emitValue(buf, len);
}