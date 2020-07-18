#pragma once


template<class emitValue_t>
class RawSampleProcessor {
private:
	uint16_t* sampleBuffer;
	uint32_t accumPhase;	

public:
	// integration time for each output value
	int accumPeriod = 50;

	emitValue_t emitValue;

	RawSampleProcessor(const emitValue_t& cb, uint16_t* sampleBuffer): emitValue(cb), sampleBuffer(sampleBuffer) {}
	void init() {
		accumPhase = 0;
	}
	
	bool process(uint16_t* samples, int len) {
		uint16_t* end = samples+len;
		while(samples < end) {

			sampleBuffer[accumPhase]=*samples;
		
			// save sample in buffer
			accumPhase++;
			samples ++;
			if(int(accumPhase) >= accumPeriod) {
				emitValue(sampleBuffer, accumPhase);
				accumPhase = 0;
			}
		}
		return true;
	}
};


template<class emitValue_t>
class SampleProcessor {
public:
	uint32_t accumPhase;
	int32_t accumRe,accumIm;

	// integration time for each output value
	int accumPeriod = 50;

	// sinusoid table, must be accumPeriod*2 entries (interleaved real & imag)
	const int16_t* correlationTable = nullptr;

	bool clipFlag = false;

	// void emitValue(int32_t valRe, int32_t valIm)
	emitValue_t emitValue;

	SampleProcessor(const emitValue_t& cb): emitValue(cb) {}
	void init() {
		accumPhase = 0;
		accumRe = accumIm = 0;
		clipFlag = false;
	}
	void setCorrelationTable(const int16_t* table, int length) {
		accumPhase = 0;
		correlationTable = table;
		accumPeriod = length;
	}
	// len specifies the number of aggregates (i.e. when nStreams > 1,
	// the number of words in the array must be len * nStreams).
	// returns whether we completed a cycle.
	bool process(uint16_t* samples, int len) {
		uint16_t* end = samples+len;
		bool ret = false;
		while(samples < end) {
			int32_t lo_im = correlationTable[accumPhase*2];
			int32_t lo_re = correlationTable[accumPhase*2 + 1];

			int16_t sample = int16_t((*samples)*16 - 32768);
			if(sample > 30000 || sample < -30000) 
				clipFlag = true;
		
			accumRe += lo_re*sample/256;
			accumIm += lo_im*sample/256;
			
			accumPhase++;
			samples ++;
			if(int(accumPhase) >= accumPeriod) {
				emitValue(accumRe, accumIm);
				accumRe = accumIm = 0;
				accumPhase = 0;
				ret = true;
			}
		}
		return ret;
	}
};

