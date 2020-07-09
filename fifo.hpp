#pragma once
#include <stdint.h>

// size must be a power of 2.
// actual capacity is size - 1 elements.
// thread safe in the case of one reader and multiple writers.
template<class T, int size>
class FIFO {
private:
	volatile uint32_t _rpos = 0, _wpos = 0;

	static constexpr uint32_t sizeMask = size - 1;

	T elements[size];

public:
	
	FIFO() {}


	bool writable() const {
		return ((_wpos+1) & sizeMask) != _rpos; 
	}
	
	bool readable() const {
		if((_wpos & sizeMask) == _rpos)
			return false;
		return true;
	}

	T dequeue() {
		if(!readable())
			abort();
		auto ret = elements[_rpos];
		_rpos = (_rpos+1) & sizeMask;
		return ret;
	}

	void clear() {
		while(readable()) 
			dequeue();
	}

	bool enqueue(const T& value) {
		if(!writable())
			return false;
		elements[_wpos]=value;
		_wpos = (_wpos+1) & sizeMask;		
		return true;
	}
};

