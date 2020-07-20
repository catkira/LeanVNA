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
		return !(_wpos == _rpos);
	}

	T dequeue() {
		if(!readable())
			abort();
		__sync_synchronize();		
		T ret = elements[_rpos];
		_rpos = (_rpos+1) & sizeMask;
		return ret;
	}

	void clear() {
		_rpos = _wpos = 0;
	}

	bool enqueue(const T& value) {
		if(!writable())
			abort();
		__sync_synchronize();
		elements[_wpos]=value;
		_wpos = (_wpos+1) & sizeMask;		
		return true;
	}
};

