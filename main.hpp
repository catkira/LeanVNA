#pragma once

#include <stdint.h>
#include <mculib/small_function.hpp>
#include "common.hpp"

// DO NOT INCLUDE THIS FILE OTHER THAN FROM main2.cpp and ui.cpp!!!!!
// ONLY ui.cpp IS ALLOWED TO CALL FUNCTIONS DEFINED IN main2.cpp!!!!!

// The following are the application callback functions that the
// UI code will call to perform actions.
extern bool cpu_enable_fpu(void);


