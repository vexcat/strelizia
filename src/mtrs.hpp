#include "main.h"
#include "okapi/api.hpp"

struct Motors {
	okapi::MotorGroup left    {-11,  12};
	okapi::MotorGroup right   { 13, -14};
	okapi::MotorGroup all     {-11,  12,  13, -14};
	okapi::MotorGroup intake  { 15};
	okapi::MotorGroup mgl     { 16};
	okapi::MotorGroup lift    { 17};
	okapi::MotorGroup claw    { 18};
};

extern Motors mtrs;