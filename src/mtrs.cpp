#include "mtrs.hpp"
#include "okapi/api.hpp"
std::unique_ptr<Motors> mtrs;

	CubeLift::CubeLift(okapi::AbstractMotor& icaptive, pros::ADIPotentiometer& icaptiveEnc):
	captive(&icaptive), captiveEnc(&icaptiveEnc), ctrl(0.008, 0.00002, 0.0003, 0, okapi::TimeUtilFactory::createDefault()) {}