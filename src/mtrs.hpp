#pragma once
#include "main.h"
#include "okapi/api/device/motor/abstractMotor.hpp"
#include "okapi/impl/device/motor/motorGroup.hpp"
#include "okapi/api/control/iterative/iterativePosPidController.hpp"
#include "sensors.hpp"
#include <stdint.h>
#include <vector>

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

/*
class EncoderAverage: public okapi::ContinuousRotarySensor {
	std::vector<std::shared_ptr<okapi::ContinuousRotarySensor>> mySensors;
	public:
	EncoderAverage(std::initializer_list<std::shared_ptr<okapi::ContinuousRotarySensor>> sensors):
	mySensors(std::move(sensors)) {}
	double get() const override {
		auto sum = 0.0;
		for(auto& enc: mySensors) {
			sum += enc->get();
		}
		return sum / mySensors.size();
	}
	double controllerGet() override {
		return get();
	}
	std::int32_t reset() override {
		std::int32_t ret = 0;
		for(auto& enc: mySensors) ret = enc->reset();
		return ret;
	}
};

class ReversedEncoder: public okapi::ContinuousRotarySensor {
	std::shared_ptr<okapi::ContinuousRotarySensor> encoder;
	public:
	ReversedEncoder(std::shared_ptr<okapi::ContinuousRotarySensor> sensor):
	encoder(std::move(sensor)) {}
	double get() const override { return -encoder->get(); }
	double controllerGet() override { return get(); }
	std::int32_t reset() override { return encoder->reset(); }
};

class ExtraSpecialMotorWithExternalSensorsAsEncoders: public okapi::AbstractMotor {
	okapi::AbstractMotor& captive;
	std::shared_ptr<okapi::ContinuousRotarySensor> realEnc;
	double lastTarget = 0;
	double offset;
	double units = 1;
	public:
	ExtraSpecialMotorWithExternalSensorsAsEncoders(okapi::AbstractMotor& icaptive,
	std::shared_ptr<okapi::ContinuousRotarySensor> ienc):
	captive(icaptive), realEnc(ienc) {
		icaptive.setEncoderUnits(okapi::AbstractMotor::encoderUnits::rotations);
		icaptive.setGearing(okapi::AbstractMotor::gearset::green);
		units = 1.0 / 360.0;
	}

	//V5's built-in profiled movement still has to use integrated encoder.
	std::int32_t moveAbsolute(double position, std::int32_t velocity) override {
		lastTarget = position;
		auto integratedPosition = (position - getPosition()) + captive.getPosition();
		return captive.moveAbsolute(integratedPosition, velocity);
	}

	std::int32_t moveRelative(double position, std::int32_t velocity) override {
		lastTarget = position + getPosition();
		return captive.moveRelative(position, velocity);
	}

	std::int32_t moveVelocity(std::int16_t velocity) override {
		return captive.moveVelocity(velocity);
	}

	std::int32_t moveVoltage(std::int16_t voltage) override {
		return captive.moveVoltage(voltage);
	}

	double getTargetPosition() override {
		return lastTarget;
	}

	double getPosition() override {
		return (realEnc->get() + offset) * units;
	}

	std::int32_t tarePosition() override {
		auto oldOffset = offset;
		offset = -realEnc->get();
		lastTarget += (offset - oldOffset) * units;
		captive.tarePosition();
		return 1;
	}

	std::int32_t getRawPosition(std::uint32_t* timestamp) override {
		captive.getRawPosition(timestamp);
		return getPosition() / units;
	}

	std::int32_t getTargetVelocity() override { return captive.getTargetVelocity(); }
	double getActualVelocity() override { return captive.getActualVelocity(); }
	std::int32_t getCurrentDraw() override { return captive.getCurrentDraw();	}
	std::int32_t getDirection() override { return captive.getDirection(); }
	double getEfficiency() override { return captive.getEfficiency(); }
	std::int32_t isOverCurrent() override { return captive.isOverCurrent(); }
	std::int32_t isOverTemp() override { return captive.isOverTemp(); }
	std::int32_t isStopped() override { return captive.isStopped(); }
	std::int32_t getZeroPositionFlag() override { return captive.getZeroPositionFlag(); }
	uint32_t getFaults() override { return captive.getFaults(); }
	uint32_t getFlags() override { return captive.getFlags(); }
	double getPower() override { return captive.getPower(); }
	double getTemperature() override { return captive.getTemperature(); }
	double getTorque() override { return captive.getTorque(); }
	std::int32_t getVoltage() override { return captive.getVoltage(); }
	std::int32_t setBrakeMode(okapi::AbstractMotor::brakeMode b) override { return captive.setBrakeMode(b); }
	okapi::AbstractMotor::brakeMode getBrakeMode() override { return captive.getBrakeMode(); }
	std::int32_t setCurrentLimit(std::int32_t limit) override { return captive.setCurrentLimit(limit); }
	std::int32_t getCurrentLimit() override { return captive.getCurrentLimit(); }
	std::int32_t modifyProfiledVelocity(std::int32_t vel) override {
		return captive.modifyProfiledVelocity(vel);
	}
	double encGearing() {
		auto it = (int)getGearing();
		if(it == 100) return 1800;
		if(it == 200) return 900;
		if(it == 600) return 300;
		return 0;
	}
	std::int32_t setEncoderUnits(okapi::AbstractMotor::encoderUnits eunits) override {
		if(eunits == okapi::AbstractMotor::encoderUnits::counts) {
			units = sgn(units) * encGearing() / 360.0;
		} else if(eunits == okapi::AbstractMotor::encoderUnits::degrees) {
			units = sgn(units) * 1.0;
		} else if(eunits == okapi::AbstractMotor::encoderUnits::rotations) {
			units = sgn(units) * 1.0 / 360.0;
		}
		return captive.setEncoderUnits(eunits);
	}
	okapi::AbstractMotor::encoderUnits getEncoderUnits() override {
		if(std::abs(units) == 1) {
			return okapi::AbstractMotor::encoderUnits::degrees;
		} else if(std::abs(units) < 1) {
			return okapi::AbstractMotor::encoderUnits::rotations;
		} else {
			return okapi::AbstractMotor::encoderUnits::counts;
		}
	};
	std::int32_t setGearing(okapi::AbstractMotor::gearset gearing) {
		auto ret = captive.setGearing(gearing);
		setEncoderUnits(getEncoderUnits());
		return ret;
	}
	okapi::AbstractMotor::gearset getGearing() {
		return captive.getGearing();
	}
	void setEncReversed(bool isReversed) {
		units = (isReversed ? -1 : 1) * std::abs(units);
	}
	std::int32_t setReversed(bool isReversed) override {
		return captive.setReversed(isReversed);
	}

	std::int32_t setVoltageLimit(std::int32_t ilimit) override {
		return captive.setVoltageLimit(ilimit);
	}

	std::shared_ptr<okapi::ContinuousRotarySensor> getEncoder() override {
		return realEnc;
	}

	void controllerSet(double vel) {
		moveVelocity(vel * encGearing());
	}
};
*/

class CubeLift {
	okapi::AbstractMotor* captive;
	pros::ADIPotentiometer* captiveEnc;
	pros::Mutex pidLock;
	bool pidActive = false;
	std::unique_ptr<pros::Task> pidBackgroundTask;
	const double bottomTargetTicks = 2807;
	const double lowTargetTicks = 1615;
	const double highTargetTicks = 1308;
	double currentTarget = bottomTargetTicks;
	okapi::IterativePosPIDController ctrl;
	void deactivatePID() {
		pidLock.take(TIMEOUT_MAX);
		pidActive = false;
		pidLock.give();
	}
	void activatePID() {
		pidLock.take(TIMEOUT_MAX);
		pidActive = true;
		pidLock.give();
		if(pidBackgroundTask) pidBackgroundTask->notify();
		printf("PID was activated\n");
	}
	void runPID() {
		uint32_t lastTime = pros::millis();
		while(true) {
			pidLock.take(TIMEOUT_MAX);
			ctrl.flipDisable(!pidActive);
			if(pidActive) {
				captive->controllerSet(ctrl.step(captiveEnc->get_value()));
				pidLock.give();
				pros::c::task_delay_until(&lastTime, 10);
			} else {
				pidLock.give();
				pros::Task::current().notify_take(false, TIMEOUT_MAX);
				lastTime = pros::millis();
				ctrl.reset();
			}
		}
	}
	static void invokePID(void* me) {
		((CubeLift*)me)->runPID();
	}
	public:
	bool isPIDActive() {
		return pidActive;
	}
	CubeLift(okapi::AbstractMotor& icaptive, pros::ADIPotentiometer& icaptiveEnc);
	void startThread() {
		pidBackgroundTask = std::make_unique<pros::Task>(invokePID, (void*)this, "CubeLift PID");
	}
	void lowTarget() {
		currentTarget = lowTargetTicks;
		ctrl.setTarget(currentTarget);
		activatePID();
	}
	void highTarget() {
		currentTarget = highTargetTicks;
		ctrl.setTarget(currentTarget);
		activatePID();
	}
	void bottomTarget() {
		currentTarget = bottomTargetTicks;
		ctrl.setTarget(currentTarget);
		activatePID();
	}
	void toggle() {
		if(currentTarget == lowTargetTicks) {
			highTarget();
			return;
		}
		if(currentTarget == highTargetTicks) {
			bottomTarget();
			return;
		}
		if(currentTarget == bottomTargetTicks) {
			lowTarget();
			return;
		}
	}
	void controllerSet(double vel) {
		deactivatePID();
		captive->controllerSet(vel);
	}
	void moveVoltage(int16_t volts) {
		deactivatePID();
		captive->moveVoltage(volts);
	}
	double getPosition() {
		return captive->getPosition();
	}
};

struct Motors {
	okapi::MotorGroup left  {  1,   3};
	okapi::MotorGroup right {- 2, - 5}; //1
	okapi::MotorGroup rightRev {2, 5};
	okapi::MotorGroup all   {  1,   3, - 2, - 5};
	okapi::MotorGroup turn  {  1,   3,   2,   5};
	okapi::MotorGroup intake   {- 7, 20};
	okapi::MotorGroup tilter   {- 6}; // -1.2
	okapi::MotorGroup liftRaw  {- 4}; // 2.976
	CubeLift          lift     { liftRaw, *potPtr };
	Motors() {
		liftRaw.setEncoderUnits(okapi::AbstractMotor::encoderUnits::rotations);
		tilter.setGearing(okapi::AbstractMotor::gearset::red);
		liftRaw.setGearing(okapi::AbstractMotor::gearset::green);
		liftRaw.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
		all.setEncoderUnits(okapi::AbstractMotor::encoderUnits::rotations);
		left.setEncoderUnits(okapi::AbstractMotor::encoderUnits::rotations);
		right.setEncoderUnits(okapi::AbstractMotor::encoderUnits::rotations);
		rightRev.setEncoderUnits(okapi::AbstractMotor::encoderUnits::rotations);
		tilter.setEncoderUnits(okapi::AbstractMotor::encoderUnits::rotations);
		intake.setGearing(okapi::AbstractMotor::gearset::red);
		intake.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
		tilter.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
		all.tarePosition();
		tilter.tarePosition();
		liftRaw.tarePosition();
		intake.tarePosition();
		lift.startThread();
	}
};

extern std::unique_ptr<Motors> mtrs;