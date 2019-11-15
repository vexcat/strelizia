#include "main.h"
#include "okapi/api.hpp"
#include "blue_controller.hpp"
#include "mtrs.hpp"
#include "sensors.hpp"

double powered(int ctrl_power, double exp) {
	if(ctrl_power == 0) return 0;
	if(ctrl_power < 0) return -powered(-ctrl_power, exp);
	if(ctrl_power > 127) ctrl_power =  127;
	double ctrl = ctrl_power / 127.0;
	return std::pow(ctrl, exp);
}

volatile bool opcontrolActive = true;
volatile bool opcontrolActiveAck = true;
void pauseControl() {
	opcontrolActive = false;
	while(opcontrolActiveAck) pros::delay(1);
}

void resumeControl() {
	opcontrolActive = true;
	while(!opcontrolActiveAck) pros::delay(1);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

//split arcade base
//R1 - Toggle claw
//R2 - Toggle intake
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	mtrs->intake.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	uint32_t liftActivatedAt = pros::millis();
	mtrs->lift.setEncoderUnits(okapi::AbstractMotor::encoderUnits::rotations);
	mtrs->tilter.setEncoderUnits(okapi::AbstractMotor::encoderUnits::rotations);
	double initialTilterPos = mtrs->tilter.getPosition();
	double initialLiftPos = mtrs->lift.getPosition();
	mtrs->lift.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	mtrs->tilter.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	int automaticTiltActivationTime = -1;
	bool tilterInUse = false;
	bool liftInUse = false;
	while (true) {
		if(!opcontrolActive) {
			opcontrolActiveAck = false;
			pros::delay(10);
			continue;
		}
		opcontrolActiveAck = true;
		double y_ctrl  = master.get_analog(ANALOG_LEFT_Y) / 127.0;
		double x_ctrl = master.get_analog(ANALOG_LEFT_X) / 127.0;
		double liftControl = -master.get_analog(ANALOG_RIGHT_Y) / 127.0;
		if(!liftInUse || liftInUse) {
			mtrs->lift.controllerSet(liftControl);
			liftInUse = false;
		}
		
		double tiltControl = master.get_digital(DIGITAL_R1) - master.get_digital(DIGITAL_L1);
		if(!tilterInUse || tiltControl) {
			mtrs->tilter.controllerSet(tiltControl);
			tilterInUse = false;
		}

		//Arms down, Tray in
		if(master.get_digital_new_press(DIGITAL_Y)) {
			mtrs->lift.moveAbsolute(initialLiftPos, 200);
			liftInUse = true;
			mtrs->left.controllerSet(0);
			mtrs->right.controllerSet(0);
			mtrs->tilter.controllerSet(0);
			mtrs->intake.controllerSet(0);
			pros::delay(400);
			mtrs->tilter.moveAbsolute(initialTilterPos, 200);
			tilterInUse = true;
		}
		
		//Tray in
		if(master.get_digital_new_press(DIGITAL_X)) {
			mtrs->tilter.moveAbsolute(initialTilterPos, 200);
			tilterInUse = true;
		}

		mtrs->left .controllerSet(y_ctrl + x_ctrl);
		mtrs->right.controllerSet(y_ctrl - x_ctrl);

		//If the arms are far away, intake slower.
		double intakeMultiplier = int(std::abs(mtrs->lift.getPosition() - initialLiftPos) > 0.75) * 0.2;
		mtrs->intake.controllerSet((master.get_digital(DIGITAL_R2) ? 1 : 0) - master.get_digital(DIGITAL_L2));

		pros::delay(10);
	}
}
