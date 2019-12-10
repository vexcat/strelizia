#include "main.h"
#include "okapi/api.hpp"
#include "blue_controller.hpp"
#include "mtrs.hpp"
#include "sensors.hpp"
#include "atoms.hpp"

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
	mtrs->lift.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	mtrs->tilter.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	double initialTilterPos = 0;
	double initialLiftPos = 0;
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
			mtrs->tilter.controllerSet(((mtrs->tilter.getPosition() > 2.67 && tiltControl > 0) ? 0.7 : 1.0) * tiltControl);
			tilterInUse = false;
		}
		
		//Tray in
		if(master.get_digital_new_press(DIGITAL_Y)) {
			mtrs->tilter.moveAbsolute(initialTilterPos, 200);
			tilterInUse = true;
		}

		if(master.get_digital_new_press(DIGITAL_UP)) {
			((void (*)())retrieve_hawt_atom("auto"))();
		}

		if(master.get_digital_new_press(DIGITAL_RIGHT)) {
      mtrs->tilter.moveAbsolute(3.98, 200);
			tilterInUse = true;
		}

		mtrs->left .controllerSet(y_ctrl + x_ctrl);
		mtrs->right.controllerSet(y_ctrl - x_ctrl);

		mtrs->intake.controllerSet(0.80 * (master.get_digital(DIGITAL_R2) - master.get_digital(DIGITAL_L2)));

		pros::delay(10);
	}
}
