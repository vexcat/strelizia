#include "main.h"
#include "okapi/api.hpp"
#include "blue_controller.hpp"
#include "mtrs.hpp"
#include "sensors.hpp"
#include "atoms.hpp"
#include "blackbox.hpp"

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
	//mtrs->intake.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	mtrs->tilter.setEncoderUnits(okapi::AbstractMotor::encoderUnits::rotations);
	//mtrs->tilter.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	auto lastOuttakePress = pros::millis();
	bool highSpeedMode = false;
	bool tilterInUse = false;
	int i = 0;
	while (true) {
		if(!opcontrolActive) {
			opcontrolActiveAck = false;
			pros::delay(10);
			continue;
		}
		opcontrolActiveAck = true;
		auto directControl = master.get_digital(DIGITAL_UP) - master.get_digital(DIGITAL_DOWN);
		double y_ctrl  = powered(master.get_analog(ANALOG_LEFT_Y), 1.3);
		double x_ctrl = powered(master.get_analog(ANALOG_LEFT_X), 1.3);
		if(directControl) {
			mtrs->all.controllerSet(directControl);
		} else {
			mtrs->left .controllerSet(y_ctrl + x_ctrl);
			mtrs->right.controllerSet(y_ctrl - x_ctrl);
		}
		
		double liftControl = -master.get_analog(ANALOG_RIGHT_Y) / 127.0;
		double tiltControl = master.get_digital(DIGITAL_R1) - master.get_digital(DIGITAL_L1);
		double liftSpeed = 1;

		if(std::abs(liftControl) > 0.1) {
			mtrs->lift.controllerSet(liftControl * liftSpeed);
			//If arms are above the threshold, tilter must be > 1 (1.2 ideal)
			if(mtrs->lift.getPosition() < -0.38 && mtrs->tilter.getPosition() < 1) {
				mtrs->tilter.moveAbsolute(1.2, 100);
				tilterInUse = true;
			}
			//If arms are below the threshold, tilter should automatically lower ( < 0.5, 0 ideal).
			if(mtrs->lift.getPosition() > -0.38 && mtrs->tilter.getPosition() > 0.5) {
				mtrs->tilter.moveAbsolute(0, 100);
				tilterInUse = true;
			}
		} else {
			mtrs->lift.controllerSet(0);
		}

		if(tiltControl) {
			mtrs->tilter.controllerSet(tiltControl * (tiltControl > 0 && mtrs->tilter.getPosition() > 1.2 ? 0.45 : 1.0));
			tilterInUse = false;
		} else if(!tilterInUse) {
			mtrs->tilter.controllerSet(0);
		}

		if(bumper->get_value()) {
			mtrs->liftRaw.tarePosition();
		}
		
		//Tray in
		if(master.get_digital_new_press(DIGITAL_Y)) {
			mtrs->tilter.moveAbsolute(0, 200);
			tilterInUse = true;
		}

		if(master.get_digital_new_press(DIGITAL_LEFT)) {
			((void (*)())retrieve_hawt_atom("auto"))();
		}

		auto intakeCtrl = 0.0;
		if(master.get_digital_new_press(DIGITAL_L2)) {
			if(pros::millis() - lastOuttakePress < 500) {
				highSpeedMode = true;
			}
			lastOuttakePress = pros::millis();
		}
		if(master.get_digital(DIGITAL_L2)) {
			intakeCtrl = highSpeedMode ? -1 : -0.4;
		} else if(master.get_digital(DIGITAL_R2)) {
			intakeCtrl = 1;
		} else {
			highSpeedMode = false;
		}
		if(master.get_digital_new_press(DIGITAL_B)) {
			mtrs->tilter.tarePosition();
		}

		if(master.get_digital_new_press(DIGITAL_A)) {
			if(toggle_blackbox()) {
				master.clear_line(0);
				pros::delay(51);
				master.set_text(0, 0, "Record Active");
				pros::delay(51);
			} else {
				master.clear_line(0);
				pros::delay(51);
				master.set_text(0, 0, "Record Off");
				pros::delay(51);
			}
		}

		mtrs->intake.controllerSet(intakeCtrl);
		//if(i % 10 == 0) printf("%f\n", imuPtr->get_rotation());

		pros::delay(10);
		i++;
	}
}
