#include "main.h"
#include "tabu.hpp"
#include "entropy.hpp"
#include "followtest.hpp"
#include "ultrasonic.hpp"
#include "pidtest.hpp"

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

std::string bruh() {
	std::string acc = "";
	int c = 0;
	while((c = getchar()) != -1  && c != '\n') {
		acc += c;
	}
	return acc;
}

void inputTask(void*) {
	while(true) {
		auto line = bruh();
		tabu_handler(line);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	try {
		init_random();
		init_follow_test();
		init_pid_test();
		init_sonic();
		printf("da - ke - do, suki nano nano daisuki nano nano\n");

		tabu_reply_on("ping", [](const Message& msg) -> json {
			return "Got ping message with content " + msg.content.dump() + ".";
		});

		pros::lcd::initialize();
		pros::lcd::set_text(1, "Hello PROS User!");

		pros::lcd::register_btn1_cb(on_center_button);

		pros::Task maInput(inputTask);
	} catch(const char* initError) {
		printf("Got an init error: %s\n", initError);
		fflush(stdout);
		pros::delay(500);
		throw;
	} catch(const std::exception& except) {
		printf("Got an init error: %s\n", except.what());
		fflush(stdout);
		pros::delay(500);
		throw;
	} catch(...) {
		printf("Got some init error.\n");
		fflush(stdout);
		pros::delay(500);
		throw;
	}
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}
