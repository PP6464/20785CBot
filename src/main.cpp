// Included libraries
#include "main.h"
#include "list"
// Define port numbers
#define LEFT_WHEEL_PORTS {1, 2, 3}
#define RIGHT_WHEEL_PORTS {4, 5, 6}

using namespace pros;
using namespace std;

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
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

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	prevcount=0
	while (1){
		status = pros::screen_touch_status();
		if status.press_count!=prevcount{
			Rightdrive.move_voltage(12000);
			Leftdrive.move_voltage(12000);
  			pros::delay(1000); // Move at max voltage for 1 second
  			Rightdrive.move_voltage(0);
			Leftdrive.move_voltage(0);
			prevcount=status.press_count;
		}
	}
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
void opcontrol() {
	Controller master(E_CONTROLLER_MASTER);
	pros::Motor lhs_1 (20,1,0); //port, internal gearing (1=green,2=blue), reverse
  	pros::Motor lhs_2 (19,1,0);
	pros::Motor lhs_3 (18,1,1);
  	pros::Motor_Group Leftdrive ({lhs_1,lhs_2,lhs_3});
	pros::Motor rhs_1 (21,1,1); 
  	pros::Motor rhs_2 (10,1,1);
	pros::Motor rhs_3 (8,1,0);
  	pros::Motor_Group Rightdrive ({lhs_1,lhs_2,lhs_3});

	while (true) {

		Leftdrive.move(127*(master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y)/10)^3);
		Rightdrive.move(127*(master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y)/10)^3);

		pros::delay(20);
	}
}
