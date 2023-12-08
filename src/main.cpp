// Included libraries
#include "main.h"
#include "list"
#include "lemlib/api.hpp"
// Define port numbers

using namespace pros;
using namespace std;
void screen() {
    // loop forever
    while (true) {
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        pros::lcd::print(0, "x: %f", pose.x); // print the x position
        pros::lcd::print(1, "y: %f", pose.y); // print the y position
        pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
        pros::delay(10);
    }
}
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
	Controller master(E_CONTROLLER_MASTER);
	pros::Motor lhs_1 (2,E_MOTOR_GEARSET_06,false); //port, internal gearing (1=green,2=blue), reverse
  	pros::Motor lhs_2 (1,E_MOTOR_GEARSET_06,false);
	pros::Motor lhs_3 (4,E_MOTOR_GEARSET_06,true);
  	pros::Motor_Group Leftdrive ({lhs_1,lhs_2,lhs_3});
	pros::Motor rhs_1 (14,E_MOTOR_GEARSET_06,true); 
  	pros::Motor rhs_2 (15,E_MOTOR_GEARSET_06,true);
	pros::Motor rhs_3 (3,E_MOTOR_GEARSET_06,false);
  	pros::Motor_Group Rightdrive ({rhs_1,rhs_2,rhs_3});
	lemlib::Drivetrain_t drivetrain {
    	&Leftdrive, // left drivetrain motors
    	&Rightdrive, // right drivetrain motors
    	10, // track width
    	3.25, // wheel diameter
    	360 // wheel rpm
	};
	pros::Imu inertial_sensor(2);
	inertial_sensor.reset();
	lemlib::OdomSensors_t sensors {&inertial_sensor};
	lemlib::ChassisController_t lateralController {
    	8, // kP
    	30, // kD
    	1, // smallErrorRange
    	100, // smallErrorTimeout
    	3, // largeErrorRange
    	500, // largeErrorTimeout
    	5 // slew rate
	};
 
	// turning PID
	lemlib::ChassisController_t angularController {
    	4, // kP
    	40, // kD
    	1, // smallErrorRange
    	100, // smallErrorTimeout
    	3, // largeErrorRange
    	500, // largeErrorTimeout
    	40 // slew rate
	};
 
 
	// create the chassis
	lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);
	chassis.setPose(0, 0, 0); // X: 0, Y: 0, Heading: 0
	pros::Task screenTask(screen);
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
	chassis.moveTo(-10, 0, 1000); // drive PID tuning
	//chassis.turnTo(0,-10,1000); // turn PID tuning
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
	while true{
		Leftdrive.move(master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y)^3 * 27/10^6)
	}
}
