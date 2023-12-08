// Included libraries
#include "main.h"
#include "list"
#include "lemlib/api.hpp"
// Define port numbers

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
Controller master(E_CONTROLLER_MASTER);
pros::Motor catapult (3, E_MOTOR_GEARSET_36, false);
pros::Motor lhs_1 (17,E_MOTOR_GEARSET_06,false); //port, internal gearing (1=green,2=blue), reverse
pros::Motor lhs_2 (20,E_MOTOR_GEARSET_06,false);
pros::Motor lhs_3 (19,E_MOTOR_GEARSET_06,true);
pros::Motor_Group Leftdrive ({lhs_1,lhs_2,lhs_3});
pros::Motor rhs_1 (7,E_MOTOR_GEARSET_06,true); 
pros::Motor rhs_2 (5,E_MOTOR_GEARSET_06,true);
pros::Motor rhs_3 (6,E_MOTOR_GEARSET_06,false);
pros::Motor_Group Rightdrive ({rhs_1,rhs_2,rhs_3});
pros::Motor blocker(2, E_MOTOR_GEARSET_18, false);
pros::Motor intake(10, E_MOTOR_GEARSET_18, false);
lemlib::Drivetrain_t drivetrain {
	&Leftdrive, // left drivetrain motors
	&Rightdrive, // right drivetrain motors
	10, // track width
	3.25, // wheel diameter
	360 // wheel rpm
};
bool intakeOn = false; 
pros::Imu inertial_sensor(9);
lemlib::OdomSensors_t sensors {nullptr,nullptr,nullptr,nullptr,&inertial_sensor};
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

void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
	inertial_sensor.reset();
 
 
	// create the chassis
	chassis.setPose(0, 0, 0); // X: 0, Y: 0, Heading: 0
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
void competition_initialize() {
	Rightdrive.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	Leftdrive.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	blocker.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	cata.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

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
	chassis.moveTo(-10, 0, 1000); // drive PID tunin
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
 *a
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	while (true) {
		Leftdrive.move_voltage(master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y)^3*12000/127^3);
		Rightdrive.move_voltage(master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y)^3*12000/127^3);

		if (master.get_digital(E_CONTROLLER_DIGITAL_R2) == 1){
			catapult.move_voltage(12000);
		};
		if (master.get_digital(E_CONTROLLER_DIGITAL_L1))
		{
			blocker.move_voltage(-12000);
		}
		else if (master.get_digital(E_CONTROLLER_DIGITAL_L2))
		{
			blocker.move_voltage(12000);
		}
		else{
			blocker.brake();
		}
		if (master.get_digital(E_CONTROLLER_DIGITAL_R1))
		{
			intakeOn = !intakeOn;
		}
		if (intakeOn)
		{
			intake.move_voltage(12000);
		}
		else{
			intake.brake();
		}
	}
	pros::delay(20);
}
