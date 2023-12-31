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
pros::Motor catapult (3, E_MOTOR_GEARSET_36, false,E_MOTOR_ENCODER_DEGREES);
pros::Motor lhs_1 (17,E_MOTOR_GEARSET_06,false,E_MOTOR_ENCODER_DEGREES); //port, internal gearing (1=green,2=blue), reverse
pros::Motor lhs_2 (20,E_MOTOR_GEARSET_06,false,E_MOTOR_ENCODER_DEGREES);
pros::Motor lhs_3 (19,E_MOTOR_GEARSET_06,true,E_MOTOR_ENCODER_DEGREES);
pros::Motor_Group Leftdrive ({lhs_1,lhs_2,lhs_3});
pros::Motor rhs_1 (7,E_MOTOR_GEARSET_06,true,E_MOTOR_ENCODER_DEGREES); 
pros::Motor rhs_2 (5,E_MOTOR_GEARSET_06,true,E_MOTOR_ENCODER_DEGREES);
pros::Motor rhs_3 (6,E_MOTOR_GEARSET_06,false,E_MOTOR_ENCODER_DEGREES);
pros::Motor_Group Rightdrive ({rhs_1,rhs_2,rhs_3});
pros::Motor blocker(2, E_MOTOR_GEARSET_18, false,E_MOTOR_ENCODER_DEGREES);
pros::Motor intake(10, E_MOTOR_GEARSET_18, false,E_MOTOR_ENCODER_DEGREES);
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
void disabled() {}
void competition_initialize() {
}
void autonomous() {
	chassis.moveTo(-10, 0, 1000); // drive PID tunin
	//chassis.turnTo(0,-10,1000); // turn PID tuning
}
void opcontrol() {
	Rightdrive.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	Leftdrive.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	blocker.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	cata.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	bool intaken=false;
	double x=720; //ZUHEB - change this to the number of degrees
	while (true) {
		Leftdrive.move_voltage(master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y)^3*12000/127^3);
		Rightdrive.move_voltage(master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y)^3*12000/127^3);

		if (master.get_digital(E_CONTROLLER_DIGITAL_R2) == 1){
			catapult.move_relative(x,200);
			pros::delay(200);
		}
		if (master.get_digital(E_CONTROLLER_DIGITAL_A) == 1){
			catapult.move_voltage(12000);
		}
		else{
			catapult.brake();
		}
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
		if (master.get_digital(E_CONTROLLER_DIGITAL_R1)){
			if (intaken){
				intake.move_voltage(-12000);
				pros::delay(1000); //ZUHEB - decrease this until the triball barely comes out of the robot
			}
			else{
				intake.move_voltage(12000);
			}
		}
		if (intake.get_actual_velocity()<50){
			intaken=true;
		}
	}
	pros::delay(20);
}
