// Included libraries
#include "main.h"
#include "list"
#include "lemlib/api.hpp"
#include "cmath"
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
pros::Motor slapper1 (3, E_MOTOR_GEARSET_36, false,E_MOTOR_ENCODER_DEGREES);
pros::Motor slapper2 (21, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
pros::Motor_Group slapper ({slapper1,slapper2});
pros::Motor lhs_1 (17,E_MOTOR_GEARSET_06,false,E_MOTOR_ENCODER_DEGREES); //port, internal gearing (1=green,2=blue), reverse
pros::Motor lhs_2 (20,E_MOTOR_GEARSET_06,false,E_MOTOR_ENCODER_DEGREES);
pros::Motor lhs_3 (19,E_MOTOR_GEARSET_06,true,E_MOTOR_ENCODER_DEGREES);
pros::Motor_Group Leftdrive ({lhs_1,lhs_2,lhs_3});
pros::Motor rhs_1 (7,E_MOTOR_GEARSET_06,true,E_MOTOR_ENCODER_DEGREES); 
pros::Motor rhs_2 (5,E_MOTOR_GEARSET_06,true,E_MOTOR_ENCODER_DEGREES);
pros::Motor rhs_3 (6,E_MOTOR_GEARSET_06,false,E_MOTOR_ENCODER_DEGREES);
pros::Motor_Group Rightdrive ({rhs_1,rhs_2,rhs_3});
pros::Motor intake(10, E_MOTOR_GEARSET_18, false,E_MOTOR_ENCODER_DEGREES);
pros::ADIDigitalOut Lift (A, LOW);
pros::ADIDigitalOut Wing (B, LOW);
pros::ADIDigitalOut AWP (C, LOW);
pros::ADIAnalogIn LineTracker (D);
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
	Lift.set_value(LOW);
	Wing.set_value(LOW);
	AWP.set_value(LOW);
	slapper.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
	intake.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
	Leftdrive.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
	Rightdrive.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
	LineTracker.calibrate(D);
	bool load_or_block=false;
	while (true){
		if (!(load_or_block)){
			Leftdrive.move_voltage(12000*((arctan(2*(2*(master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y/127))-((master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y/127)))/((abs(master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y/127)))))))/(2*arctan(2))));
			Rightdrive.move_voltage(12000*((arctan(2*(2*(master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y/127))-((master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y/127)))/((abs(master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y/127)))))))/(2*arctan(2))));}
		else{
			Leftdrive.brake();
			Rightdrive.brake();
		}
		if(master.get_digital(E_CONTROLLER_DIGITAL_A)){
			Leftdrive.set_reversed(!(Leftdrive.is_reversed()));
			Rightdrive.set_reversed(!(Rightdrive.is_reversed()));
			pros::delay(500);
		}
		if (load_or_block){
			if (abs(LineTracker.get_value_calibrated())>500){
				pros::delay(500);
				slapper.tare_position();
				slapper.move_absolute(120,100);
			}
		}
		if (master.get_digital(E_CONTROLLER_DIGITAL_R1)){
			Lift.set_value(LOW);
		}
		else if (master.get_digital(E_CONTROLLER_DIGITAL_R2)){
			Lift.set_value(HIGH);
		}
		if (master.get_digital(E_CONTROLLER_DIGITAL_L1)){
			Intake.tare_position();
			Intake.set_reversed(false);
			Intake.move_absolute(90,100);
		}
		else if (master.get_digital(E_CONTROLLER_DIGITAL_L2)){
			Intake.tare_position();
			Intake.set_reversed(true);
			Intake.move_absolute(90,100);
		}
		if (master.get_digital(E_CONTROLLER_DIGITAL_RIGHT)){
			Wing.set_value(LOW);
		}
		else if (master.get_digital(E_CONTROLLER_DIGITAL_LEFT)){
			Wing.set_value(HIGH);
		}
	}
	pros::delay(20);
}
