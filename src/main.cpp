#include "main.h"
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"
#include <string>

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
	} 
	else {
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
	/*
	pros::MotorGroup left_mg({-2, -12, -14}, pros::MotorGearset::green);	// Creates a motor group with forwards ports 2, 12, and 14
	pros::MotorGroup right_mg({1, 11, 13}, pros::MotorGearset::green);		// Creates a motor group with reversed ports 1, 11, and 13
	pros::Motor lower_intake(15); 			   		// Creates a motor for the lower intake
	pros::Motor upper_intake(16);						// Creates a motor for the upper intake
	pros::Motor lower_back_intake(3);    	   			// Creates a motor for the lower back intake
	pros::Motor upper_back_intake(4);		   			// Creates a motor for the upper back intake

	left_mg.move_relative(100, 100);
	right_mg.move_relative(100, 100);
	while (!((left_mg.get_position() < 105) && (left_mg.get_position() > 95))) {
		pros::delay(2);
	}
	left_mg.brake();
	right_mg.brake();
	*/
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
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::MotorGroup left_mg({-2, -12, -14}, pros::MotorGearset::green);	// Creates a motor group with forwards ports 2, 12, and 14
	pros::MotorGroup right_mg({1, 11, 13}, pros::MotorGearset::green);		// Creates a motor group with reversed ports 1, 11, and 13
	pros::Motor lower_intake(15); 			   		// Creates a motor for the lower intake
	pros::Motor upper_intake(16);						// Creates a motor for the upper intake
	pros::Motor lower_back_intake(3);    	   			// Creates a motor for the lower back intake
	pros::Motor upper_back_intake(4);		   			// Creates a motor for the upper back intake
	pros::adi::DigitalOut flap('h');			   	// Creates a group for the pistons that power the flap
	

	while (true) {


		//autonomous();


		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    			// Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  			// Gets the turn left/right from right joystick

		left_mg.move(dir + turn);                      			// Sets left motor voltage
		right_mg.move(dir - turn);                     			// Sets right motor voltage


		// Flap control system (ignore that there isnt anything here)



		// Intake control system
		int intakeFwd = master.get_digital(DIGITAL_R2);							// Gets the state of R2
		int intakeRev = master.get_digital(DIGITAL_R1);							// Gets the state of R1

		lower_intake.move(-(intakeFwd - intakeRev) * 127);					    // Sets the lower intake voltage


		// Outtake control system
		int outtakeFwd = master.get_digital(DIGITAL_L2);
		int outtakeRev = master.get_digital(DIGITAL_L1);

		upper_intake.move((outtakeFwd - outtakeRev) * 127);						// Sets the upper intake voltage
		lower_back_intake.move((outtakeFwd - outtakeRev) * 127);				    // Sets the lower back intake voltage
		upper_back_intake.move((outtakeFwd - outtakeRev) * 127);					// Sets the upper back intake voltage

		
		// Debug information
		pros::lcd::set_text(0, "Left Analog Y:  " + std::to_string(dir));							// Prints the left analog y value
		pros::lcd::set_text(1, "Right Analog X: " + std::to_string(turn));							// Prints the right analog x value
		pros::lcd::set_text(2, "Intake direction: " + std::to_string(intakeFwd - intakeRev));		// Prints the direction of the intake
		pros::lcd::set_text(3, "Outtake direction: " + std::to_string(outtakeFwd - outtakeRev));	// Prints the direction of the outtake


		pros::delay(20);                         // Run for 20 ms then update
	}
}
