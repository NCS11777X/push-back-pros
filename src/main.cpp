#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-2, -12, -14},
                            pros::MotorGearset::green); // left motor group - ports 2 (reversed), 12 (reversed), 14 (reversed)
                            
pros::MotorGroup rightMotors({1, 11, 13}, pros::MotorGearset::green); // right motor group - ports 1, 11, 13

pros::Motor lower_intake(16); 	   // Creates a motor for the lower intake

pros::Motor upper_intake(15);		   // Creates a motor for the upper intake

pros::Motor lower_back_intake(-3);   // Creates a motor for the lower back intake

pros::Motor upper_back_intake(-4);   // Creates a motor for the upper back intake

pros::adi::DigitalOut flap('h'); // Creates a group for the pistons that power the flap

pros::adi::DigitalOut tracking_wheel('g');  // Creates a piston for the tracking wheel

// Inertial Sensor on port 5
pros::Imu imu(5);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 7, reversed
pros::Rotation horizontalEnc(-7);
// vertical tracking wheel encoder. Rotation sensor, port 8, reversed
pros::Rotation verticalEnc(-8);
// horizontal tracking wheel. 2" diameter, 7.5" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -7.5);
// vertical tracking wheel. 2" diameter, 1.625" offset, right of the robot (positive)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 1.625);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10.5, // 10 inch track width
                              lemlib::Omniwheel::OLD_275, // using new 4" omnis
                              333, // drivetrain rpm is 333 + 1/3
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            0, // anti windup
                                            0, // small error range, in inches
                                            0, // small error range timeout, in milliseconds
                                            0, // large error range, in inches
                                            0, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(10, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             100, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    tracking_wheel.set_value(1);   // activates the piston
    

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs
    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {

    //chassis.setPose(0, 0, 0);

    //chassis.moveToPose(0, 100, 0, 5000);

    //upper_intake.move(127);
    //pros::delay(100);
    //upper_intake.brake();
}

/**
 * Runs in driver control
 */
void opcontrol() {

    tracking_wheel.set_value(0); // pulls the tracking wheel up

    // loop to continuously update motors
    while (true) {


        // drivetrain controls
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);    // get joystick positions
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);  // get joystick positions
        
        chassis.arcade(leftY, rightX);  // move the chassis with curvature drive
        

        // pneumatics control
        int flapState = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);

        flap.set_value(flapState);


        // intake control system
        if(controller.get_digital(DIGITAL_R2)){
            lower_intake.move(-127);
            upper_intake.move(127);
            lower_back_intake.move(127);
            upper_back_intake.move(127);
        }
        else if (controller.get_digital(DIGITAL_R1)) {
            lower_intake.move(127);
            upper_intake.move(-127);
            lower_back_intake.move(-127);
            upper_back_intake.move(-127);
        }
        else {
            lower_intake.move(0);
            upper_intake.move(0);
            lower_back_intake.move(0);
            upper_back_intake.move(0);
        }
               
        
        pros::delay(10);    // delay to save resources
    }
}
