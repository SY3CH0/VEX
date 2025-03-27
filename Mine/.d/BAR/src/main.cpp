#include "main.h"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/core/lv_obj.h"
#include "liblvgl/widgets/lv_img.h"
#include "pros/adi.hpp" // pneumatics
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/optical.hpp"
#include "pros/rtos.hpp" // tasks
#include "pros/llemu.hpp" // brain screen
#include "pros/imu.hpp" // inertial sensor
#include "pros/motors.hpp" // motor groups
LV_IMG_DECLARE(rob);
unsigned long matchStartTime;
// yug is a gud boi
// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-8, 9, -10},
                            pros::MotorGearset::blue); // left motor group - ports 1, 2, 3 (not reversed)
pros::MotorGroup rightMotors({1, -2, -3}, pros::MotorGearset::blue); // right motor group - ports 4, 5, 6 ( not reversed)

//motors
pros::Motor belt(-7, pros::v5::MotorGears::blue); // belt motor on port 7; reversed
pros::Motor intake(20, pros::v5::MotorGears::green); // flex wheel motor on port 8; not reversed
// Define the arm motor on port 5
pros::Motor arm(19, pros::v5::MotorGears::green);

//Arm States
//arm states
const int numStates = 3;
//make sure these are in centidegrees (1 degree = 100 centidegrees)
int states[numStates] = {0, 300, 2000};
int currState = 0;
int target = 0;

void nextState() {
    currState += 1;
    if (currState == numStates) {
        currState = 0;
    }
    target = states[currState];
}

void liftControl() {
    double kp = 0.5;
    double error = target - arm.get_position();
    double velocity = kp * error;
    arm.move(velocity);
}



// Inertial Sensor on port 10
pros::Imu imu(6);

pros::Optical colorSensor(5);

// Define the limit switch on port 'C'
pros::adi::Button limitSwitch('C');

// Pneumatics
pros::adi::Pneumatics clampP1('A', false); // clamp on port A; starts off retracted
pros::adi::Pneumatics doinker('B', false); // doinker on port C; starts off retracted
bool pistonToggle = false; // toggle for pneumatics

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-11);
// horizontal tracking wheel. 2" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -5.75);
// vertical tracking wheel. 2" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, -2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
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
   // pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    lv_obj_t *img = lv_img_create(lv_scr_act());
    lv_img_set_src(img, &rob);
    lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    colorSensor.set_led_pwm(255); // set color sensor LED to full brightness
     

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
           // pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            //pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            //pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
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
    
}

/**
 * Runs in driver control
 */
 void opcontrol() {
    matchStartTime = pros::millis();
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(rightX, leftY);
        // delay to save resources
        pros::delay(10);

        unsigned long elapsedTime = pros::millis() - matchStartTime;

		// clamp
		if (controller.get_digital(DIGITAL_B)){
			if(pistonToggle == false){
				clampP1.extend();
                
				pros::delay(500);
				pistonToggle = true;

			}
			else{
				clampP1.retract();
                
				pros::delay(500);
				pistonToggle = false;
			}
		}

        // doinker
		if (controller.get_digital(DIGITAL_Y)){
			if(pistonToggle == false){
				doinker.extend();
				pros::delay(500);
				pistonToggle = true;

			}
			else{
				doinker.retract();
				pros::delay(500);
				pistonToggle = false;
			}
		}

		// intake
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			intake.move_velocity(200);
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			intake.move_velocity(-200);
		}
		else{
			intake.move_velocity(0);
		}

		// belt
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			belt.move_velocity(600);
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            belt.move_velocity(-600);
		}
		else{
			belt.move_velocity(0);
		}


        // color sensor
        int hue = colorSensor.get_hue();
        if (hue >= 200 && hue <= 250) {
            // Perform actions when color sensor detects values between 200 and 250 
            belt.move_velocity(600);
            pros::delay(50);
            belt.move_velocity(0);
        } else {
            // Perform actions when color sensor does not detect values in range
           // pros::lcd::print(2, "Color out of range: %d", hue);
        }

        if (elapsedTime < 85000) { // 85000 milliseconds = 85 seconds (120 - 35)
            if (limitSwitch.get_value()) {
                // Activate the clamp if the limit switch is pressed
                clampP1.extend();
                pistonToggle = true;
            } else {
                // User control for the clamp
                if (controller.get_digital(DIGITAL_B)) {
                    if (pistonToggle == false) {
                        pros::delay(300);
                        clampP1.extend();
                        pros::delay(500);
                        pistonToggle = true;
                    } else {
                        clampP1.retract();
                        pros::delay(500);
                        pistonToggle = false;
                    }
                }
            }
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {  // change R1 to smth diff
			nextState();
		}
	    pros::delay(20);

    }
}