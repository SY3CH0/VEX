#include "main.h"
#include "pros/adi.hpp" // pneumatics
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/rtos.hpp" // tasks
#include "pros/llemu.hpp" // brain screen
#include "pros/imu.hpp" // inertial sensor
#include "pros/motors.hpp" // motor groups

pros::Rotation odomR(11); // Replace with the correct port number
pros::Rotation odomL(12); // Replace with the correct port number

pros::MotorGroup leftMotors({-2, -1},
    pros::MotorGearset::green); // left motor group - ports 2, 1       
pros::MotorGroup rightMotors({-3, -4}, pros::MotorGearset::green); // right motor group - ports 3, 4

bool resetSensors = false;
bool drivePIDRunning = false;

/*
void drivePIDs(bool drivePID, int targetValue) {
    double kP = 0.5, kI = 1.5, kD = 1.5;
    int integral = 0, error = 0, prevError = 0, derivative = 0, LateralAction = 0;


    if (drivePID) {
        resetSensors = false;
        leftMotors.tare_position();
        leftMotors.tare_position();
    }


    drivePIDRunning = true;
    while (drivePIDRunning) {
        int odomRAvgPos = odomR.get_position();
        int odomLAvgPos = odomL.get_position();
        int AveragePosition = ( odomRAvgPos) / 1;


        error = targetValue - AveragePosition;
        integral += error;
        if (abs(error) < 10){
            integral = 0;}  // Reset integral if error is small
        derivative = error - prevError;
        LateralAction = (error * kP) + (derivative * kD)+ (integral *kI) ;


        leftMotors.move(LateralAction);
        rightMotors.move(LateralAction);


        prevError = error;
        pros::delay(20);


        if(abs(prevError) < 5)
{
    leftMotors.move_velocity(0);
    rightMotors.move_velocity(0);


    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    break;
}      
    } }
*/

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
// pros::MotorGroup intake({-6, 7}, pros::MotorGearset::green); // intake motor group - ports 6, 7

//motors
pros::Motor intake(-6, pros::v5::MotorGears::green); // flex wheel motor on port 6; reversed
pros::Motor belt(7, pros::v5::MotorGears::blue); // belt motor on port 7

// Inertial Sensor on port 10
pros::Imu imu(9);


// Pneumatics
pros::adi::Pneumatics piston1('A', false); // Piston 1 on port A; starts off retracted
pros::adi::Pneumatics piston2('B', false); // Piston 2 on port B; starts off retracted
pros::adi::Pneumatics doinker('C', false); // Piston 2 on port B; starts off retracted

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
// pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
// pros::Rotation verticalEnc(-11);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
// lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
// lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12.5, // 10 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              8 // horizontal drift is 2. If we had traction wheels, it would have been 8
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
lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &imu);

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


// Runs during auto
 
// This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 
/*
void autonomous() {
    // Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    chassis.moveToPose(20, 15, 90, 4000);
    // Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms
    chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});
    // cancel the movement after it has traveled 10 inches
    chassis.waitUntil(10);
    chassis.cancelMotion();
    // Turn to face the point x:45, y:-45. Timeout set to 1000
    // dont turn faster than 60 (out of a maximum of 127)
    chassis.turnToPoint(45, -45, 1000, {.maxSpeed = 60});
    // Turn to face a direction of 90º. Timeout set to 1000
    // will always be faster than 100 (out of a maximum of 127)
    // also force it to turn clockwise, the long way around
    chassis.turnToHeading(90, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 100});
    // Follow the path in path.txt. Lookahead at 15, Timeout set to 4000
    // following the path with the back of the robot (forwards = false)
    // see line 116 to see how to define a path
    chassis.follow(example_txt, 15, 4000, false);
    // wait until the chassis has traveled 10 inches. Otherwise the code directly after
    // the movement will run immediately
    // Unless its another movement, in which case it will wait
    chassis.waitUntil(10);
    pros::lcd::print(4, "Traveled 10 inches during pure pursuit!");
    // wait until the movement is done
    chassis.waitUntilDone();
    pros::lcd::print(4, "pure pursuit finished!");
}
*/

void autonomous() {
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
    chassis.turnToHeading(90, 100000);
}

// Runs in driver control
/*
void opcontrol() {
    // loop forever

	bool pistonToggle = false;

    while (true) {
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(rightX, leftY);

        // delay to save resources
        pros::delay(25);

		// clamp
		if (controller.get_digital(DIGITAL_X)){
			if(pistonToggle == false){
				piston1.extend();
				piston2.extend();
				pros::delay(500);
				pistonToggle = true;

			}
			else{
				piston1.retract();
				piston2.retract();
				pros::delay(500);
				pistonToggle = false;
			}
		}

		// intake
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			intake.move_velocity(10000);
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			intake.move_velocity(10000);
		}
		else{
			intake.move_velocity(0);
		}

		// belt
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			belt.move_velocity(200);
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			belt.move_velocity(-200);
		}
		else{
			belt.move_velocity(0);
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
    }
}
    */
