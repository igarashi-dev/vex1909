#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/ai_vision.hpp"
#include "pros/colors.hpp"
#include "vision_tracker.hpp"
#include "pros/rtos.hpp"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-18,  -19},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({9,10}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)



// Inertial Sensor on port 10
pros::Imu imu(16);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-11);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller

lemlib::ControllerSettings linearController(1.85, // proportional gain (kP)
                                            0.0014, // integral gain (kI)
                                            8, // derivative gain (kD)
                                            0, // anti windup
                                            .5, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            1.25, // large error range, in inches
                                            300, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(1.95, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             14.2, // derivative gain (kD)
                                             1, // anti windup
                                             0.7, // small error range, in degrees
                                             150, // small error range timeout, in milliseconds
                                             1.25, // large error range, in degrees
                                             400, // large error range timeout, in milliseconds
                                             127 // maximum acceleration (slew)
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
    vision_tracker_init();
     

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
            //pros::lcd::print(3, "Center Ai %d", horizontalEnc.get_position()); // left tracking wheel encoder
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
double distance=0;//
/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {

//double newX = chassis.getPose().x + distance * sin(chassis.getPose().theta * (M_PI / 180.0));
//double newY = chassis.getPose().y + distance * cos(chassis.getPose().theta * (M_PI / 180.0));
chassis.turnToHeading(90, 2000); // turn to 90 degrees with a 2 second timeout

}
int forward=0;
 
bool tracking = false;
/**
 * Runs in driver control
 */
void opcontrol() {
    //vision1.enable_detection_types()
    //vision1.set_color();
   
    
	    //vision1.set_color(color);
    
    // controller
    // loop to continuously update motors
    while (true) {
         /*aivision.reset();
    aivision.enable_detection_types(pros::AivisionModeType::colors);
    pros::AIVision::Color color = {.id = 1, .red = 32, .green = 142, .blue = 194, .hue_range = 20, .saturation_range = 0.5};
    pros::AIVision::Color color2 = {.id = 2, .red = 255, .green = 255, .blue = 153, .hue_range = 30, .saturation_range = 0.5};
    aivision.set_color(color);
    pros::AIVision::Code code = {.id = 1, .length = 2, .c1 = 1, .c2 = 2};
    aivision.set_code(code);
    aivision.set_color(color2);
    aivision.set_code({.id = 2, .length = 2, .c1 = 1, .c2 = 2});
    auto objects = aivision.get_all_objects();


 for (auto &object : objects) {
    if (pros::AIVision::is_type(object, pros::AivisionDetectType::color)) {

        int turn = 0;   
        if (abs(object.object.color.xoffset) > 170) {
            turn = object.object.color.xoffset * 0.5;;
        }
        

        if (abs(object.object.color.xoffset) < 150) {
            turn = object.object.color.xoffset * -0.5;;
        }
if (abs(object.object.color.xoffset) >150 && abs(object.object.color.xoffset) < 170) {
            forward = 80;   
        }
    }
}*/
if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
    tracking = true;
}
if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
    tracking = false;
}
if(tracking) {
vision_tracker_enable(true);
} else {
vision_tracker_enable(false);

        // get joystick positions
        int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        // move the chassis with curvature drive
        chassis.arcade(rightY, leftX);
}
        
       

   
        // delay to save resources
        pros::delay(10);
    }
}
