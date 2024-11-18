#include "main.h"
#include "lemlib/api.hpp"
#include <cmath>

pros::Controller controller(pros::E_CONTROLLER_MASTER);

//reverse later 1
pros::MotorGroup left_motors({-18, -19, -20}, pros::MotorGearset::blue); // left motors on ports 1, 2, 3
pros::MotorGroup right_motors({17, 14, 15}, pros::MotorGearset::blue); // right motors on ports 4, 5, 6

pros::MotorGroup intakeMotors({11, -12}, pros::MotorGearset::blue);

pros::ADIDigitalOut clampPiston('G');
pros::ADIDigitalOut hangPiston('D');
pros::ADIDigitalOut dongerPiston('H', true); 
//Drive train
lemlib::Drivetrain drivetrain(
	&left_motors, // left motor group2
	&right_motors, // right motor group
	11.4, // 11.4 inch track width
    lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
    450, // drivetrain rpm is 450
    2 // horizontal drift is 2 (for now)
);

pros::adi::Encoder vertical_encoder('E', 'F'); // left encoder on ports 1, 2 Relative to the tracking center: front is positive, back is negative
pros::Imu imu(16);

lemlib::TrackingWheel vertical_tracking_wheel(
	&vertical_encoder, // encoder
	lemlib::Omniwheel::NEW_275, 
	0.0 // 0 inch distance from tracking center
); 

lemlib::OdomSensors sensors(&vertical_tracking_wheel,
                            nullptr,
                            nullptr,
                            nullptr, 
                            &imu
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen

    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen 
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });
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

// ASSET(MidMogoRush_txt)
// void autonomous() {
//     // Set up chassis
//     chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
//     chassis.setPose(-50.513, -61.352, 270);

//     // Rush mid mogo
//     chassis.follow(MidMogoRush_txt, 10, 4000, false); // Rush mogo
//     chassis.waitUntilDone();

//     // Clamp mogo + Pull out
//     clampPiston.set_value(true);
//     pros::delay(400);
//     chassis.swingToHeading(20, lemlib::DriveSide::RIGHT, 1000); // Pull out of center (20)
//     chassis.waitUntilDone();

//     // Get the rings on the ground
//     intakeMotors.move_velocity(600);
//     chassis.moveToPoint(-11.293, -38.057, 500); // Part 1 intake ring
//     chassis.waitUntilDone();

//     // Turn to face wall (back facing wall)
//     chassis.turnToHeading(90, 700); // Turn to side (back facing wall)
//     pros::delay(400);
//     chassis.waitUntilDone();

//     // Move to wall
//     chassis.moveToPoint(-49.195, -38.057, 1500, {.forwards = false}); // goto wall
//     chassis.waitUntilDone();

//     // Release goal and face goal 2 (back facing goal 2)
//     intakeMotors.move_velocity(0);
//     clampPiston.set_value(false);
//     pros::delay(400);
//     chassis.turnToPoint(-23.04, -22.877, 700, {.forwards = false}); // Turn to face goal
//     chassis.waitUntilDone();

//     // Move to goal 2
//     chassis.moveToPoint(-23.04, -22.877, 2500, {.forwards = false, .maxSpeed = 60}); // Move to goal
//     chassis.waitUntilDone();

//     // Clamp goal 2
//     clampPiston.set_value(true);
//     pros::delay(400);
//     chassis.waitUntilDone();

//     // Turn to mid 
//     chassis.turnToPoint(-23.04, -9.478, 1000);
//     chassis.waitUntilDone();

//     // Move to mid 
//     intakeMotors.move_velocity(0);
//     chassis.moveToPoint(-23.04, -9.478, 2000, {.forwards = true, .maxSpeed = 50});
//     chassis.waitUntilDone();
// }

// Reverse 2
// ASSET(MidMogoRushR_txt)
// void autonomous() {
//     // Set up chassis
//     chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
//     chassis.setPose(50.513, -61.352, 90);     

//     // Rush mid mogo
//     chassis.follow(MidMogoRushR_txt, 10, 4000, false); // Rush mogo
//     chassis.waitUntilDone();

//     // Clamp mogo + Pull out
//     clampPiston.set_value(true);
//     pros::delay(400);
//     chassis.swingToHeading(340, lemlib::DriveSide::LEFT, 1000); // Pull out of center (20)
//     chassis.waitUntilDone();

//     // Get the rings on the ground
//     intakeMotors.move_velocity(600);
//     chassis.moveToPoint(11.293, -38.057, 500); // Part 1 intake ring
//     chassis.waitUntilDone();

//     // Turn to face wall (back facing wall)
//     chassis.turnToHeading(270, 700); // Turn to side (back facing wall)
//     pros::delay(400);
//     chassis.waitUntilDone();

//     // Move to wall
//     chassis.moveToPoint(49.195, -38.057, 1500, {.forwards = false}); // goto wall
//     chassis.waitUntilDone();

//     // Release goal and face goal 2 (back facing goal 2)
//     intakeMotors.move_velocity(0);
//     clampPiston.set_value(false);
//     pros::delay(400);
//     chassis.turnToPoint(23.04, -22.877, 700, {.forwards = false}); // Turn to face goal
//     chassis.waitUntilDone();

//     // Move to goal 2
//     chassis.moveToPoint(23.04, -22.877, 2500, {.forwards = false, .maxSpeed = 60}); // Move to goal
//     chassis.waitUntilDone();

//     // Clamp goal 2
//     clampPiston.set_value(true);
//     pros::delay(400);
//     chassis.waitUntilDone();

//     // Turn to mid 
//     chassis.turnToPoint(23.04, -9.478, 1000);
//     chassis.waitUntilDone();

//     // Move to mid 
//     intakeMotors.move_velocity(0);
//     chassis.moveToPoint(23.04, -9.478, 2000, {.forwards = true, .maxSpeed = 50});
//     chassis.waitUntilDone();
// }

// void autonomous(){
//     chassis.setPose(0, 0, 90);
//     chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

//     chassis.moveToPoint(24, 0, 4000);
// }

// 1 stake
void autonomous(){
    chassis.setPose(-51.132, 24, 270);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    chassis.moveToPoint(-30.789, 24, 3000, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntilDone();

    clampPiston.set_value(true);
    pros::delay(400);

    intakeMotors.move_velocity(600);
    chassis.turnToPoint(-23.427, 47.258, 500);
    chassis.waitUntilDone();

    chassis.moveToPoint(-23.427, 47.258, 2000, {.forwards = true, .maxSpeed = 80});
    chassis.waitUntilDone();

    chassis.turnToPoint(-23.427, 9.865, 1000);
    chassis.waitUntilDone();

    chassis.moveToPoint(-23.427, 9.865, 2000, {.forwards = true, .maxSpeed = 50});
    chassis.waitUntilDone();

    intakeMotors.move_velocity(0);
}

// void autonomous(){
//     chassis.setPose(51.132, 24, 90);
//     chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

//     chassis.moveToPoint(30.789, 24, 3000, {.forwards = false, .maxSpeed = 70});
//     chassis.waitUntilDone();

//     clampPiston.set_value(true);
//     pros::delay(400);

//     intakeMotors.move_velocity(600);
//     chassis.turnToPoint(23.427, 47.258, 500);
//     chassis.waitUntilDone();

//     chassis.moveToPoint(23.427, 47.258, 2000, {.forwards = true, .maxSpeed = 80});
//     chassis.waitUntilDone();

//     chassis.turnToPoint(23.427, 9.865, 1000);
//     chassis.waitUntilDone();

//     chassis.moveToPoint(23.427, 9.865, 2000, {.forwards = true, .maxSpeed = 50});
//     chassis.waitUntilDone();

//     intakeMotors.move_velocity(0);
// }

// void autonomous(){
//     chassis.setPose(51.132, -24, 90);
//     chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

//     chassis.moveToPoint(30.789, -24, 3000, {.forwards = false, .maxSpeed = 70});
//     chassis.waitUntilDone();

//     clampPiston.set_value(true);
//     pros::delay(400);

//     intakeMotors.move_velocity(600);
//     chassis.turnToPoint(23.427, -47.258, 500);
//     chassis.waitUntilDone();

//     chassis.moveToPoint(23.427, -47.258, 2000, {.forwards = true, .maxSpeed = 80});
//     chassis.waitUntilDone();

//     chassis.turnToPoint(23.427, -9.865, 1000);
//     chassis.waitUntilDone();

//     chassis.moveToPoint(23.427, -9.865, 2000, {.forwards = true, .maxSpeed = 50});
//     chassis.waitUntilDone();

//     intakeMotors.move_velocity(0);
// }

// void autonomous(){
//     chassis.setPose(-51.132, -24, 270);
//     chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

//     chassis.moveToPoint(-30.789, -24, 3000, {.forwards = false, .maxSpeed = 70});
//     chassis.waitUntilDone();

//     clampPiston.set_value(true);
//     pros::delay(400);

//     intakeMotors.move_velocity(600);
//     chassis.turnToPoint(-23.427, -47.258, 500);
//     chassis.waitUntilDone();

//     chassis.moveToPoint(-23.427, -47.258, 2000, {.forwards = true, .maxSpeed = 80});
//     chassis.waitUntilDone();

//     chassis.turnToPoint(-23.427, -9.865, 1000);
//     chassis.waitUntilDone();

//     chassis.moveToPoint(-23.427, -9.865, 2000, {.forwards = false, .maxSpeed = 50});
//     chassis.waitUntilDone();

//     intakeMotors.move_velocity(0);
// }


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


void hang(){
    static bool hangState = false;
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
        hangState = !hangState;
        hangPiston.set_value(hangState);
    }
}

void intake() {
    static bool r2_toggle = false;

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
        r2_toggle = !r2_toggle;
    }

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        intakeMotors.move_velocity(600);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        intakeMotors.move_velocity(-600);
    } else if (r2_toggle) {
        intakeMotors.move_velocity(600);
    } else {
        intakeMotors.move_velocity(0);
    }
}

void doungler_control(){
    static bool donger_state = true   ;
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        donger_state = !donger_state;
    }
    dongerPiston.set_value(donger_state);
}

void clamp_control() { 
    static bool clamp_state = false;
    
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        clamp_state = !clamp_state;
        clampPiston.set_value(clamp_state);
    }
}

void drive_control(){
    static bool isReversed = false;

    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            isReversed = !isReversed;
        }

        if (isReversed) {
            leftY = -leftY;
        }
        chassis.arcade(leftY, rightX);
};

void opcontrol() {
    // Wait until 1 minute and 30 seconds has passed
    // pros::Task wait_till_done([&](){
    //     pros::delay(95000);
    //     hangPiston.set_value(false);
    // });

    while (true) {
        
        drive_control();
        hang();
        clamp_control();
        intake(); 
        doungler_control();

        pros::delay(25);
    }
}