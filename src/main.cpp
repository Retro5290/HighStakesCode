#include "main.h"
#include "lemlib/api.hpp"
#include "config.hpp"

/*
    L2: LB down
    R2: LB up
    R1: INTAKE
    L1: OUTTAKE
    RIGHT: LB Toggle
    DOWN: Doinker
    LEFT: Hang
    UP: Intake Toggle
    B: Clamp
    Y: REVERSE DRIVE
    X: BRAKE MODE
*/

namespace controls {
    class Mechanisms {
    private:
        enum class LBToggleState {
            IDLE,
            INTAKE,
            CLEAR
        }; 

        static constexpr double LB_POSITIONS[] = {
            0.0,    // IDLE
            -250.0,  // INTAKE
            -750.0   // CLEAR
        };

    public:      

        static void update_LB() {
            /* 
            Mode        Motor 1     Motor 2
            Intake (2M) CCW         CW
            Outake (1M) CW          None
            Raise  (1M) None        CCW
            Hold   (1M) None        Hold

            Motor 1: Intake + Outake
            Motor 2: Raise + Intake
            */

            
        }

        static void update_intake() {

        }

        static void update_hang() {
            
            static bool hangState = false;
            if (robot::masterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
                hangState = !hangState;
                robot::mechanisms::hang.set_value(hangState);
            }
        }


        static void update_clamp() {
            static bool clampState = false;
            if (robot::masterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
                clampState = !clampState;
                robot::mechanisms::clamp.set_value(clampState);
            }
        }
        static void update_doinker() {
            static bool doinkerState = false;
            if (robot::masterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
                doinkerState = !doinkerState;
                robot::mechanisms::doinker.set_value(doinkerState);
            }
        }

        static void drive(){
            static bool reverseDrive = false;
            static bool brakeMode = false;

            int x = robot::masterController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            int y = robot::masterController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
            if (robot::masterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
                reverseDrive = !reverseDrive;
            }
            if (reverseDrive) {
                x = -x;
            }
            if (robot::masterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
                brakeMode = !brakeMode;
                if (brakeMode) {
                    pros::delay(500);
                    robot::drivetrain::chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
                } else {
                    pros::delay(500);
                    robot::drivetrain::chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
                }
            }

            robot::drivetrain::chassis.arcade(x, y);
        }
    };
} 
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    
    robot::drivetrain::chassis.calibrate(); // calibrate sensors
    robot::mechanisms::lbMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    robot::drivetrain::chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {

            double averageAngle = (robot::mechanisms::lbMotors.get_position(0) + robot::mechanisms::lbMotors.get_position(1)) / 2;

            // print robot location to the brain screen 
            pros::lcd::print(0, "X: %f", robot::drivetrain::chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", robot::drivetrain::chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", robot::drivetrain::chassis.getPose().theta); // heading
            // if (robot::mechanisms::opticalSensor.get_hue() > 100 && robot::mechanisms::opticalSensor.get_hue() < 220) {
            //     pros::lcd::print(3, "Color: Blue");
            // } else if (robot::mechanisms::opticalSensor.get_hue() > 0 && robot::mechanisms::opticalSensor.get_hue() < 25) {
            //     pros::lcd::print(3, "Color: Red");
            // } else {
            //     pros::lcd::print(3, "Color: None");
            // }
            // pros::lcd::print(4, "Hue: %f", robot::mechanisms::opticalSensor.get_hue());
            pros::lcd::print(3, "lbAngleAvg: %f", averageAngle);
            pros::lcd::print(4, "lbBrakeMode: %d", robot::mechanisms::lbMotors.get_brake_mode());
            pros::lcd::print(5, "lbLimitSwitch: %d", robot::mechanisms::lbLimitSwitch.get_value());
            // delay to save resources
            pros::delay(robot::constants::LOOP_DELAY);
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
    robot::drivetrain::chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    while (true) {
        controls::Mechanisms::drive();
        controls::Mechanisms::update_hang();
        controls::Mechanisms::update_intake();      
        controls::Mechanisms::update_clamp();
        controls::Mechanisms::update_LB();
        controls::Mechanisms::update_doinker();
        // ... other updates ...
        pros::delay(robot::constants::LOOP_DELAY);
    }
}