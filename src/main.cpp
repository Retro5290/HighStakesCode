#include "main.h"
#include "lemlib/api.hpp"
#include "config.hpp"

namespace controls {
    class Mechanisms {
    private:
        enum class LBToggleState {
            IDLE,
            INTAKE1,
            INTAKE2
        };

        static constexpr double LB_POSITIONS[] = {
            0.0,    // IDLE
            -200.0,  // INTAKE1
            -400.0   // INTAKE2
        };

        static constexpr double LB_BOUNARIES[] = {
            400, // Below this is INTAKE1
            800  // Below this is INTAKE2
            // Rest is Idle
        };

    public:
        static void drive(){
            int x = robot::masterController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            int y = robot::masterController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
            robot::drivetrain::chassis.arcade(x, y);
        }
        static void update_hang() {
            static bool hangState = false;
            if (robot::masterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
                hangState = !hangState;
                robot::mechanisms::hang.set_value(hangState);
            }
        }
        static void update_LB(){
            static LBToggleState lbState = LBToggleState::IDLE;
            static bool isAutoMoving = false;

            int leftLBPosition = robot::mechanisms::lbMotors.get_position(0);
            int rightLBPosition = robot::mechanisms::lbMotors.get_position(1);
            int averageLBPosition = abs((leftLBPosition + rightLBPosition) / 2);

            if (robot::masterController.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                robot::mechanisms::lbMotors.move_velocity(100);
                isAutoMoving = false;
            } else if (robot::masterController.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                isAutoMoving = false;
                robot::mechanisms::lbMotors.move_velocity(-100);
            } else if (!isAutoMoving) {
                robot::mechanisms::lbMotors.move_velocity(0);
            }

            if (robot::mechanisms::lbLimitSwitch.get_value()) {
                lbState = LBToggleState::IDLE;
                robot::mechanisms::lbMotors.tare_position();
                isAutoMoving = false;
            }

            if (robot::masterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
                switch (lbState) {
                    case LBToggleState::IDLE:
                        lbState = LBToggleState::INTAKE1;
                        isAutoMoving = true;
                        break;
                    case LBToggleState::INTAKE1:
                        lbState = LBToggleState::INTAKE2;
                        isAutoMoving = true;
                        break;
                    case LBToggleState::INTAKE2:
                        break;
                    default:
                        if (averageLBPosition < LB_BOUNARIES[0]) {
                            lbState = LBToggleState::INTAKE1;
                            isAutoMoving = true;    
                        } else if (averageLBPosition < LB_BOUNARIES[1]) {
                            lbState = LBToggleState::INTAKE2;
                            isAutoMoving = true;
                        } else {
                            lbState = LBToggleState::IDLE;
                            isAutoMoving = true;
                        }
                        break;
                }
                
            }

            if (isAutoMoving) {
                if (lbState == LBToggleState::IDLE) {
                    robot::mechanisms::lbMotors.move_velocity(200);
                } else {
                    robot::mechanisms::lbMotors.move_absolute(LB_POSITIONS[static_cast<int>(lbState)], 200);
                    if (averageLBPosition == LB_POSITIONS[static_cast<int>(lbState)]) {
                        isAutoMoving = false;
                    }
                }
            }
        }

        static void update_intake() {
            static bool r2Toggle = false;
            if (robot::masterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
                r2Toggle = !r2Toggle;
            }

            const int intake_speed = robot::constants::INTAKE_SPEED;
            if (robot::masterController.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                robot::mechanisms::intakeMotor.move_velocity(intake_speed);
            } else if (robot::masterController.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                robot::mechanisms::intakeMotor.move_velocity(-intake_speed);
            } else if (r2Toggle) {
                robot::mechanisms::intakeMotor.move_velocity(intake_speed);
            } else {
                robot::mechanisms::intakeMotor.move_velocity(0);
            }
        }
        static void update_clamp() {
            static bool clampState = false;
            if (robot::masterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
                clampState = !clampState;
                robot::mechanisms::clamp.set_value(clampState);
            }
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
    robot::mechanisms::lbMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {

            double averageAngle = (robot::mechanisms::lbMotors.get_position(0) + robot::mechanisms::lbMotors.get_position(1)) / 2;

            // print robot location to the brain screen 
            pros::lcd::print(0, "X: %f", robot::drivetrain::chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", robot::drivetrain::chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", robot::drivetrain::chassis.getPose().theta); // heading
            pros::lcd::print(3, "lbAngle1: %f", robot::mechanisms::lbMotors.get_position(0));
            pros::lcd::print(4, "lbAngle2: %f", robot::mechanisms::lbMotors.get_position(1));
            pros::lcd::print(5, "lbAngleAvg: %f", averageAngle);
            pros::lcd::print(6, "lbLimitSwitch: %d", robot::mechanisms::lbLimitSwitch.get_value());
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
    while (true) {
        // controls::Mechanisms::drive();
        // controls::Mechanisms::update_hang();
        // controls::Mechanisms::update_intake();
        // controls::Mechanisms::update_clamp();
        controls::Mechanisms::update_LB();
        // ... other updates ...
        pros::delay(robot::constants::LOOP_DELAY);
    }
}