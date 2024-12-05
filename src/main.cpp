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
            -340.0,  // INTAKE
            -750.0   // CLEAR
        };

        static constexpr int LB_POSITION_LOSS_BOUNDARY = 725;
        static constexpr int LB_FINETUNE_BOUNDARY = 300;

        static constexpr double MIN_VELOCITY = 50;  
        static constexpr double SLEW_RATE = 100;

        static double slewMove(double targetVelocity, double currentVelocity) {
            if (targetVelocity == currentVelocity) {
                return targetVelocity;
            }
            if (targetVelocity == 0) {
                // Slowing down
                if (std::abs(currentVelocity) < MIN_VELOCITY) {
                    return 0;
                }
                if (currentVelocity > 0) {
                    return currentVelocity - SLEW_RATE;
                }
                if (currentVelocity < 0) {
                    return currentVelocity + SLEW_RATE;
                }
            }
        
            // Speeding up or maintaining speed
            if (currentVelocity < targetVelocity) {
                return std::min(currentVelocity + SLEW_RATE, targetVelocity);
            }
            if (currentVelocity > targetVelocity) {
                return std::max(currentVelocity - SLEW_RATE, targetVelocity);
            }
            
            return targetVelocity;
        }

    public:      
        static void drive(){
            static bool reverseDrive = false;

            int x = robot::masterController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            int y = robot::masterController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
            if (robot::masterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
                reverseDrive = !reverseDrive;
            }
            if (reverseDrive) {
                x = -x;
            }
            robot::drivetrain::chassis.arcade(x, y);
        }
        static void update_hang() {
            
            static bool hangState = false;
            if (robot::masterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
                hangState = !hangState;
                robot::mechanisms::hang.set_value(hangState);
            }
        }

        static void update_LB(){
            // Static Variables
            static LBToggleState lbState = LBToggleState::IDLE;
            static bool isAutoMoving = false;
            static bool isOutOfBounds = false;

            // Position Variables
            int leftLBPosition = robot::mechanisms::lbMotors.get_position(0);
            int rightLBPosition = robot::mechanisms::lbMotors.get_position(1);
            int averageLBPosition = std::abs((leftLBPosition + rightLBPosition) / 2);

            // Manual Speed Variables
            int manualSpeed = (averageLBPosition < LB_FINETUNE_BOUNDARY) ? 30 : 150;

            // Manual Movement
            if (robot::masterController.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                robot::mechanisms::lbMotors.move_velocity(-manualSpeed);
                isOutOfBounds = true;
                isAutoMoving = false;

            } else if (robot::masterController.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                robot::mechanisms::lbMotors.move_velocity(manualSpeed);
                isOutOfBounds = true;
                isAutoMoving = false;

            } else if (!isAutoMoving) {
                double currentVelocity = robot::mechanisms::lbMotors.get_actual_velocity(0);
                currentVelocity = slewMove(0, currentVelocity);
                robot::mechanisms::lbMotors.move_velocity(currentVelocity);
            }

            // Limit Switch Interrupt
            if (robot::mechanisms::lbLimitSwitch.get_value()) {
                isOutOfBounds = false;
                robot::mechanisms::lbMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

                // Reset Auto Movement
                if (isAutoMoving && lbState == LBToggleState::IDLE) {
                    robot::mechanisms::lbMotors.tare_position(0);
                    robot::mechanisms::lbMotors.tare_position(1);
                    isAutoMoving = false;
                }

                // User Interrupt
                if (!isAutoMoving) {
                    robot::mechanisms::lbMotors.tare_position(0);
                    robot::mechanisms::lbMotors.tare_position(1);
                    lbState = LBToggleState::IDLE;
                }
            } else {
                robot::mechanisms::lbMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            }

            // Toggle Auto Movement
            if (robot::masterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {

                // After user interrupts
                if (isOutOfBounds) {
                    if (averageLBPosition < LB_POSITION_LOSS_BOUNDARY) {
                        lbState = LBToggleState::INTAKE;
                        isAutoMoving = true;  
                        isOutOfBounds = false;
                    } else {
                        lbState = LBToggleState::IDLE;
                        isAutoMoving = true;
                        isOutOfBounds = false;
                    }

                } else {
                    switch (lbState) {
                    case LBToggleState::IDLE:
                        lbState = LBToggleState::INTAKE;
                        isAutoMoving = true;
                        break;
                    case LBToggleState::INTAKE:
                        lbState = LBToggleState::CLEAR;
                        isAutoMoving = true;
                        break;
                    case LBToggleState::CLEAR:
                        lbState = LBToggleState::CLEAR;
                        isAutoMoving = true;
                        break;
                    }
                }
            }

            // Auto Movement
            if (isAutoMoving) {
                if (lbState == LBToggleState::IDLE) {
                    robot::mechanisms::lbMotors.move_velocity(200);
                } else {
                    robot::mechanisms::lbMotors.move_absolute(LB_POSITIONS[static_cast<int>(lbState)], 150);
                    
                    double targetPosition = LB_POSITIONS[static_cast<int>(lbState)];
                    
                    if (std::abs(averageLBPosition - targetPosition) < 15) {  
                        isAutoMoving = false;
                        if (!isOutOfBounds) {
                            robot::mechanisms::lbMotors.move_velocity(0);
                        }
                    }
                }
            }
            
            std::cout<< "Target velocity: " << robot::mechanisms::lbMotors.get_target_velocity() << std::endl;
            std::cout<< "Current velocity: " << robot::mechanisms::lbMotors.get_actual_velocity() << std::endl;
            std::cout<< "Motor 1 Power: " << robot::mechanisms::lbMotors.get_power(0) <<" watts" << std::endl;
            std::cout<< "Motor 2 Power: " << robot::mechanisms::lbMotors.get_power(1) <<" watts" << std::endl;
            std::cout<< "Average Position: " << averageLBPosition << std::endl;
            std::cout<< "----------------------------------------" << std::endl;
            std::cout<< std::endl;
        }
        static void update_intake() {
            static bool intakeToggle = false;
            if (robot::masterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
                intakeToggle = !intakeToggle;
            }

            const int intake_speed = robot::constants::INTAKE_SPEED;
            if (robot::masterController.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                robot::mechanisms::intakeMotor.move_velocity(-intake_speed);
            } else if (robot::masterController.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                robot::mechanisms::intakeMotor.move_velocity(intake_speed);
            } else if (intakeToggle) {
                robot::mechanisms::intakeMotor.move_velocity(-intake_speed);
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
            pros::lcd::print(3, "Hue: %f", robot::mechanisms::opticalSensor.get_hue());
            // pros::lcd::print(3, "lbAngleAvg: %f", averageAngle);
            // pros::lcd::print(4, "lbBrakeMode: %d", robot::mechanisms::lbMotors.get_brake_mode());
            // pros::lcd::print(5, "lbLimitSwitch: %d", robot::mechanisms::lbLimitSwitch.get_value());
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
        controls::Mechanisms::drive();
        controls::Mechanisms::update_hang();
        controls::Mechanisms::update_intake();      
        controls::Mechanisms::update_clamp();
        controls::Mechanisms::update_LB();
        // ... other updates ...
        pros::delay(robot::constants::LOOP_DELAY);
    }
}