#include "main.h"
#include "config.hpp"
#include "auto.h"
#include "lemlib/timer.hpp"

enum class AutonomousMode {
    SKILLS,
    ONE_STAKE_RED,
    TWO_STAKE_RED,
    ONE_STAKE_BLUE,
    TWO_STAKE_BLUE,      
    TEST
};

// Current autonomous selection
static AutonomousMode current_auto = AutonomousMode::TWO_STAKE_BLUE;

namespace autosetting {
    struct IntakeState {
        // Ring Eject States
        static bool targetColor;
        static bool isEjecting;
        static bool ringDetected;
        static uint32_t ejectStartTime;
        static uint32_t ringEjectCooldown;
        static uint32_t ringDetectedTime;

        // Run Intake States
        static bool shouldRun;
        static uint32_t startTime;
        static uint32_t duration;
    };

    // Tunable constants
    constexpr bool ENABLE_COLOR_SORT = false;  // Set to true to enable color sorting/ejection
    constexpr int EJECT_TIME = 500;   // Time to stop intake for ejection (ms)
    constexpr int RING_TRAVEL_TIME = 200; // Time for ring to reach top after detection (ms)
    constexpr int RING_EJECT_COOLDOWN = 1000; // Time to wait before detecting another ring (ms)

    // Ring eject state variables
    bool IntakeState::targetColor = (current_auto == AutonomousMode::ONE_STAKE_BLUE ||
    current_auto == AutonomousMode::TWO_STAKE_BLUE); // false = red team (eject blue), true = blue team (eject red)
    bool IntakeState::isEjecting = false;
    uint32_t IntakeState::ejectStartTime = 0;
    uint32_t IntakeState::ringDetectedTime = 0;
    uint32_t IntakeState::ringEjectCooldown = 0;
    bool IntakeState::ringDetected = false;

    // Run intake state variables
    bool IntakeState::shouldRun = false;
    uint32_t IntakeState::startTime = 0;
    uint32_t IntakeState::duration = 0;

    void intake_task_fn(void* param) {
        while (pros::competition::is_autonomous()) {
            uint32_t currentTime = pros::millis();
            
            // Check if run intake is called
            if (IntakeState::shouldRun && 
                (currentTime - IntakeState::startTime < IntakeState::duration)) {
                
                // Only check for color sorting if enabled
                if (ENABLE_COLOR_SORT) {
                    if (IntakeState::isEjecting) {
                        if (currentTime - IntakeState::ejectStartTime < EJECT_TIME) {
                            // During eject time, stop the intake
                            robot::mechanisms::intakeMotor.move_velocity(0);
                        } else {
                            // Eject time finished
                            IntakeState::isEjecting = false;
                            IntakeState::ringDetected = false;
                        }
                    } else if (IntakeState::ringDetected) {
                        // Ring detected but waiting for it to reach top
                        if (currentTime - IntakeState::ringDetectedTime >= RING_TRAVEL_TIME) {
                            // Ring should be at top now, start ejection
                            IntakeState::isEjecting = true;    
                            IntakeState::ejectStartTime = currentTime;
                            IntakeState::ringDetected = false;
                        }
                        robot::mechanisms::intakeMotor.move_velocity(-robot::constants::INTAKE_SPEED);
                    } else {
                        // Normal operation
                        robot::mechanisms::intakeMotor.move_velocity(-robot::constants::INTAKE_SPEED);

                        // Check for wrong color ring
                        if (IntakeState::targetColor) { // Looking for red rings
                            if (robot::mechanisms::opticalSensor.get_hue() >= 0 && 
                                robot::mechanisms::opticalSensor.get_hue() <= 25 &&
                                currentTime >= IntakeState::ringEjectCooldown) {
                                // Red ring detected, start travel time countdown
                                IntakeState::ringDetected = true;
                                IntakeState::ringDetectedTime = currentTime;
                                IntakeState::ringEjectCooldown = currentTime + RING_EJECT_COOLDOWN;
                            }
                        } else { // Looking for blue rings
                            if (robot::mechanisms::opticalSensor.get_hue() >= 100 && 
                                robot::mechanisms::opticalSensor.get_hue() <= 220 &&
                                currentTime >= IntakeState::ringEjectCooldown) {
                                // Blue ring detected, start travel time countdown
                                IntakeState::ringDetected = true;
                                IntakeState::ringDetectedTime = currentTime;
                                IntakeState::ringEjectCooldown = currentTime + RING_EJECT_COOLDOWN;
                            }
                        }
                    }
                } else {
                    // Simply run intake when color sort is disabled
                    robot::mechanisms::intakeMotor.move_velocity(-robot::constants::INTAKE_SPEED);
                }
            } else {
                robot::mechanisms::intakeMotor.move_velocity(0);
                IntakeState::shouldRun = false;
                IntakeState::isEjecting = false;
                IntakeState::ringDetected = false;
                IntakeState::ringEjectCooldown = 0;
            }
            pros::delay(10);
        }
    }
  
    void run_intake(int runTime) {
        // Reset all intake state variables
        IntakeState::startTime = pros::millis();
        IntakeState::duration = runTime;
        IntakeState::shouldRun = true;
        IntakeState::isEjecting = false;
        IntakeState::ringDetected = false;
        IntakeState::ringEjectCooldown = 0;
    }
}
/*
          0
     270     90
         180
*/
void skills_auto() {
    try {
    } catch (const std::exception& e) {
        pros::lcd::print(0, "Skills Auto Error: %s", e.what());
    };
}
/*
          0
     270     90
         180
*/
ASSET(RedSideRush_txt);
ASSET(RedSideBack_txt);
ASSET(RedSideGoalLineup_txt);
void one_stake_red_auto() {
    try {
        // Move to the mid goal
        robot::drivetrain::chassis.setPose(-50.551, -59.301, 90);
        robot::drivetrain::chassis.follow(RedSideRush_txt, 12, 3000); 
        robot::drivetrain::chassis.waitUntil(30);
        robot::mechanisms::doinker.set_value(true); // Doink the goal
        pros::delay(500);
        robot::drivetrain::chassis.follow(RedSideBack_txt, 12, 1500, false);
        robot::drivetrain::chassis.waitUntilDone();
        
        robot::mechanisms::doinker.set_value(false);
        pros::delay(500);
        robot::drivetrain::chassis.turnToHeading(250, 1000, {.maxSpeed = 100});
        robot::drivetrain::chassis.follow(RedSideGoalLineup_txt, 12, 1500, false);
        robot::drivetrain::chassis.waitUntilDone();
        robot::mechanisms::clamp.set_value(false);
        pros::delay(500);
        autosetting::run_intake(2500);
        robot::drivetrain::chassis.swingToPoint(-19.165, -43.026, lemlib::DriveSide::RIGHT, 1000, {.maxSpeed = 80});
 
        // // Pick up rings
        robot::drivetrain::chassis.moveToPoint(-21.102, -39.733, 500, {.maxSpeed = 80});

        robot::drivetrain::chassis.turnToPoint(-53.264, -38.958, 500, {.forwards = false, .minSpeed = 127, .earlyExitRange = 10});
        robot::drivetrain::chassis.moveToPoint(-53.264, -38.958, 2000, {.forwards = false});
        robot::drivetrain::chassis.waitUntilDone();

        // // Drop off stake
        robot::mechanisms::clamp.set_value(true);
        pros::delay(500);
        robot::drivetrain::chassis.turnToPoint(-23.04, -23.652, 800, {.forwards = false});
        robot::drivetrain::chassis.moveToPoint(-23.047, -23.652, 1500, {.forwards = false, .maxSpeed = 80});
        robot::drivetrain::chassis.waitUntilDone();
        robot::mechanisms::clamp.set_value(false);
        pros::delay(500);
        autosetting::run_intake(2500);
        robot::mechanisms::lbMotors.move_absolute(-1300, 150);
        robot::drivetrain::chassis.turnToPoint(-14.128, -12.028, 1000);
        robot::drivetrain::chassis.moveToPoint(-14.128, -12.028, 1500, {.maxSpeed = 60});
        robot::drivetrain::chassis.waitUntilDone();
        pros::delay(500);
    } catch (const std::exception& e) {
        pros::lcd::print(0, "Red Auto Error: %s", e.what());
    }
}
/*
          0 
     270     90
         180
*/

ASSET(RedSideOneIntake_txt);
ASSET(RedSideTwoRing_txt);
void two_stake_red_auto() {
    try {
        robot::drivetrain::chassis.setPose(-58.332, 46.676, 305);
        robot::drivetrain::chassis.moveToPoint(-25.783, 24.59, 4000, {.forwards = false, .maxSpeed = 60});
        robot::drivetrain::chassis.waitUntilDone();
        robot::mechanisms::clamp.set_value(false);
        pros::delay(500);
        autosetting::run_intake(10500);
        robot::drivetrain::chassis.turnToPoint(-24.233, 40.864, 1000);
        robot::drivetrain::chassis.moveToPoint(-24.233, 40.864, 4000);
        robot::drivetrain::chassis.waitUntilDone();
        pros::delay(1000);
        robot::drivetrain::chassis.turnToPoint(-25.008, 8.315, 1000, {.maxSpeed = 60});
        robot::drivetrain::chassis.moveToPoint(-25.008, 8.315, 4000);
        pros::delay(1000);     
        robot::mechanisms::doinker.set_value(true);
    } catch (const std::exception& e) {
        pros::lcd::print(0, "Two Stake Red Auto Error: %s", e.what());
    }
}
/*
          0
     270     90
         180
*/
ASSET(BlueSideRush_txt);
ASSET(BlueSideBack_txt);
ASSET(BlueSideGoalLineup_txt);
void one_stake_blue_auto() {
    try {
        robot::drivetrain::chassis.setPose(50.551, -59.301, 270);
        robot::drivetrain::chassis.moveToPose(9.896, -49.613, 300, 2000);
        robot::drivetrain::chassis.waitUntilDone();
        robot::mechanisms::doinker.set_value(true); // Doink the goal

        robot::drivetrain::chassis.moveToPose(34.396, -59.494, 270, 3000, {.forwards = false});
        robot::drivetrain::chassis.waitUntilDone();
        
        robot::mechanisms::doinker.set_value(false);
        pros::delay(500);
        robot::drivetrain::chassis.turnToHeading(110, 1000);
        robot::drivetrain::chassis.follow(BlueSideGoalLineup_txt, 12, 1500, false);
        robot::drivetrain::chassis.waitUntilDone();
        robot::mechanisms::clamp.set_value(false);
        pros::delay(500);
        autosetting::run_intake(2500);
        robot::drivetrain::chassis.swingToPoint(19.165, -43.026, lemlib::DriveSide::RIGHT, 1000, {.maxSpeed = 80});
 
        // // Pick up rings
        robot::drivetrain::chassis.moveToPoint(21.102, -39.733, 500, {.maxSpeed = 80});

        robot::drivetrain::chassis.turnToPoint(53.264, -38.958, 500, {.forwards = false, .minSpeed = 127, .earlyExitRange = 10});
        robot::drivetrain::chassis.moveToPoint(53.264, -38.958, 2000, {.forwards = false});
        robot::drivetrain::chassis.waitUntilDone();

        // // Drop off stake
        robot::mechanisms::clamp.set_value(true);
        pros::delay(500);
        robot::drivetrain::chassis.turnToPoint(23.04, -23.652, 800, {.forwards = false});
        robot::drivetrain::chassis.moveToPoint(23.047, -23.652, 1500, {.forwards = false, .maxSpeed = 80});
        robot::drivetrain::chassis.waitUntilDone();
        robot::mechanisms::clamp.set_value(false);
        pros::delay(500);
        autosetting::run_intake(2500);
        robot::mechanisms::lbMotors.move_absolute(-1300, 150);
        robot::drivetrain::chassis.turnToPoint(14.128, -12.028, 1000);
        robot::drivetrain::chassis.moveToPoint(14.128, -12.028, 1500, {.maxSpeed = 60});
        robot::drivetrain::chassis.waitUntilDone();
        pros::delay(500);
    } catch (const std::exception& e) {
        pros::lcd::print(0, "One Stake Blue Auto Error: %s", e.what());
                        }
                    }
/*
          0
     270     90
         180
*///55
void two_stake_blue_auto() {
    try {
        robot::drivetrain::chassis.setPose(58.332, 46.676, 55);
        robot::drivetrain::chassis.moveToPoint(25.783, 24.59, 4000, {.forwards = false, .maxSpeed = 60});
        robot::drivetrain::chassis.waitUntilDone();
        robot::mechanisms::clamp.set_value(false);
        pros::delay(500);
        autosetting::run_intake(10500);
        robot::drivetrain::chassis.turnToPoint(24.233, 40.864, 1000);
        robot::drivetrain::chassis.moveToPoint(24.233, 40.864, 4000);
        robot::drivetrain::chassis.waitUntilDone();
        pros::delay(1000);
        robot::drivetrain::chassis.turnToPoint(25.008, 8.315, 1000, {.maxSpeed = 60});
        robot::drivetrain::chassis.moveToPoint(25.008, 8.315, 4000);
        pros::delay(1000);     
        robot::mechanisms::doinker.set_value(true);  
    } catch (const std::exception& e) {
        pros::lcd::print(0, "Two Stake Blue Auto Error: %s", e.what());
    }
}
/*
          0
     270     90
         180
*/
void test_auto() {
    robot::drivetrain::chassis.setPose(0, 0, 90);
    robot::drivetrain::chassis.moveToPoint(30, 0, 10000);
}

void autonomous() {
    robot::mechanisms::intakeMotor.move_velocity(200);
    std::cout << "Running Auto" << std::endl;
    // Create task at start of autonomous
    pros::Task intake_task(autosetting::intake_task_fn, nullptr, "Intake Task");
    
    // Your existing autonomous code
    switch (current_auto) {
        case AutonomousMode::SKILLS:
            skills_auto();
            break;
        case AutonomousMode::ONE_STAKE_RED:
            one_stake_red_auto();
            break;
        case AutonomousMode::TWO_STAKE_RED:
            two_stake_red_auto();
            break;
        case AutonomousMode::ONE_STAKE_BLUE:
            one_stake_blue_auto();
            break;
        case AutonomousMode::TWO_STAKE_BLUE:
            two_stake_blue_auto();
            break;
        case AutonomousMode::TEST:
            test_auto();
            break;
    }
}

