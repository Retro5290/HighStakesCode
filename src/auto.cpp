#include "main.h"
#include "config.hpp"
#include "auto.h"
#include "lemlib/timer.hpp"

enum class AutonomousMode {
    SKILLS,
    RED_RING,
    RED_STAKE,
    BLUE_RING,
    BLUE_STAKE,      
    TEST
};

// Current autonomous selection
static AutonomousMode current_auto = AutonomousMode::BLUE_STAKE;

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
        static uint32_t runSpeed;
    };

    // Tunable constants
    constexpr bool ENABLE_COLOR_SORT = false;  // Set to true to enable color sorting/ejection
    constexpr int EJECT_TIME = 500;   // Time to stop intake for ejection (ms)
    constexpr int RING_TRAVEL_TIME = 200; // Time for ring to reach top after detection (ms)
    constexpr int RING_EJECT_COOLDOWN = 1000; // Time to wait before detecting another ring (ms)

    // Ring eject state variables
    bool IntakeState::targetColor = (current_auto == AutonomousMode::BLUE_RING ||
    current_auto == AutonomousMode::BLUE_STAKE); // false = red team (eject blue), true = blue team (eject red)
    bool IntakeState::isEjecting = false;
    uint32_t IntakeState::ejectStartTime = 0;
    uint32_t IntakeState::ringDetectedTime = 0;
    uint32_t IntakeState::ringEjectCooldown = 0;
    bool IntakeState::ringDetected = false;
    uint32_t IntakeState::runSpeed = robot::constants::INTAKE_SPEED;

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
                        robot::mechanisms::intakeMotor.move_velocity(-IntakeState::runSpeed);
                    } else {
                        // Normal operation
                        robot::mechanisms::intakeMotor.move_velocity(-IntakeState::runSpeed);

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
                    robot::mechanisms::intakeMotor.move_velocity(-IntakeState::runSpeed);
                }
            } else {
                robot::mechanisms::intakeMotor.move_velocity(0);
                IntakeState::shouldRun = false;
                IntakeState::isEjecting = false;
                IntakeState::ringDetected = false;
                IntakeState::ringEjectCooldown = 0;
                IntakeState::runSpeed = robot::constants::INTAKE_SPEED;
            }
            pros::delay(10);
        }
    }
  
    void run_intake(int runTime, uint32_t intakeSpeed = robot::constants::INTAKE_SPEED) {
        // Reset all intake state variables
        IntakeState::startTime = pros::millis();
        IntakeState::duration = runTime;
        IntakeState::shouldRun = true;
        IntakeState::isEjecting = false;
        IntakeState::ringDetected = false;
        IntakeState::ringEjectCooldown = 0;
        IntakeState::runSpeed = intakeSpeed;
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

void red_ring_auto() {
    try {
        robot::drivetrain::chassis.setPose(-58.688, 12.771, 0);
        robot::drivetrain::chassis.moveToPoint(-58.688, 0, 1000, {.forwards = false});
        robot::drivetrain::chassis.turnToHeading(90, 1000);
        robot::drivetrain::chassis.waitUntilDone();

        autosetting::run_intake(1000);
        robot::mechanisms::intake.set_value(true);
        pros::delay(700);

        // Score alliance stake
        robot::drivetrain::chassis.moveToPoint(-52.682, 0, 1000, {.forwards = true});
        autosetting::run_intake(1000);
        pros::delay(500);
        
        // Swing to Mogo
        robot::drivetrain::chassis.swingToPoint(-28.465, 20.327, lemlib::DriveSide::LEFT, 1000, 
        {.forwards = false, .minSpeed = 127, .earlyExitRange = 20});

        robot::drivetrain::chassis.moveToPose(-28.465, 20.327, 230, 2000, 
        {.forwards = false, .minSpeed = 127, .earlyExitRange = 20}); // need tune
        robot::mechanisms::intake.set_value(false);
        robot::drivetrain::chassis.moveToPoint(-28.465, 20.327, 1000, {.forwards = false, .maxSpeed = 60});
        robot::drivetrain::chassis.waitUntil(20); // need tune
        robot::mechanisms::clamp.set_value(true);
        
        // Moving towards mid
        robot::drivetrain::chassis.turnToPoint(-10.64, 37.57, 1000, {.minSpeed = 127, .earlyExitRange = 20});
        robot::drivetrain::chassis.moveToPose(-10.64, 37.57, 40, 2000, {.minSpeed = 127, .earlyExitRange = 22});
        robot::drivetrain::chassis.waitUntilDone();
        autosetting::run_intake(5000);
        robot::drivetrain::chassis.moveToPose(-10.64, 37.57, 40, 1000, {.maxSpeed = 80}); // Score first ring
        robot::drivetrain::chassis.moveToPose(-9.478, 42.995, 20, 1000, {.maxSpeed = 80});  // Score second ring
        robot::drivetrain::chassis.moveToPoint(-15.096, 33.114, 1000, {.forwards = false}); // Fall back

        // Starting third ring
        robot::drivetrain::chassis.turnToPoint(-21.49, 43.189, 1000, {.minSpeed = 127, .earlyExitRange = 20});
        autosetting::run_intake(2000);
        robot::drivetrain::chassis.moveToPoint(-21.49, 43.189, 1500); // Score third ring

        // Corner 
        robot::drivetrain::chassis.turnToPoint(-57.913, 59.851, 1000, {.minSpeed = 127, .earlyExitRange = 20});
        robot::drivetrain::chassis.moveToPoint(-57.913, 59.851, 1500);
        robot::drivetrain::chassis.waitUntil(22); // need tune
        autosetting::run_intake(2000);

        // ------------------------------------- Mid Ladder -------------------------------------
        robot::drivetrain::chassis.turnToPoint(-28.658, 8.122, 1000, {.minSpeed = 127, .earlyExitRange = 20});
        robot::drivetrain::chassis.moveToPoint(-28.658, 8.122, 1500);
        robot::drivetrain::chassis.waitUntilDone();
        robot::mechanisms::doinker.set_value(true);

    } catch (const std::exception& e) {
        pros::lcd::print(0, "Red Auto Error: %s", e.what());
    }
}
/*
          0 
     270     90
         180
*/

void red_stake_auto() {
    try {

    } catch (const std::exception& e) {
        pros::lcd::print(0, "Two Stake Red Auto Error: %s", e.what());
    }
}
/*
          0
     270     90
         180
*/
void blue_ring_auto() {
    try {

    } catch (const std::exception& e) {
        pros::lcd::print(0, "One Stake Blue Auto Error: %s", e.what());
                        }
                    }
/*
          0
     270     90
         180
*///55
void blue_stake_auto() {
    try {
 
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
        case AutonomousMode::RED_RING:
            red_ring_auto();
            break;
        case AutonomousMode::RED_STAKE:
            red_stake_auto();
            break;
        case AutonomousMode::BLUE_RING:
            blue_ring_auto();
            break;
        case AutonomousMode::BLUE_STAKE:
            blue_stake_auto();
            break;
        case AutonomousMode::TEST:
            test_auto();
            break;
    }
}

