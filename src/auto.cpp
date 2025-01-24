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
static AutonomousMode current_auto = AutonomousMode::RED_STAKE;

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

    void driveForward(double inches, float speed = 100, float timeout = 10000) {
        // Get current position
        lemlib::Pose current = robot::drivetrain::chassis.getPose();
        
        // Calculate target point in front of current position
        // At 0 degrees, forward is positive Y
        // At 90 degrees, forward is positive X
        double angleRad = current.theta * M_PI / 180.0; // Convert degrees to radians
        double targetX = current.x + inches * sin(angleRad); // use sin for X
        double targetY = current.y + inches * cos(angleRad); // use cos for Y
        
        // Move to the calculated point
        robot::drivetrain::chassis.moveToPose(targetX, targetY, current.theta, timeout, {
            .forwards = (inches > 0),  // drive forwards if inches is positive
            .maxSpeed = float(speed),    // adjust speed as needed
        });
        
        robot::drivetrain::chassis.waitUntilDone();
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
        robot::drivetrain::chassis.setPose(-59.044, 10.494, 90);
        robot::drivetrain::chassis.moveToPoint(-59.044, 0, 1000);
        robot::drivetrain::chassis.turnToHeading(180, 1000);
        robot::drivetrain::chassis.waitUntilDone();
        robot::mechanisms::clamp.set_value(true);

    } catch (const std::exception& e) {
        pros::lcd::print(0, "Red Auto Error: %s", e.what());
    }
}
/*
          0 
     270     90
         180
*/
ASSET(RedStakeRush_txt)
ASSET(RedStakeReturn_txt)
void red_stake_auto() {
    try {
        robot::drivetrain::chassis.setPose(-52.053, -59.611, 90);
        robot::drivetrain::chassis.follow(RedStakeRush_txt, 10, 10000);
        robot::drivetrain::chassis.waitUntilDone();
        robot::mechanisms::doinker.set_value(true);
        pros::delay(500);
        robot::drivetrain::chassis.follow(RedStakeReturn_txt, 10, 10000, false);
        robot::drivetrain::chassis.waitUntilDone();
        robot::mechanisms::doinker.set_value(false);
        robot::drivetrain::chassis.moveToPoint(-49.528, -60.194, 1000, {.forwards = false});
        robot::drivetrain::chassis.turnToHeading(270, 1000);
        robot::drivetrain::chassis.moveToPoint(-19.74, -60.194, 1500, {.forwards = false, .maxSpeed = 70});
        robot::drivetrain::chassis.waitUntil(25);
        robot::mechanisms::clamp.set_value(true);
        robot::drivetrain::chassis.waitUntilDone();
        autosetting::run_intake(2000);
        robot::drivetrain::chassis.moveToPoint(-50.499, -59.028, 1500);
        robot::drivetrain::chassis.turnToPoint(-29.137, -50.095, 1000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
        robot::drivetrain::chassis.waitUntilDone();
        robot::mechanisms::clamp.set_value(false);
        pros::delay(100);

        // Day 2 stuff (need tuning)
        robot::drivetrain::chassis.moveToPoint(-23.7, -47.182, 1500, {.minSpeed = 127, .earlyExitRange = 20});
        robot::drivetrain::chassis.waitUntilDone();
        autosetting::run_intake(900);
        robot::drivetrain::chassis.moveToPoint(-23.7, -47.182, 1500, {.maxSpeed = 70});
        robot::drivetrain::chassis.waitUntilDone();
        robot::drivetrain::chassis.turnToPoint(-22.923, -19.334, 1000, {.forwards = false});
        robot::drivetrain::chassis.moveToPoint(-22.923, -19.334, 1500, {.forwards = false, .maxSpeed = 80});
        robot::drivetrain::chassis.waitUntil(25);
        robot::mechanisms::clamp.set_value(true);
        robot::drivetrain::chassis.waitUntilDone();  
        autosetting::run_intake(3000);
        pros::delay(300);
        robot::drivetrain::chassis.turnToHeading(20, 1000);
        robot::drivetrain::chassis.moveToPoint(-21.564, -12.809, 1500);
        robot::drivetrain::chassis.waitUntilDone();
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
    try {
        robot::drivetrain::chassis.setPose(0, 0, 90);

        // Tune Lateral PID
        robot::drivetrain::chassis.turnToHeading(180, 1000);

        // Tune Angular PID
        /*
        robot::drivetrain::chassis.turnToHeading(90, 1000);
        robot::drivetrain::chassis.waitUntilDone();
        */
    } catch (const std::exception& e) {
        pros::lcd::print(0, "Test Auto Error: %s", e.what());
    }

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

