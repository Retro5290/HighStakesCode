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
static AutonomousMode current_auto = AutonomousMode::TEST;

namespace autosetting {
    struct IntakeState {
        static bool targetColor;
        static bool shouldRun;
        static uint32_t startTime;
        static uint32_t duration;
    };

    bool IntakeState::targetColor = (current_auto == AutonomousMode::ONE_STAKE_BLUE || current_auto == AutonomousMode::TWO_STAKE_BLUE); // false = red, true = blue
    bool IntakeState::shouldRun = false;
    uint32_t IntakeState::startTime = 0;
    uint32_t IntakeState::duration = 0;

    void intake_task_fn(void* param) {
        while (pros::competition::is_autonomous()) {
            if (IntakeState::shouldRun && 
                (pros::millis() - IntakeState::startTime < IntakeState::duration)) {
                robot::mechanisms::intakeMotor.move_velocity(robot::constants::INTAKE_SPEED);
                // if (robot::mechanisms::opticalSensor.get_hue() < 100) {
                     
                // } 
            } else {
                robot::mechanisms::intakeMotor.move_velocity(0);
                IntakeState::shouldRun = false;
            }
            pros::delay(10);
        }
    }

    void run_intake(int runTime) {
        IntakeState::startTime = pros::millis();
        IntakeState::duration = runTime;
        IntakeState::shouldRun = true;
    }
}

void skills_auto() {
    try {
    } catch (const std::exception& e) {
        pros::lcd::print(0, "Skills Auto Error: %s", e.what());
    }
}
/*
          0
     270     90
         180
*/
ASSET(RedSideRush_txt);
void one_stake_red_auto() {
    try {
        // Move to the mid goal
        robot::drivetrain::chassis.setPose(-50.939, -34.502, 90);
        robot::drivetrain::chassis.follow(RedSideRush_txt, 15, 2000);
        robot::drivetrain::chassis.waitUntil(35);
        // Grab the ring along the way

        autosetting::run_intake(2000);
        robot::drivetrain::chassis.waitUntil(50);
        robot::mechanisms::doinker.set_value(true); // Doink the goal
        pros::delay(500);

        // Swing to side
        robot::drivetrain::chassis.swingToHeading(200, lemlib::DriveSide::LEFT, 2000); //reverse
        robot::drivetrain::chassis.waitUntilDone();
        robot::mechanisms::doinker.set_value(false);
        pros::delay(500);

        // Turn back to goal
        robot::drivetrain::chassis.turnToHeading(20, 2000); //reverse
        robot::drivetrain::chassis.waitUntilDone();

        robot::drivetrain::chassis.moveToPoint(-50.939, -34.502, 2000, {.forwards = false});
        robot::drivetrain::chassis.waitUntilDone();

        // Clamp the Goal
        robot::mechanisms::clamp.set_value(true);
        pros::delay(500);

        robot::drivetrain::chassis.turnToPoint(-60.82, -52.52, 2000, {.forwards = false});
        robot::drivetrain::chassis.waitUntilDone();

        robot::drivetrain::chassis.moveToPoint(-60.82, -52.52, 2000, {.forwards = false});
        robot::drivetrain::chassis.waitUntilDone();

        robot::mechanisms::clamp.set_value(false);
        pros::delay(500); // Release the goal

        robot::drivetrain::chassis.turnToPoint(-32.727, -29.271, 2000, {.forwards = false});
        robot::drivetrain::chassis.waitUntilDone();

        robot::drivetrain::chassis.moveToPoint(-32.727, -29.271, 2000, {.forwards = false});
        robot::drivetrain::chassis.waitUntilDone();

        // Clamp alliance goal
        robot::mechanisms::doinker.set_value(true);
        pros::delay(500);

        // Score the second ring on to alliance goal
        autosetting::run_intake(2000);
        robot::drivetrain::chassis.turnToPoint(-29.627, -6.99, 2000);
        robot::drivetrain::chassis.waitUntilDone();

        robot::drivetrain::chassis.moveToPoint(-29.627, -6.99, 2000, {.maxSpeed = 80});
        robot::drivetrain::chassis.waitUntilDone();

    } catch (const std::exception& e) {
        pros::lcd::print(0, "Red Auto Error: %s", e.what());
    }
}

void two_stake_red_auto() {
    try {
        robot::drivetrain::chassis.setPose(-58.688, 16.065, 0);
        robot::drivetrain::chassis.moveToPoint(-58.688, 16.065, 2000, {.maxSpeed = 100});
        robot::drivetrain::chassis.waitUntilDone(); 
        robot::drivetrain::chassis.turnToHeading(90, 2000);
        robot::drivetrain::chassis.waitUntilDone();
        autosetting::run_intake(2000);
        robot::drivetrain::chassis.waitUntil(50);
    } catch (const std::exception& e) {
        pros::lcd::print(0, "Two Stake Red Auto Error: %s", e.what());
    }
}

void one_stake_blue_auto() {
    try {
    } catch (const std::exception& e) {
        pros::lcd::print(0, "One Stake Blue Auto Error: %s", e.what());
    }
}

void two_stake_blue_auto() {
    try {
    } catch (const std::exception& e) {
        pros::lcd::print(0, "Two Stake Blue Auto Error: %s", e.what());
    }
}

void test_auto() {
    printf("Running Test Auto\n");
    autosetting::run_intake(2000);
    robot::mechanisms::lbMotors.move_velocity(-2);
    pros::delay(3000);
    autosetting::run_intake(2000);
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

