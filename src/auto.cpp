#include "main.h"
#include "config.hpp"
#include "auto.h"

enum class AutonomousMode {
    SKILLS,
    ONE_STAKE_RED,
    TWO_STAKE_RED,
    ONE_STAKE_BLUE,
    TWO_STAKE_BLUE
};

// Current autonomous selection
static AutonomousMode current_auto = AutonomousMode::SKILLS;

void run_intake(int runTime){
    pros::Task intake_task([](){
        robot::mechanisms::intakeMotor.move_velocity(robot::constants::INTAKE_SPEED);
        pros::delay(2000);
        robot::mechanisms::intakeMotor.move_velocity(0);
    });
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
        robot::drivetrain::chassis.setPose(-50.939, -34.502, 90);
        robot::drivetrain::chassis.follow(RedSideRush_txt, 15, 2000);
        robot::drivetrain::chassis.waitUntil(35);
        run_intake(2000);
        robot::drivetrain::chassis.waitUntil(50);
        robot::mechanisms::doinker.set_value(true);
        pros::delay(500);

        robot::drivetrain::chassis.swingToHeading(180, lemlib::DriveSide::LEFT, 2000);
    } catch (const std::exception& e) {
        pros::lcd::print(0, "Red Auto Error: %s", e.what());
    }
}

void two_stake_red_auto() {
    try {
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

void autonomous() {
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
    }
}

