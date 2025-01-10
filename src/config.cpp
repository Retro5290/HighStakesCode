#include "config.hpp"

namespace robot {
    // Controllers
    pros::Controller masterController(pros::E_CONTROLLER_MASTER);
    pros::Controller partnerController(pros::E_CONTROLLER_PARTNER);

    namespace drivetrain {
        // Drive Train Motors
        pros::MotorGroup leftMotors({-16, -9, -10}, pros::MotorGearset::blue); //8 9 10    
        pros::MotorGroup rightMotors({3, 4, 5}, pros::MotorGearset::blue); // 134

        // Sensors
        pros::Rotation verticalRotation(7);
        pros::Imu imu(18);    

        // Tracking wheel setup
        lemlib::TrackingWheel verticalTrackingWheel(
            &verticalRotation,
            lemlib::Omniwheel::NEW_2, 
            0.0
        );

        // Drivetrain configuration
        lemlib::Drivetrain drivetrain(
            &leftMotors,
            &rightMotors,
            11.4,
            lemlib::Omniwheel::NEW_325,
            450,
            2
        );

        // Odometry sensors
        lemlib::OdomSensors sensors(
            &verticalTrackingWheel,
            nullptr,
            nullptr,
            nullptr,
            &imu
        );

        // PID Controllers
        lemlib::ControllerSettings lateralController(
            10, // kP
            0,  // kI
            3,  // kD
            3,  // anti-windup
            1,  // small error range
            100,// small error timeout
            3,  // large error range
            500,// large error timeout
            20  // slew rate
        );
        
        lemlib::ControllerSettings angularController(
            2,  // kP
            0,  // kI
            10, // kD
            3,  // anti-windup
            1,  // small error range
            100,// small error timeout
            3,  // large error range
            500,// large error timeout
            0   // slew rate
        );  

        // Chassis instance
        lemlib::Chassis chassis(
            drivetrain,
            lateralController,
            angularController,
            sensors
        );
    }

    namespace mechanisms {
        pros::MotorGroup lbMotors({11, -20}, pros::MotorGearset::green);
        pros::Motor intakeMotor(1, pros::MotorGearset::blue);
   
        // Digital I/O
        pros::ADIDigitalIn lbLimitSwitch('H');
        pros::Optical opticalSensor(11);

        // Digital Out
        pros::ADIDigitalOut hang('E');
        pros::ADIDigitalOut clamp('G', true);
        pros::ADIDigitalOut doinker('F');
        pros::ADIDigitalOut intake('B');
    }       
}
