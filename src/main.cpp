#include "main.h"
#include "lemlib/api.hpp"
#include "pros/misc.h"

#include <algorithm>
#include <cmath>
#include <cstdint>


// Hardware Setup
pros::MotorGroup leftMotors({-3, -4, -5}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({6, 7, 8}, pros::MotorGearset::blue);
pros::MotorGroup liftMotor({-11, 12});

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::adi::Pneumatics wrist('H', false, true);
pros::adi::Pneumatics intakeLift('G', false, true);

pros::Motor intake(-13, pros::v5::MotorGears::blue);
pros::Motor clamp(-1, pros::v5::MotorGears::blue);

// Drivetrain and odometry
lemlib::Drivetrain drivetrain(
    &leftMotors,
    &rightMotors,
    9.5,                         // Track width in inches
    lemlib::Omniwheel::NEW_275,
    450,                         // Drive motor RPM
    2                            // Drivetrain horizontal drift
);

pros::Imu imu(14);
pros::Rotation verticalRotationSensor(15);
pros::Rotation horizontalRotationSensor(16);
pros::Rotation liftRotation(-20);

// Odometry Set up

lemlib::TrackingWheel horizontalTrackingWheel(
    &horizontalRotationSensor, 2.0, 0.175
);
lemlib::TrackingWheel verticalTrackingWheel(
    &verticalRotationSensor, 2.0, 1.25
);

lemlib::OdomSensors odomSensors(
    &verticalTrackingWheel,
    nullptr,
    &horizontalTrackingWheel,
    nullptr,
    &imu
);

lemlib::ControllerSettings lateralController(
    10,     // kP: proportional gain
    0,      // kI: integral gain
    50,     // kD: derivative gain
    3,      // Small error threshold
    1,      // Small error timeout (ms)
    100,    // Large error threshold
    3,      // Large error timeout (ms)
    500,    // Slew rate
    0       // Optional setting
);

lemlib::ControllerSettings angularController(
    2,      // kP: proportional gain
    0,      // kI: integral gain
    12,      // kD: derivative gain
    3,      // Small error threshold
    1,      // Small error timeout (ms)
    100,    // Large error threshold
    3,      // Large error timeout (ms)
    500,    // Slew rate
    0       // Optional setting
);

lemlib::Chassis chassis(
    drivetrain,
    lateralController,
    angularController,
    odomSensors
);

// Lift settings and PID state
constexpr int liftMaxHeight = 216000;
// constexpr int liftMinHeight = 10000;
constexpr int manualLiftControl = -1;

static volatile int liftTarget = manualLiftControl;
static volatile int liftLoops = 0;
static volatile int liftLastTarget = manualLiftControl;
static volatile int liftOutput = 0;

void spinTurn(int maxSpeed, double target, double error) {
    constexpr double kP = 2.4;
    constexpr int minSpeed = 50;
    constexpr int settleTime = 100;

    int settled = 0;

    while (true) {
        double current = chassis.getPose().theta;
        double difference = target - current;

        while (difference > 180) difference -= 360;
        while (difference < -180) difference += 360;

        if (std::abs(difference) <= error) {
            settled += 10;

            leftMotors.move(0);
            rightMotors.move(0);

            if (settled >= settleTime) break;
        } else {
            settled = 0;

            int speed = std::clamp(
                static_cast<int>(std::abs(difference) * kP),
                minSpeed,
                maxSpeed
            );

            if (difference > 0) {
                leftMotors.move(speed / 7);
                rightMotors.move(-speed);
            } else {
                leftMotors.move(-speed);
                rightMotors.move(speed / 7);
            }
        }
        pros::delay(10);
    }
    leftMotors.move(0);
    rightMotors.move(0);
}

void goalReset(int goalNumber) {
    constexpr double goalCoordinates[8][2] = {
        {-48,  24},
        {-48, -24},
        {-24, -48},
        { 24, -48},
        { 48, -24},
        { 48,  24},
        { 24,  48},
        {-24,  48}
    };

    if (goalNumber < 1 || goalNumber > 8) return;

    const double goalX = goalCoordinates[goalNumber - 1][0];
    const double goalY = goalCoordinates[goalNumber - 1][1];

    const double theta = chassis.getPose().theta * M_PI / 180.0;
    const double heading = chassis.getPose().theta;

    const double robotX = goalX + 7.0 * sin(theta);
    const double robotY = goalY + 7.0 * cos(theta);

    chassis.setPose(robotX, robotY, heading);
}

void liftToPosition(int targetPosition) {
    liftTarget = targetPosition;
}

void stopLiftMacro() {
    liftTarget = manualLiftControl;
}

enum class macroStepState {
    idle,
    singleClampBack,
    singleWaitUp,
    matchWaitDown
};

macroStepState macroStep = macroStepState::idle;
uint32_t macroStepStart = 0;


// Function for single pin scoring

void startSinglePin() {
    stopLiftMacro();
    wrist.retract();
    liftMotor.move(-127);
    pros::delay(300);
    wrist.extend();
    pros::delay(100);
    clamp.move(-127);
    liftMotor.move(0);
    pros::delay(500);
    wrist.retract();
    liftToPosition(30000);

    macroStep = macroStepState::idle;
}

// Function for Matchloading

void startMatchLoad() {
    stopLiftMacro();

    liftMotor.move(-127);
    pros::delay(400);
    liftMotor.move(0);
    clamp.move(127);
    pros::delay(300);
    liftToPosition(10000);
}


// Function for auto for lift

void startIntakeLift(){
    intake.move(127);
    clamp.move(127);
    intakeLift.retract();
    pros::delay(300);
    intake.move(0);

    leftMotors.move(-40);
    rightMotors.move(-40);
    pros::delay(300);
    leftMotors.move(0);
    rightMotors.move(0);
    intakeLift.extend();
    pros::delay(250);
    intake.move(127);
    leftMotors.move(40);
    rightMotors.move(40);
    pros::delay(150);
    intake.move(127);
    leftMotors.move(0);
    rightMotors.move(0);
    pros::delay(1500);
    intake.move(0);
    clamp.move(60);
    pros::delay(300);

    macroStep = macroStepState::matchWaitDown;
    macroStepStart = pros::millis();
}

// Intializing all the functions for the start

void initialize() {

    wrist.set_value(false);
    intakeLift.set_value(false);
    liftMotor.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    liftRotation.reset_position();
    pros::lcd::initialize();

    chassis.calibrate();

    pros::Task screenTask([] {
        // For testing purposes 
        while (true) {
            const auto pose = chassis.getPose();

            pros::lcd::print(0, "X: %.2f", pose.x);
            pros::lcd::print(1, "Y: %.2f", pose.y);
            pros::lcd::print(2, "Theta: %.2f", pose.theta);
            pros::lcd::print(3, "Lift: %d", liftRotation.get_position());
            pros::lcd::print(4, "Target: %d Last: %d", liftTarget, liftLastTarget);
            pros::lcd::print(5, "Output: %d Loops: %d", liftOutput, liftLoops);
            pros::lcd::print(6, "Motor velocity: %.0f", liftMotor.get_actual_velocity());

            pros::delay(100);
        }
    });

    // Lift Macro

    pros::Task liftTask([] {
        constexpr double kP = 0.012;
        constexpr double kI = 0.0;
        constexpr double kD = 0.01;
        constexpr double errorThreshold = 100.0;

        double previousError = 0;
        double integral = 0;
        int activeTarget = manualLiftControl;

        while (true) {
            const int target = liftTarget;
            if (target != manualLiftControl) {
                const double currentPosition = liftRotation.get_position();
                const double error = target - currentPosition;

                if (target != activeTarget) {
                    previousError = error;
                    integral = 0;
                    activeTarget = target;
                }

                integral += error;
                const double derivative = error - previousError;
                double output = kP * error + kI * integral + kD * derivative;
                constexpr double minOutput = 15.0;

                if (std::abs(error) > errorThreshold && std::abs(output) < minOutput) {
                    output = (output > 0) ? minOutput : -minOutput;
                }

                output = std::clamp(output, -127.0, 127.0);

                liftMotor.move(static_cast<int>(output));
                liftOutput = static_cast<int>(output);
                liftLastTarget = target;

                if (std::abs(error) < errorThreshold) {
                    liftMotor.brake();
                    liftTarget = manualLiftControl;
                }

                previousError = error;
            } else {
                activeTarget = manualLiftControl;
            }

            ++liftLoops;
            pros::delay(10);
        }
    });
    liftToPosition(10000);
}

void pid(){
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0, 24, 1000);
}

void fourpinauto(){
    chassis.setPose(-65, -9, 270);
    liftToPosition(10000);
    clamp.move(60);
    wrist.retract();

    //first pin
    chassis.moveToPoint(-57, -19, 1500, {.forwards = false, .minSpeed = 20, .earlyExitRange = 0});
    pros::delay(1000);
    intakeLift.retract();
    startSinglePin();

    //second pin
    spinTurn(127, 280, 2);
    chassis.moveToPoint(-66, -21.5, 500);
    liftToPosition(9500);
    wrist.extend();
    intake.move(127);
    chassis.waitUntilDone();
    startIntakeLift();
    wrist.retract();
    clamp.move(127);
    pros::delay(100);
    liftToPosition(50000);
    pros::delay(500);
    chassis.moveToPoint(-53.5, -21.5, 1500, {.forwards = false});
    pros::delay(300);
    liftToPosition(20000);
    pros::delay(200);
    clamp.move(-127);
    pros::delay(400);
    liftToPosition(55000);
    clamp.move(60);

    //third pin
    spinTurn(127, 220, 2);
    chassis.moveToPoint(-58.5, -35, 1000, {.minSpeed = 0, .earlyExitRange = 0}); 
    liftToPosition(17000);
    chassis.turnToPoint(-54.5, -42, 1000, {.forwards = false, .minSpeed = 20});
    chassis.moveToPoint(-54.5, -42, 1000, {.forwards = false, .maxSpeed = 20, .earlyExitRange = 6.5});
    chassis.waitUntilDone();
    startMatchLoad();
    clamp.move(127);
    pros::delay(250);
    liftToPosition(100000);
    chassis.turnToPoint(-49, -26.5, 1000, {.forwards = false, .minSpeed = 0, .earlyExitRange = 0});
    chassis.moveToPoint(-49, -26.5, 1000, {.forwards = false, .minSpeed = 0, .earlyExitRange = 0});
    chassis.waitUntilDone();
    liftToPosition(75000);
    clamp.move(-127);
    pros::delay(600);
    liftToPosition(100000);
    clamp.move(60);
    

    //fourth pin
    spinTurn(127, 90, 2);
    chassis.moveToPoint(-46, -21.5, 500);
    wrist.extend();
    intake.move(127);
    chassis.waitUntilDone();
    startIntakeLift();
    wrist.retract();
    clamp.move(127);
    pros::delay(100);
    liftToPosition(50000);
    pros::delay(500);
    chassis.moveToPoint(-53.5, -21.5, 1000, {.forwards = false});
    pros::delay(300);
    liftToPosition(20000);
    pros::delay(200);
    clamp.move(-127);
    pros::delay(400);
    liftToPosition(55000);
    clamp.move(60);
    
    // chassis.moveToPoint(-48, -40, 1000);
    // chassis.turnToPoint(-28, -28, 1000, {.forwards = false, .minSpeed = 0, .earlyExitRange = 0});
    // chassis.moveToPoint(-28, -28, 1000, {.forwards = false, .minSpeed = 0, .earlyExitRange = 0});
    // liftToPosition(17000);
    // startMatchLoad();
    // clamp.move(127);
    // chassis.turnToPoint(17, -47, 1000, {.forwards = false, .minSpeed = 0, .earlyExitRange = 0});
    // liftToPosition(75000);
    // chassis.moveToPoint(17, -47, 1000, {.forwards = false, .minSpeed = 0, .earlyExitRange = 0});
    // chassis.waitUntilDone();
    // liftToPosition(55000);
    // clamp.move(-127);
    // pros::delay(300);
    // liftToPosition(80000);
    // clamp.move(60);

    // // ---- Seg 3: drive forward to the wall at (-63, -24) -------------------
    // // TODO MECH: e.g. chainbarFunction(-8000); before arrival
    // chassis.moveToPoint(-63, -24, 1200, {.maxSpeed = 90});
    // chassis.waitUntilDone();
    // // TODO MECH: e.g. intake_piston.set_value(false);  pros::delay(250);
 
    // // ---- Seg 4: back out to (-53.5, -24) ----------------------------------
    // chassis.moveToPoint(-53.5, -24, 1000, {.forwards = false});
    // chassis.waitUntilDone();
 
    // // ---- Seg 5: curve to (-47, -30) facing 180 ----------------------------
    // chassis.moveToPose(-47, -30, 180, 1500);
    // chassis.waitUntilDone();
 
    // // ---- Seg 6: drive down to (-47, -40) ----------------------------------
    // chassis.moveToPoint(-47, -39.9, 1000);
    // chassis.waitUntilDone();
    // // TODO MECH
 
    // // ---- Seg 7: back up to (-47, -30) -------------------------------------
    // chassis.moveToPoint(-47, -30, 1000, {.forwards = false});
    // chassis.waitUntilDone();
 
    // // ---- Seg 8: curve to (-41, -24) facing 90 -----------------------------
    // chassis.moveToPose(-41, -24, 90, 1500);
    // chassis.waitUntilDone();
 
    // // ---- Seg 9: drive forward to (-30.5, -24) -----------------------------
    // chassis.moveToPoint(-30.5, -24, 1000);
    // chassis.waitUntilDone();
    // // TODO MECH
 
    // // ---- Seg 10: long reverse across the field to (17, -47), end at 270 --
    // chassis.turnToPoint(17, -47, 800, {.forwards = false});
    // chassis.moveToPose(17, -47, 270, 3000, {.forwards = false, .lead = 0.4});
    // chassis.waitUntil(30);  // inches travelled
    // // TODO MECH: e.g. start raising chainbar mid-drive
    // chassis.waitUntilDone();
 
    // // ---- Seg 11: curve to (9, -63) facing 180 -----------------------------
    // chassis.moveToPose(9, -63, 180, 1800);
    // chassis.waitUntilDone();
    // // TODO MECH
 
    // // ---- Seg 12: back up to (9, -50) and finish ---------------------------
    // chassis.moveToPoint(9, -50, 1000, {.forwards = false});
    // chassis.waitUntilDone();

}

void autoskills(){
   chassis.setPose(-62.5, -13, 270);
    liftToPosition(10000);
    clamp.move(60);
    wrist.retract();

    //first pin
    chassis.moveToPoint(-53, -20.5, 1000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 0});
    chassis.waitUntilDone();
    goalReset(2);
    startSinglePin();

    //go to loader
    spinTurn(127, 205, 5);
    // goalReset(2);
    chassis.moveToPoint(-45.5, -56, 1000);
    pros::delay(500);
    chassis.turnToPoint(-59.5, -56, 1000, {.forwards = false});
    liftToPosition(15500);
    chassis.moveToPoint(-59.5, -56, 1000, {.forwards = false});
   
    chassis.waitUntilDone();
    leftMotors.move(-40);
    rightMotors.move(-40);
    pros::delay(1500);
    startMatchLoad();
    pros::delay(500);
    leftMotors.move(0);
    rightMotors.move(0);
    chassis.moveToPoint(-45.5, -57, 1000);
    liftToPosition(30000);
    chassis.waitUntilDone();
    chassis.turnToPoint(-42, -29, 1000, {.forwards = false});
    chassis.moveToPoint(-42, -29, 1000, {.forwards = false});
    liftToPosition(20000);
    clamp.move(-127);
    pros::delay(250);
    liftToPosition(30000);

}


void disabled() {}

void competition_initialize() {}

void autonomous() {
    autoskills();
    

}

void opcontrol() {

    // controller commands 

    bool matchLoadReady = false;
    while (true) {
        const int drive = -master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        const int turn = static_cast<int>(
            master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) * 0.75
        );
        const int liftPosition = liftRotation.get_position();

        leftMotors.move(drive + turn);
        rightMotors.move(drive - turn);

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            stopLiftMacro();
            macroStep = macroStepState::idle;
            wrist.retract();

            if (liftPosition < liftMaxHeight) {
                liftMotor.move(127);
            } else {
                liftMotor.brake();
            }
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            stopLiftMacro();
            macroStep = macroStepState::idle;


                if (liftPosition > 14000) {
                    liftMotor.move(-127);
                } else if (liftPosition > 10000) {
                    liftMotor.move(-15);
                } else {
                    liftMotor.brake();
                }
            }
        else if (liftTarget == manualLiftControl) {
            liftMotor.brake();
        }
        // Intake control
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            wrist.extend();
            intake.move(127);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            clamp.move(-127);
            intake.move(-127);
        } else {
            intake.move(0);
        }

        if (macroStep == macroStepState::idle) {
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
                liftToPosition(10000);
            } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
                liftToPosition(liftRotation.get_position()+60000);
            } else {
                if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                    clamp.move(-100);
                    intake.move(-100);
                }
                else{
                    clamp.move(60);
                }
            }
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            startIntakeLift();
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            master.rumble(".");
            pros::lcd::print(7, "B triggered: %u", pros::millis());
            startSinglePin();
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)
            && macroStep == macroStepState::idle) {
            if (!matchLoadReady) {
                liftToPosition(15500);
                wrist.retract();
                matchLoadReady = true;
            } else {
                startMatchLoad();
                matchLoadReady = false;
            }
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            startSinglePin();
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            clamp.move(-127);
            intake.move(-127);
        }
        pros::delay(20);
    }
        
}