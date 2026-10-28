#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>
#include <Servo.h>
#include <PID_v1.h>
#include <Encoder.h>

// Pin Constants
#define YAW_MOTOR_PIN_A 9
#define YAW_MOTOR_PIN_B 10
#define YAW_ENCODER_PIN_A 2
#define YAW_ENCODER_PIN_B 3

#define SHOULDER_MOTOR_PIN_A 5
#define SHOULDER_MOTOR_PIN_B 6
#define SHOULDER_ENCODER_PIN_A 4
#define SHOULDER_ENCODER_PIN_B 7

#define ELBOW_MOTOR_PIN_A 11
#define ELBOW_MOTOR_PIN_B 12
#define ELBOW_ENCODER_PIN_A 8
#define ELBOW_ENCODER_PIN_B 13

#define WRIST_SERVO_PIN 3
#define WRIST_SERVO_OFFSET -5
#define WRIST_SERVO_MIN 833
#define WRIST_SERVO_MAX 2167

#define MOTOR_MAX_SPEED 150
#define MOTOR_MIN_SPEED -150

#define MOTOR_HOME_SPEED 50

#define CLAW_MOTOR_DURATION 700 // Duration to fully open the claw in milliseconds

// PID constants
#define KP 2.0 // Proportional gain
#define KI 0.0 // Integral gain
#define KD 1.0 // Derivative gain

#define SAMPLE_TIME 20 // PID sample time in milliseconds
#define SERIAL_SAMPLE_TIME 100 // Serial print sample time in milliseconds

/**
 * @class EncoderJoint
 * @brief Represents a single robotic joint controlled by a DC motor with an encoder and PID control.
 */
class EncoderJoint {
public:
    EncoderJoint(
        uint8_t dir1, uint8_t dir2, uint8_t pwm, uint8_t encA, uint8_t encB, uint8_t limSw,
        double kp, double ki, double kd, double homedAngle, double degreesPerTick, 
        double maxAngle, double minAngle, boolean reverseEncoder, uint8_t backlashOffset);

    void init();
    void setSetpointDegrees(double angle);
    double getCurrentAngle();
    void moveMotor(int speed);
    void update();
    void printDebugInfo(const char* label);
    void home();

private:
    uint8_t dir1;
    uint8_t dir2;
    uint8_t limSw;
    Encoder encoder;
    uint8_t pwm;
    PID pid;
    double input;
    double output;
    double setpoint;
    double degreesPerTick;
    double maxAngle;
    double minAngle;
    double homedAngle;
    boolean reverseEncoder;
    uint8_t backlashOffset;
};

class WristServo {
public:
    WristServo(uint8_t pin, int minAngle, int maxAngle, int kinematicsOffset, boolean reversed);
    void setAngle(int angle);
    void attach();
    int getWristServoAngle();
private:
    uint8_t servoPin;
    int minAngle;
    int maxAngle;
    int kinematicsOffset;
    boolean reversed;
    Servo servo;
};

class ClawDCMotor {
public:
    ClawDCMotor(uint8_t dir1, uint8_t dir2, uint8_t pwm);
    void setOpen();
    void setClose();
    void init();

private:
    uint8_t dir1;
    uint8_t dir2;
    uint8_t pwm;
};

#endif
