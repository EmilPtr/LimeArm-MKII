#include "control.h"

EncoderJoint::EncoderJoint(
    uint8_t dir1,
    uint8_t dir2,
    uint8_t pwm,
    uint8_t encA, 
    uint8_t encB, 
    uint8_t limSw,
    double kp, 
    double ki, 
    double kd, 
    double homedAngle, 
    double encoderResolution, 
    double gearRatio, 
    double maxAngle, 
    double minAngle):

    dir1(dir1),
    dir2(dir2),
    limSw(limSw),
    encoder(encA, encB),
    pwm(pwm),
    pid(&input, &output, &setpoint, kp, ki, kd, DIRECT),
    input(0),
    output(0),
    setpoint(0),
    encoderResolution(encoderResolution),
    gearRatio(gearRatio),
    maxAngle(maxAngle),
    minAngle(minAngle),
    homedAngle(homedAngle)
{
    // Initialize the PID controller automatically upon instantiation
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(MOTOR_MIN_SPEED, MOTOR_MAX_SPEED);
}

void EncoderJoint::init() {
    pinMode(dir1, OUTPUT);
    pinMode(dir2, OUTPUT);
    pinMode(pwm, OUTPUT);
    pinMode(limSw, INPUT_PULLUP); // Use internal pull-up for the active-low limit switch
}

void EncoderJoint::setSetpointDegrees(double angle) {
    if (angle > maxAngle) {
        setpoint = maxAngle;
    } else if (angle < minAngle) {
        setpoint = minAngle;
    } else {
        setpoint = angle;
    }
}

double EncoderJoint::getCurrentAngle() {
    long encoderValue = encoder.read();
    // Convert raw ticks to degrees factoring in the encoder resolution and gearbox ratio
    return (encoderValue / encoderResolution) * (360.0 / gearRatio);
}

void EncoderJoint::moveMotor(int speed) {
    if (speed > 0) {
        digitalWrite(dir1, HIGH);
        digitalWrite(dir2, LOW);
        analogWrite(pwm, speed);
    } else if (speed < 0) {
        digitalWrite(dir1, LOW);
        digitalWrite(dir2, HIGH);
        analogWrite(pwm, -speed); // Convert negative speed to positive magnitude for PWM
    } else {
        digitalWrite(dir1, LOW);
        digitalWrite(dir2, LOW);
        analogWrite(pwm, 0); // Stop the motor completely
    }
}

void EncoderJoint::update() {
    input = getCurrentAngle();
    if (pid.Compute()) {
        moveMotor(output);
    }
}

void EncoderJoint::printDebugInfo(const char* label) {
    Serial.print(label);
    Serial.print(" -> Setpoint: ");
    Serial.print(setpoint);
    Serial.print(", Current Angle: ");
    Serial.print(input);
    Serial.print(", Output: ");
    Serial.println(output);
}

void EncoderJoint::home() {
    // Move the motor backwards slowly to search for the limit switch
    moveMotor(MOTOR_HOME_SPEED); 

    // Blocking loop: Wait until the limit switch triggers (reads LOW due to INPUT_PULLUP)
    while (digitalRead(limSw) == HIGH) {
        delay(5);
    }

    // Immediately stop the motor upon triggering the switch
    moveMotor(0);

    // Calculate the raw encoder ticks that correspond to the predetermined homed angle
    // Formula derived from: angle = (ticks / encoderResolution) * (360.0 / gearRatio)
    long homedTicks = (long)((homedAngle * encoderResolution * gearRatio) / 360.0);

    // Overwrite the encoder's internal counter to establish our absolute zero/homed reference
    encoder.write(homedTicks);

    // Update internal PID states to reflect the new homed position so the joint doesn't jump
    input = homedAngle;
    setpoint = homedAngle;
}
