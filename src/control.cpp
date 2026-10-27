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
    double degreesPerTick, 
    double maxAngle, 
    double minAngle,
    boolean reverseEncoder,
    uint8_t backlashOffset):

    dir1(dir1),
    dir2(dir2),
    limSw(limSw),
    encoder(encA, encB),
    pwm(pwm),
    pid(&input, &output, &setpoint, kp, ki, kd, DIRECT),
    input(0),
    output(0),
    setpoint(0),
    degreesPerTick(degreesPerTick),
    maxAngle(maxAngle),
    minAngle(minAngle),
    homedAngle(homedAngle),
    reverseEncoder(reverseEncoder),
    backlashOffset(backlashOffset)
{
    // Initialize the PID controller automatically upon instantiation
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(MOTOR_MIN_SPEED, MOTOR_MAX_SPEED);
    pid.SetSampleTime(SAMPLE_TIME);
}

void EncoderJoint::init() {
    pinMode(dir1, OUTPUT);
    pinMode(dir2, OUTPUT);
    pinMode(pwm, OUTPUT);
    pinMode(limSw, INPUT_PULLUP); // Use internal pull-up for the active-low limit switch
}

void EncoderJoint::setSetpointDegrees(double angle) {
    angle += backlashOffset; // Apply backlash offset to the desired angle
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
    if (reverseEncoder) {
        encoderValue = -encoderValue;
    }
    return encoderValue * degreesPerTick;
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
    long homedTicks = (long)(homedAngle / degreesPerTick);

    // Overwrite the encoder's internal counter to establish our absolute zero/homed reference
    if (reverseEncoder) {
        encoder.write(-homedTicks);
    } else {
        encoder.write(homedTicks);
    }

    // Update internal PID states to reflect the new homed position so the joint doesn't jump
    input = homedAngle;
    setpoint = homedAngle;
}


WristServo::WristServo(uint8_t pin, int minAngle, int maxAngle, int kinematicsOffset, boolean reversed)
    : servoPin(pin), minAngle(minAngle), maxAngle(maxAngle), kinematicsOffset(kinematicsOffset), reversed(reversed) {
    servo.attach(servoPin);
}

void WristServo::setAngle(int angle) {
    // Apply kinematics offset and reverse if necessary
    int adjustedAngle = constrain(angle, minAngle, maxAngle) + kinematicsOffset;
    if (reversed) {
        adjustedAngle = maxAngle - (adjustedAngle - minAngle);
    }

    // Write the adjusted angle to the servo
    servo.write(adjustedAngle);
}

ClawDCMotor::ClawDCMotor(uint8_t dir1, uint8_t dir2, uint8_t pwm)
    : dir1(dir1), dir2(dir2), pwm(pwm) {
    pinMode(dir1, OUTPUT);
    pinMode(dir2, OUTPUT);
    pinMode(pwm, OUTPUT);
}

void ClawDCMotor::openBlocking() {
    // Set the motor to open the claw
    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, LOW);
    analogWrite(pwm, 255);

    // Wait for a fixed duration to ensure the claw is fully open
    delay(MOTOR_OPEN_DURATION);

    // Stop the motor after opening
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, LOW);
    analogWrite(pwm, 0);
}

void ClawDCMotor::setClose() {
    // Set the motor to close the claw
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, HIGH);
    analogWrite(pwm, 255);
}