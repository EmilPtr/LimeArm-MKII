#include <Arduino.h>
#include <Servo.h>
#include "control.h"

// Base motor (Port 1)
EncoderJoint baseMotor(
    35, // dir1
    34, // dir2
    12, // pwm
    18, // encA
    31, // encB
    A15,  // limSw 
    1,  // kp 
    0,  // ki 
    0,  // kd 
    270,  // homedAngle 
    360.0 / (2348 * (72.0/18.0)), // degreesPerTick (derived from: encoderResolution=2348, gearRatio=72.0/18.0)
    270,  // maxAngle 
    0,   // minAngle 
    true // reverseEncoder
);

// Shoulder motor (Port 2)
EncoderJoint shoulderMotor(
    37, // dir1
    36, // dir2
    8,  // pwm
    19, // encA
    38, // encB
    0,  // limSw (tbd)
    1,  // kp 
    0,  // ki 
    0,  // kd 
    120,  // homedAngle 
    360.0 / (1440 * (72.0/8.0)), // degreesPerTick (derived from: encoderResolution=1440, gearRatio=72.0/8.0)
    120, // maxAngle
    0,   // minAngle
    false // reverseEncoder
);

// Elbow motor (Port 3)
EncoderJoint elbowMotor(
    43, // dir1
    42, // dir2
    9,  // pwm
    3,  // encA
    49, // encB
    0,  // limSw (tbd)
    1,  // kp 
    0,  // ki 
    0,  // kd 
    180,  // homedAngle 
    360.0 / (1440 * (72.0/8.0)), // degreesPerTick (derived from: encoderResolution=1440, gearRatio=72.0/8.0)
    180, // maxAngle
    75,  // minAngle
    false // reverseEncoder
);

unsigned long lastUpdate = 0;
unsigned long lastPrint = 0;

void setup() {
    Serial.begin(115200);
    baseMotor.init();
    shoulderMotor.init();
    elbowMotor.init();
    baseMotor.home();
    shoulderMotor.home();
    elbowMotor.home();
}

void loop() {
    if (millis() - lastUpdate > SAMPLE_TIME) {
        baseMotor.update();
        shoulderMotor.update();
        elbowMotor.update();
        lastUpdate = millis();
    }

    if (millis() - lastPrint > SERIAL_SAMPLE_TIME) {
        baseMotor.printDebugInfo("Base Motor");
        shoulderMotor.printDebugInfo("Shoulder Motor");
        elbowMotor.printDebugInfo("Elbow Motor");
        lastPrint = millis();
    }
}