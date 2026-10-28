#include <Arduino.h>
#include <Servo.h>
#include "control.h"
#include "comm.h"

// Base motor (Port 1)
EncoderJoint baseMotor(
    35, // dir1
    34, // dir2
    12, // pwm
    18, // encA
    31, // encB
    A15,  // limSw 
    10,  // kp 
    0.1,  // ki 
    0,  // kd 
    270,  // homedAngle 
    360.0 / ((52*46.8) * (72.0/18.0)), // degreesPerTick (derived from: encoderResolution=2348, gearRatio=72.0/18.0)
    270,  // maxAngle 
    135,   // minAngle 
    true, // reverseEncoder
    0 // backlashOffset
);

// Shoulder motor (Port 2)
EncoderJoint shoulderMotor(
    37, // dir1
    36, // dir2
    8,  // pwm
    19, // encA
    38, // encB
    A13,  // limSw
    15,  // kp 
    0,  // ki 
    1,  // kd 
    120,  // homedAngle 
    360.0 / ((52*34) * (72.0/8.0)), // degreesPerTick (derived from: encoderResolution=1440, gearRatio=72.0/8.0)
    120, // maxAngle
    0,   // minAngle
    false, // reverseEncoder
    5 // backlashOffset
);

// Elbow motor (Port 3)
EncoderJoint elbowMotor(
    43, // dir1
    42, // dir2
    9,  // pwm
    3,  // encA
    49, // encB
    A14,  // limSw
    15,  // kp 
    0,  // ki 
    1,  // kd 
    180,  // homedAngle 
    360.0 / ((52*34) * (72.0/8.0)), // degreesPerTick (derived from: encoderResolution=1440, gearRatio=72.0/8.0)
    180, // maxAngle
    75,  // minAngle
    false, // reverseEncoder
    15 // backlashOffset
);

WristServo wristServo(A9, 90, 270, -90, false);
ClawDCMotor clawMotor(32, 33, 11);

unsigned long lastUpdate = 0;
unsigned long lastPrint = 0;

SerialComm comm;
JointAngles targetSetpoints;

void setup() {
    Serial.begin(115200);
    baseMotor.init();
    shoulderMotor.init();
    elbowMotor.init();
    baseMotor.home();
    shoulderMotor.home();
    elbowMotor.home();
    wristServo.attach();
    clawMotor.init();
    clawMotor.setClose(); // Start with the claw closed
}

void loop() {
    // 1. Process incoming setpoints as fast as possible
    if (comm.processRX(targetSetpoints)) {
        // Apply the new parsed setpoints directly to the motors
        baseMotor.setSetpointDegrees(targetSetpoints.base);
        shoulderMotor.setSetpointDegrees(targetSetpoints.shoulder);
        elbowMotor.setSetpointDegrees(targetSetpoints.elbow);
        wristServo.setAngle(targetSetpoints.wrist);
        
        if (targetSetpoints.claw == 1) {
            clawMotor.setOpen();
        } else if (targetSetpoints.claw == 2) {
            clawMotor.setClose();
        }
        // If claw == 0, we do nothing and maintain current state
    }

    // 2. Update PID controllers at a fixed interval
    if (millis() - lastUpdate > SAMPLE_TIME) {
        baseMotor.update();
        shoulderMotor.update();
        elbowMotor.update();
        lastUpdate = millis();
    }

    // 3. Send high-speed binary telemetry at a fixed interval
    if (millis() - lastPrint > SERIAL_SAMPLE_TIME) {
        // Fetch current angles and send them
        comm.sendTX(
            baseMotor.getCurrentAngle(),
            shoulderMotor.getCurrentAngle(),
            elbowMotor.getCurrentAngle(),
            targetSetpoints.wrist,
            targetSetpoints.claw
        );
        lastPrint = millis();
    }
}