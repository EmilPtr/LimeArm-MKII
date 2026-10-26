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
    0.75,  // kp 
    0.075,  // ki 
    0,  // kd 
    270,  // homedAngle 
    2348,  // encoderResolution
    72.0/18.0,  // gearRatio 
    270,  // maxAngle 
    0   // minAngle 
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
    1440,      // encoderResolution
    8.0/72.0,  // gearRatio
    120, // maxAngle
    0    // minAngle
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
    1440,      // encoderResolution
    8.0/72.0,  // gearRatio
    180, // maxAngle
    75   // minAngle
);

void setup() {
    Serial.begin(115200);
    baseMotor.init();
    shoulderMotor.init();
    elbowMotor.init();
    baseMotor.home();
    baseMotor.setSetpointDegrees(180);
}

void loop() {
    baseMotor.update();
    //shoulderMotor.update();
    //elbowMotor.update();

    // Print less frequently to avoid flooding the Serial monitor
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 100) {
        lastPrint = millis();
        baseMotor.printDebugInfo("Base");
        //shoulderMotor.printDebugInfo("Shoulder");
        //elbowMotor.printDebugInfo("Elbow");
    }
}