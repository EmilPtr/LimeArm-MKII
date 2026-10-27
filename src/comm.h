#ifndef COMM_H
#define COMM_H

#include <Arduino.h>

#define PACKET_SOF 0xFF
#define PACKET_PAYLOAD_SIZE 9 // 4 joints * 2 bytes (int16_t) + 1 byte for claw (uint8_t)
#define PACKET_TOTAL_SIZE 11   // SOF(1) + PAYLOAD(9) + CHECKSUM(1)

// Scale factor for float <-> int16_t conversion (e.g. 270.0 degrees -> 2700)
#define ANGLE_SCALE_FACTOR 100.0f

enum ParserState {
    STATE_WAIT_SOF,
    STATE_READ_PAYLOAD,
    STATE_READ_CHECKSUM
};

struct JointAngles {
    double base;
    double shoulder;
    double elbow;
    double wrist;
    uint8_t claw; // 0 = stop, 1 = open, 2 = close
};

class SerialComm {
public:
    SerialComm();

    // Call this as fast as possible in loop(). Returns true if new setpoints were parsed.
    bool processRX(JointAngles& outSetpoints);

    // Call this at the desired telemetry interval
    void sendTX(double baseAngle, double shoulderAngle, double elbowAngle, double wristAngle, uint8_t clawState);

private:
    ParserState rxState;
    uint8_t payloadBuffer[PACKET_PAYLOAD_SIZE];
    uint8_t payloadIndex;

    uint8_t calculateChecksum(uint8_t* buffer, uint8_t length);
};

#endif
