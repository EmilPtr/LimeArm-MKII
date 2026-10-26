#include "comm.h"

SerialComm::SerialComm()
    : rxState(STATE_WAIT_SOF), payloadIndex(0) {}

uint8_t SerialComm::calculateChecksum(uint8_t* buffer, uint8_t length) {
    // Checksum covers the SOF and payload bytes
    uint8_t checksum = PACKET_SOF;
    for (uint8_t i = 0; i < length; i++) {
        checksum ^= buffer[i];
    }
    return checksum;
}

bool SerialComm::processRX(JointAngles& outSetpoints) {
    bool packetReceived = false;

    while (Serial.available() > 0) {
        uint8_t incomingByte = Serial.read();

        switch (rxState) {
            case STATE_WAIT_SOF:
                if (incomingByte == PACKET_SOF) {
                    rxState = STATE_READ_PAYLOAD;
                    payloadIndex = 0;
                }
                break;

            case STATE_READ_PAYLOAD:
                payloadBuffer[payloadIndex++] = incomingByte;
                if (payloadIndex >= PACKET_PAYLOAD_SIZE) {
                    rxState = STATE_READ_CHECKSUM;
                }
                break;

            case STATE_READ_CHECKSUM:
                // Verify packet integrity
                uint8_t expectedChecksum = calculateChecksum(payloadBuffer, PACKET_PAYLOAD_SIZE);
                if (incomingByte == expectedChecksum) {
                    // Packet valid! Parse payload
                    // Cast bytes to int16_t (Little Endian format expected)
                    int16_t baseRaw = payloadBuffer[0] | (payloadBuffer[1] << 8);
                    int16_t shoulderRaw = payloadBuffer[2] | (payloadBuffer[3] << 8);
                    int16_t elbowRaw = payloadBuffer[4] | (payloadBuffer[5] << 8);

                    // Output the parsed setpoints
                    outSetpoints.base = (double)baseRaw / ANGLE_SCALE_FACTOR;
                    outSetpoints.shoulder = (double)shoulderRaw / ANGLE_SCALE_FACTOR;
                    outSetpoints.elbow = (double)elbowRaw / ANGLE_SCALE_FACTOR;
                    packetReceived = true;
                }
                // Always reset state machine after checking (or on fail)
                rxState = STATE_WAIT_SOF;
                break;
        }
    }
    return packetReceived;
}

void SerialComm::sendTX(double baseAngle, double shoulderAngle, double elbowAngle) {
    // Prepare packet buffer
    uint8_t packet[PACKET_TOTAL_SIZE];
    
    // 1. SOF
    packet[0] = PACKET_SOF;

    // 2. Scale angles to int16_t
    int16_t baseRaw = (int16_t)(baseAngle * ANGLE_SCALE_FACTOR);
    int16_t shoulderRaw = (int16_t)(shoulderAngle * ANGLE_SCALE_FACTOR);
    int16_t elbowRaw = (int16_t)(elbowAngle * ANGLE_SCALE_FACTOR);

    // Serialize as Little Endian
    packet[1] = baseRaw & 0xFF;
    packet[2] = (baseRaw >> 8) & 0xFF;
    packet[3] = shoulderRaw & 0xFF;
    packet[4] = (shoulderRaw >> 8) & 0xFF;
    packet[5] = elbowRaw & 0xFF;
    packet[6] = (elbowRaw >> 8) & 0xFF;

    // 3. Calculate Checksum
    packet[7] = calculateChecksum(&packet[1], PACKET_PAYLOAD_SIZE);

    // Transmit binary packet as a single block for low overhead
    Serial.write(packet, PACKET_TOTAL_SIZE);
}
