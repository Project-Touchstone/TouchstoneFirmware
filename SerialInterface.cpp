#include "SerialInterface.h"

// Byte signifying end of data frame
#define END 0x0

SerialInterface::commandFlag = false;
SerialInterface::endFlag = true;

/// @brief Initializes the serial interface
/// @param baudRate Baud rate of serial communication
void SerialInterface::begin(long baudRate) {
    Serial.begin(baudRate);
    while (!Serial) {
        // Wait for serial port to connect
    }
}

/// @brief Checks if serial data is available
/// @return true (available), false (not available)
bool SerialInterface::available() {
    return Serial.available() > 0;
}

/// @brief Checks incoming serial data for a header or end byte
/// @return true (command to process), false (no command to process)
bool SerialInterface::processCommand() {
    // Ensures last data frame and command have been processed
    if (available()) {
        // Reads one byte of data
        uint8_t byte = Serial.peek();
        
        if (byte == END) {
            // Sets end flag
            endFlag = true;
            return false;
        } else if (endFlag) {
            // If end of data frame was already reached, starts new data frame
            endFlag = false;
            command = Serial.read();
            // Sets command flag
            commandFlag = true;
        }

        return true;
    } else {
        return false;
    }
}

bool SerialInterface::commandReady() {
    return commandFlag;
}

bool SerialInterface::isEnded() {
    return endFlag;
}

uint8_t SerialInterface::getCommand() {
    return command;
}

void SerialInterface::clearCommand() {
    commandFlag = false;
}

void SerialInterface::sendByte(uint8_t data) {
    Serial.write(data);
}

void SerialInterface::sendData(T data) {
    uint8_t buffer[sizeof(data)];
    for (int i = 0; i < sizeof(data); i++) {
        buffer[i] = (data & 0b11111111);
        data = data >> 8;
    }
    Serial.write(buffer, sizeof(data));
}

void SerialInterface::sendEnd() {
    Serial.write(END);
}

uint8_t SerialInterface::readByte() {
    if (Serial.available() > 0) {
        return Serial.read();
    }
    return 0;
}

float SerialInterface::readFloat() {
    float value;
    if (Serial.available() >= sizeof(value)) {
        value = Serial.parseFloat();
    }
    return value;
}