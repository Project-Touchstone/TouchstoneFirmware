#include "SerialInterface.h"

uint8_t SerialInterface::header = 0;
bool SerialInterface::headerFlag = false;
bool SerialInterface::endFlag = true;
bool SerialInterface::checkEndFlag = false;

/// @brief Initializes the serial interface
/// @param baudRate Baud rate of serial communication
void SerialInterface::begin(long baudRate) {
    Serial.begin(baudRate);
    while (!Serial) {
        // Wait for serial port to connect
    }
}

/// @brief Checks incoming serial data for a header or end byte
void SerialInterface::update() {
    // Ensures last data frame and header have been processed
    if (Serial.available() > 0) {
        // Reads one byte of data
        uint8_t byte = Serial.peek();
        
        if (checkEndFlag && byte == END) {
            Serial.read();
            checkEndFlag = false;
            // Sets end flag
            endFlag = true;
        } else if (endFlag) {
            // If end of data frame was already reached, starts new data frame
            endFlag = false;
            header = Serial.read();
            // Sets header flag
            headerFlag = true;
        }
    }
}

bool SerialInterface::headerReady() {
    return headerFlag;
}

void SerialInterface::checkEnd() {
    checkEndFlag = true;
}

bool SerialInterface::isPacketEnded() {
    return endFlag;
}

uint8_t SerialInterface::getHeader() {
    return header;
}

void SerialInterface::clearPacket() {
    headerFlag = false;
    endFlag = true;
}

void SerialInterface::flush(int8_t numBytes) {
	if (numBytes < 0) {
        numBytes = Serial.available();
	}
	for (int i = 0; i < numBytes; i++) {
        Serial.read();
    }
}

void SerialInterface::sendByte(uint8_t data) {
    Serial.write(data);
}

void SerialInterface::sendBytes(uint8_t* buffer, uint8_t len) {
    Serial.write(buffer, len);
}

void SerialInterface::sendInt16(int16_t data) {
    uint8_t buffer[sizeof(data)];
    memcpy(buffer, &data, sizeof(data));
    sendBytes(buffer, sizeof(data));
}

void SerialInterface::sendFloat(float data) {
    uint8_t buffer[sizeof(data)];
    memcpy(buffer, &data, sizeof(data));
    sendBytes(buffer, sizeof(data));
}

void SerialInterface::sendEnd() {
    SerialInterface::sendByte(END);
}

uint8_t SerialInterface::readByte() {
    return Serial.read();
}

bool SerialInterface::readBytes(uint8_t* buffer, uint8_t len) {
    if (Serial.available() >= len) {
        Serial.readBytes(buffer, len);
        return true;
    }
    return false;
}