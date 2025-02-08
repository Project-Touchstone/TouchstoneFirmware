#include "SerialInterface.h"

// Byte signifying end of data frame
#define END 0x0

SerialInterface::commandFlag = false;
SerialInterface::dataFrameLength = 0;
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
    return Serial.available();
}

/// @brief Reads incoming serial data
/// @return true (data read), false (no data read)
bool SerialInterface::readData() {
    if (available()) {
        // Reads one byte of data
        uint8_t buffer[1];
        Serial.readBytes(buffer, 1);
        
        if (buffer[0] == END) {
            // Sets end flag
            endFlag = true;
        } else if (endFlag) {
            // If end of data frame was already reached, starts new data frame
            dataFrameLength = 0;
            endFlag = false;
            command = buffer[0];
            // Sets command flag
            commandFlag = true;
        } else {
            // Adds data to data frame
            dataFrame[dataFrameLength] = buffer[0];
            dataFrameLength++;
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

uint8_t* SerialInterface::getDataFrame() {
    return &dataFrame;
}

uint8_t SerialInterface::getDataFrameLength() {
    return dataFrameLength;
}

void SerialInterface::sendData(uint8_t header) {
    sendData(header, new char[0], 0);
}
void SerialInterface::sendData(uint8_t header, char* data, uint8_t length) {
    //Sends header and data
    Serial.write(header, 1);
    sendData(data, length);
}
void SerialInterface::sendData(char* data, uint8_t length) {
    if (length > 0) {
        //Sends data
        Serial.write(*data, length);
        //Sends end byte
        Serial.write(END, 1);
    }
}