#ifndef SERIAL_INTERFACE_H
#define SERIAL_INTERFACE_H

#include <Arduino.h>

// Byte signifying end of data frame
#define END 0x0

class SerialInterface {
    private:
        // Current header
        static uint8_t header;
        // Whether new header has been received
        static bool headerFlag;
        // Whether to check for end of packet
        static bool checkEndFlag;
        // Whether current data frame has ended
        static bool endFlag;
    public:
        // Initializes the serial interface
        static void begin(long baudRate);

        // Checks for an incoming header or end byte
        static void update();

        // Whether a header is ready
        static bool headerReady();

        // Gets the current header
        static uint8_t getHeader();

        // Sends a byte of data
        static void sendByte(uint8_t data);

        static void sendBytes(uint8_t* buffer, uint8_t len);

        // Sends an integer
        static void sendInt16(int16_t data);

        // Sends a 32-bit float
        static void sendFloat(float data);

        // Sends the end of data frame
        static void sendEnd();

        // Checks to see if the packet has ended
        static void checkEnd();

        // If the packet has ended
        static bool isPacketEnded();

        // Reads a byte of data
        static uint8_t readByte();

        static bool readBytes(uint8_t* buffer, uint8_t len);

        template <typename T>
        static T readData();

        // Clears the current packet
        static void clearPacket();
};

template <typename T>
T SerialInterface::readData() {
    T data;
    uint8_t buffer[sizeof(data)];
    SerialInterface::readBytes(buffer, sizeof(data));

    memcpy(&data, buffer, sizeof(data));

    return data;
}

#endif