#ifndef SERIAL_INTERFACE_H
#define SERIAL_INTERFACE_H

#include <Arduino.h>

class SerialInterface {
    private:
        // Current header
        static uint8_t header;
        // Whether new header has been received
        static bool headerFlag;
        // Whether current data frame has ended
        static bool endFlag;
    public:
        // Initializes the serial interface
        static void begin(long baudRate);

        // Checks for an incoming header byte
        static void update();

        // Whether a header is ready
        static bool headerReady();

        // Gets the current header
        static uint8_t getHeader();

        // Sends a byte of data
        static void writeByte(uint8_t data);

        static void writeBytes(uint8_t* buffer, uint8_t len);

        // Sends an integer
        static void writeInt16(int16_t data);

        // Sends a 32-bit float
        static void writeFloat(float data);

        // Reads a byte of data
        static uint8_t readByte();

        static bool readBytes(uint8_t* buffer, uint8_t len);

        template <typename T>
        static T readData();

        // Clears the current packet
        static void clearPacket();

        // Flushes the read buffer
        static void flush(int8_t numBytes=-1);
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