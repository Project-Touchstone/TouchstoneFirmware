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
        // Whether current data frame has ended
        static bool endFlag;
    public:
        // Initializes the serial interface
        static void begin(long baudRate);

        // Checks for an incoming header or end byte
        static bool processPacket();

        // Gets the current header
        static uint8_t getHeader();

        // Sends a byte of data
        static void sendByte(uint8_t data);

        static void sendBytes(uint8_t* buffer, uint8_t len);

        // Sends an integer
        static void sendInt16(int16_t data);

        // Sends the end of data frame
        static void sendEnd();

        // Checks if the packet has ended
        static bool isEnded();

        // Reads a byte of data
        static uint8_t readByte();

        static bool readBytes(uint8_t* buffer, uint8_t len);

        static int16_t readInt();

        // Reads a 32-bit float
        static float readFloat();

        // Clears the current packet
        static void clearPacket();
};

#endif