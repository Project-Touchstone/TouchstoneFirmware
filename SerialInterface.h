#ifndef SERIAL_INTERFACE_H
#define SERIAL_INTERFACE_H

#include <Arduino.h>

class SerialInterface {
    private:
        // Current command
        static uint8_t command;
        // Whether new command has been received
        static bool commandFlag;
        // Whether current data frame has ended
        static bool endFlag;
    public:
        // Initializes the serial interface
        static void begin(long baudRate);

        // Checks for an incoming header or end byte
        static bool processCommand();

        // Gets the current command
        static uint8_t getCommand();

        // Sends a byte of data
        static void sendByte(uint8_t data);

        // Sends a data type in binary form
        template <typename T>
        static void sendData(T data);

        // Sends the end of data frame
        static void sendEnd();

        // Checks if data is available
        static bool available();

        // Checks if the command has ended
        static bool isEnded();

        // Reads a byte of data
        static uint8_t readByte();

        // Reads a 32-bit float
        static float readFloat32();

        // Clears the current command
        static void clearCommand();
};

#endif