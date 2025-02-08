#ifndef SerialInterface_h
#define SerialInterface_h

#include <Arduino.h>

class SerialInterface {
    private:
        // Data frame
        static uint8_t dataFrame[255];
        //Length of data frame
        static uint8_t dataFrameLength;

        //Whether end of data frame has been reached
        static bool endFlag;

        // Current command
        static uint8_t command;

        // Whether a command is ready to be processed
        static bool commandFlag;

    public:
        // Initializes the serial interface
        static void begin(long baudRate);

        // Checks if serial data is available
        static bool available();
        // Handles incoming serial data
        static bool readData();
        // Whether a command is ready to be processed
        static bool commandReady();
        // Whether transmission has ended
        static bool isEnded();
        // Gets current command
        static uint8_t getCommand();
        // Clears current commmand
        static void clearCommand();
        // Retrieves data frame
        static uint8_t* getDataFrame();
        // Retrieves length of data frame
        static uint8_t getDataFrameLength();

        // Sends outgoing serial data
        static void sendData(uint8_t header);
        static void sendData(uint8_t header, char* data, uint8_t length);
        static void sendData(char* data, uint8_t length);
};

#endif