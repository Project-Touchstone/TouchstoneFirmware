/*
  BusChain.h - 64 I2C bus expansion using TCA9548A chips
  Created by Carson G. Ray
*/

#ifndef BusChain_h
#define BusChain_h

#include "Arduino.h"
#include <Wire.h>
#include "freertos/task.h"

#define ROOT_ADDRESS 0x70

class BusChain {
    private:
        static void sendAddressBits(uint8_t data, bool lock);

        //Serial data to shift register
        static uint8_t ser;
        //Data clock
        static uint8_t clk;
        //Register latching clock
        static uint8_t rclk;

        //Last I2C multiplexer group opened
        static int8_t lastGroup;

        //Interval (us) between clock signals
        static uint32_t clockInterval;

        //Mutex for I2C bus access
        static SemaphoreHandle_t mutex;
        
    public:
        static void begin(uint8_t ser, uint8_t clk, uint8_t rclk, uint8_t numGroups);
        static uint8_t selectPort(uint8_t port);
        static void release();
};

#endif