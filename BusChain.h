/*
  BusChain.h - 64 I2C bus expansion using TCA9548A chips
  Created by Carson G. Ray
*/

#ifndef BusChain_h
#define BusChain_h

#include "Arduino.h"
#include <Wire.h>
#include "freertos/task.h"
#include <Wire.h>

#define ROOT_ADDRESS 0x70

class BusChain {
    private:
        // I2C bus
        TwoWire* i2cPort;
        // Address identifiers of all modules in chain
        uint8_t* busIds;
        //Last I2C multiplexer group opened
        int8_t lastGroup = -1;

        //Mutex for I2C bus access
        SemaphoreHandle_t mutex;
        
    public:
        void begin(uint8_t busId);
        void begin(uint8_t busIds);
        void begin(uint8_t* busIds);
        void begin(uint8_t* busIds, TwoWire* i2cPort);
        uint8_t selectPort(uint8_t port);
        TwoWire* getI2CPort();
        void release();
};

#endif