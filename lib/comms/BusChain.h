/*
  BusChain.h - 64 I2C bus expansion using TCA9548A chips
  Created by Carson G. Ray
*/

#ifndef BusChain_h
#define BusChain_h

//External imports
#include <Arduino.h>
#include <Wire.h>
#include <freertos/task.h>

#define ROOT_ADDRESS 0x70

class BusChain {
    private:
        // I2C bus
        TwoWire* i2cBus;
        // Address identifiers of all modules in chain
        uint8_t* moduleIds;
        //Last I2C multiplexer module opened
        int8_t lastModule = -1;

        //Mutex for I2C bus access
        SemaphoreHandle_t mutex;
        
    public:
        void begin(uint8_t moduleId);
        void begin(uint8_t* moduleIds, TwoWire* i2cBus);
        uint8_t selectChannel(uint8_t channel);
        TwoWire* getI2CBus();
        void release();
};

#endif