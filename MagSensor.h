/*
  MagSensor.h - Custom ring-based multi-magnet encoder implementation
  Created by Carson G. Ray
*/

#ifndef MagSensor_h
#define MagSensor_h

#include "Arduino.h"
#include "BusChain.h"
#include <Tlv493d.h>
#include <math.h>
#include "freertos/task.h"

class MagSensor {
    private:
        //MagSensor object
        Tlv493d magSensor;

        //BusChain object
        BusChain* busChain;
        //Encoder port on BusChain
        uint8_t encoderPort;

        //I2C port
        TwoWire *i2cPort;

        //Spinlock for RTOS
        portMUX_TYPE* spinlock;

    public:
        static const uint8_t DATA_LENGTH = 12;
        MagSensor();
        bool begin(uint8_t encoderPort, BusChain *busChain);
        void update();
        float getX();
        float getY();
        float getZ();
        int16_t rawX();
        int16_t rawY();
        int16_t rawZ();
};

#endif