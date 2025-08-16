/*
  MagSensor.h - Custom ring-based multi-magnet encoder implementation
  Created by Carson G. Ray
*/

#ifndef MagSensor_h
#define MagSensor_h

//External imports
#include <Arduino.h>
#include <Tlv493d.h>
#include <math.h>
#include <freertos/task.h>

//Local imports
#include "BusChain.h"

class MagSensor {
    private:
        //MagSensor object
        Tlv493d magSensor;

        //BusChain object
        BusChain* busChain;
        //Sensor port on BusChain
        uint8_t sensorPort;

        //I2C port
        TwoWire *i2cPort;

        //Spinlock for RTOS
        portMUX_TYPE* spinlock;

    public:
        MagSensor();
        bool begin(uint8_t sensorPort, BusChain *busChain);
        void update();
        float getX();
        float getY();
        float getZ();
        int16_t rawX();
        int16_t rawY();
        int16_t rawZ();
};

#endif