/*
  MagSensor.h - 3D magnetic sensor driver
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
#include "I2CDevice.h"

class MagSensor : public I2CDevice {
    private:
        //MagSensor object
        Tlv493d magSensor;

        //BusChain object
        BusChain* busChain;
        //Sensor channel on BusChain
        uint8_t sensorChannel = 0;

        //I2C bus
        TwoWire *i2cBus;

        //Spinlock for RTOS
        portMUX_TYPE* spinlock;

        // Whether using BusChain
        bool busChainEnable = false;

    public:
        MagSensor();
        bool begin(uint8_t sensorPort, BusChain *busChain) override;
        bool begin(TwoWire* wire) override;
        void update();
        float getX();
        float getY();
        float getZ();
        int16_t rawX();
        int16_t rawY();
        int16_t rawZ();
};

#endif