#ifndef I2CDEVICE_H
#define I2CDEVICE_H

#include <Arduino.h>
#include <Wire.h>
#include "BusChain.h"

class I2CDevice {
public:
    virtual ~I2CDevice() = default;

    // Begin using direct I2C bus
    virtual bool begin(TwoWire* wire);

    // Begin using BusChain multiplexer
    virtual bool begin(uint8_t channel, BusChain* busChain);
};

#endif
