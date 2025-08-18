#include "MagEncoder.h"

// AS5600 default I2C address is 0x36

MagEncoder::MagEncoder()
    : i2cPort(nullptr), sensorPort(0), busChain(nullptr), busChainEnable(false), lastRawAngle(0) {}

bool MagEncoder::begin(TwoWire* wire) {
    i2cPort = wire;
    as5600.begin();
    // Check if device is connected
    return as5600.isConnected();
}

bool MagEncoder::begin(uint8_t sensorPort, BusChain* busChain) {
    this->sensorPort = sensorPort;
    this->busChain = busChain;

    // Ensures busChain is used
    busChainEnable = true;

    // Selects sensor port and initializes sensor settings
    busChain->selectPort(sensorPort);
    bool ret = begin(busChain->getI2CPort());
    busChain->release();
    return ret;
}

void MagEncoder::update() {
    // If using BusChain, select the port before reading
    if (busChainEnable) {
        busChain->selectPort(sensorPort);
    }
    lastRawAngle = as5600.readAngle();
    // Release the BusChain after reading
    if (busChainEnable) {
        busChain->release();
    }
}

uint16_t MagEncoder::getRawAngle() {
    return lastRawAngle;
}

float MagEncoder::getAngleDegrees() {
    // AS5600 outputs 12-bit value (0-4095) for 0-360 degrees
    return (lastRawAngle * AS5600_RAW_TO_DEGREES);;
}

float MagEncoder::getAngleRadians() {
    return (lastRawAngle * AS5600_RAW_TO_RADIANS);
}
