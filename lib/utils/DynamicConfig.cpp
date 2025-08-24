/**
 * DyanmicConfig.cpp - Dynamic hardware peripheral configuration framework
 * Created by Carson G. Ray
 */

#include "DynamicConfig.h"

void DynamicConfig::setI2CBuses(TwoWire* buses) {
    std::lock_guard<std::mutex> lock(configMutex);
    this->i2cBuses = buses;
}

void DynamicConfig::setBusChains(BusChain* busChains) {
    std::lock_guard<std::mutex> lock(configMutex);
    this->busChains = busChains;
}

void DynamicConfig::addBusChain(const BusChainConfig config) {
    std::lock_guard<std::mutex> lock(configMutex);
    busChainConfigs.push_back(config);
}

void DynamicConfig::addMagEncoder(const I2CDeviceConfig config) {
    std::lock_guard<std::mutex> lock(configMutex);
    magEncoderConfigs.push_back(config);
}

void DynamicConfig::addMagTracker(const I2CDeviceConfig config) {
    std::lock_guard<std::mutex> lock(configMutex);
    magTrackerConfigs.push_back(config);
}

void DynamicConfig::addIMU(const IMUConfig config) {
    std::lock_guard<std::mutex> lock(configMutex);
    imuConfigs.push_back(config);
}

void DynamicConfig::addServoDriver(const I2CDeviceConfig config) {
    std::lock_guard<std::mutex> lock(configMutex);
    servoDriverConfigs.push_back(config);
}

void DynamicConfig::addServo(const ServoConfig config) {
    std::lock_guard<std::mutex> lock(configMutex);
    servoConfigs.push_back(config);
}

void DynamicConfig::addFOCMotor(const FOCMotorConfig config) {
    std::lock_guard<std::mutex> lock(configMutex);
    focMotorConfigs.push_back(config);
}

uint8_t DynamicConfig::numBusChains() const  {
    std::lock_guard<std::mutex> lock(configMutex);
    return busChainConfigs.size();
}

uint8_t DynamicConfig::numMagEncoders() const  {
    std::lock_guard<std::mutex> lock(configMutex);
    return magEncoderConfigs.size();
}

uint8_t DynamicConfig::numMagTrackers() const  {
    std::lock_guard<std::mutex> lock(configMutex);
    return magTrackerConfigs.size();
}

uint8_t DynamicConfig::numIMUs() const  {
    std::lock_guard<std::mutex> lock(configMutex);
    return imuConfigs.size();
}

uint8_t DynamicConfig::numServos() const  {
    std::lock_guard<std::mutex> lock(configMutex);
    return servoConfigs.size();
}

uint8_t DynamicConfig::numServoDrivers() const  {
    std::lock_guard<std::mutex> lock(configMutex);
    return servoDriverConfigs.size();
}

DynamicConfig::BusChainConfig DynamicConfig::getBusChain(uint8_t id) const {
    std::lock_guard<std::mutex> lock(configMutex);
    if (id < busChainConfigs.size()) {
        return busChainConfigs[id];
    }
    return BusChainConfig{};
}

DynamicConfig::I2CDeviceConfig DynamicConfig::getMagEncoder(uint8_t id) const {
    std::lock_guard<std::mutex> lock(configMutex);
    if (id < magEncoderConfigs.size()) {
        return magEncoderConfigs[id];
    }
    return I2CDeviceConfig{};
}

DynamicConfig::I2CDeviceConfig DynamicConfig::getMagTracker(uint8_t id) const {
    std::lock_guard<std::mutex> lock(configMutex);
    if (id < magTrackerConfigs.size()) {
        return magTrackerConfigs[id];
    }
    return I2CDeviceConfig{};
}

DynamicConfig::IMUConfig DynamicConfig::getIMU(uint8_t id) const {
    std::lock_guard<std::mutex> lock(configMutex);
    if (id < imuConfigs.size()) {
        return imuConfigs[id];
    }
    return IMUConfig{};
}

DynamicConfig::I2CDeviceConfig DynamicConfig::getServoDriver(uint8_t id) const {
    std::lock_guard<std::mutex> lock(configMutex);
    if (id < servoDriverConfigs.size()) {
        return servoDriverConfigs[id];
    }
    return I2CDeviceConfig{};
}

DynamicConfig::ServoConfig DynamicConfig::getServo(uint8_t id) const {
    std::lock_guard<std::mutex> lock(configMutex);
    if (id < servoConfigs.size()) {
        return servoConfigs[id];
    }
    return ServoConfig{};
}

DynamicConfig::FOCMotorConfig DynamicConfig::getFOCMotor(uint8_t id) const {
    std::lock_guard<std::mutex> lock(configMutex);
    if (id < focMotorConfigs.size()) {
        return focMotorConfigs[id];
    }
    return FOCMotorConfig{};
}

void DynamicConfig::beginBusChain(BusChainConfig config, BusChain& busChain) {
    std::lock_guard<std::mutex> lock(configMutex);
    busChain.begin(config.moduleIds.data(), &i2cBuses[config.bus]);
}

bool DynamicConfig::beginI2CDevice(const I2CDeviceConfig config, I2CDevice& i2cDevice) {
    std::lock_guard<std::mutex> lock(configMutex);
    bool res;
    if (config.onBusChain) {
        res = i2cDevice.begin(config.channel, &busChains[config.busId]);
    } else {
        res = i2cDevice.begin(&i2cBuses[config.busId]);
    }
    return res;
}

bool DynamicConfig::beginIMU(const IMUConfig config, IMU& imu) {
    // Sets IMU parameters
    imu.setParameters(imuAccelRanges[config.accelMode], imuGyroRanges[config.gyroMode], imuFilterBands[config.filterMode]);
    // Begins IMU
    return beginI2CDevice(config, imu);
}

std::string DynamicConfig::describeBusChain(const BusChainConfig config) const {
    std::lock_guard<std::mutex> lock(configMutex);
    std::string result = " on bus ";
    result = result + std::to_string(config.bus) + " with modules ";
    for (uint8_t i = 0; i < config.moduleIds.size(); i++) {
        result = result + std::to_string(config.moduleIds[i]);
        if (i < config.moduleIds.size() - 1) {
            result = result + ", ";
        }
    }
    return result;
}

std::string DynamicConfig::describeI2CDevice(const I2CDeviceConfig config) const {
    std::lock_guard<std::mutex> lock(configMutex);
    std::string busChainStr;
    std::string channelStr;
    if (config.onBusChain) {
        busChainStr = "on BusChain";
        channelStr = "channel " + std::to_string(config.channel);
    } else {
        busChainStr = "on direct bus";
        channelStr = "";
    }
    return busChainStr + std::to_string(config.busId) + channelStr;
}