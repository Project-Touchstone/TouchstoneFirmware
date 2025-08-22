#ifndef TestConfigMap_h
#define TestConfigMap_h

#include "../utils/DynamicConfig.h"

DynamicConfig::BusChainConfig busChainConfigs[] = {
    {0, {0}},
    {1, {1}}
};

DynamicConfig::I2CDeviceConfig magEncoderConfigs[] = {
    {false, 0, 0},
    {false, 1, 0}
};

DynamicConfig::I2CDeviceConfig magTrackerConfigs[] = {
    {true, 0, 0},
    {true, 0, 1}
};

DynamicConfig::I2CDeviceConfig imuConfigs[] ={
    {true, 0, 0},
    {true, 0, 1}
};

void loadConfig(DynamicConfig& config) {
    // Add BusChain configs
    for (const auto& bc : busChainConfigs) {
        config.addBusChain(bc);
    }
    // Add magnetic encoder configs
    for (const auto& me : magEncoderConfigs) {
        config.addMagEncoder(me);
    }
    // Add magnetic tracker configs
    for (const auto& mt : magTrackerConfigs) {
        config.addMagTracker(mt);
    }
    // Add IMU configs
    for (const auto& imu : imuConfigs) {
        config.addIMU(imu);
    }
};

#endif