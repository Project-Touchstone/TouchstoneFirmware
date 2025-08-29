/**
 * DyanmicConfig.h - Dynamic hardware peripheral configuration framework
 * Created by Carson G. Ray
 */

#ifndef DYNAMIC_CONFIG_H
#define DYNAMIC_CONFIG_H

//External imports
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <vector>
#include <string>
#include <mutex>

// Local library imports
#include "I2CDevice.h"
#include "IMU.h"
#include "BusChain.h"

class DynamicConfig {
    public:
        struct BusChainConfig {
            uint8_t bus; // I2C bus number (0 or 1)
            std::vector<uint8_t> moduleIds; // addresses of BusChain modules on bus
        };
        struct I2CDeviceConfig {
            bool onBusChain; // Whether sensor is on BusChain
            uint8_t busId; // buschain id (or bus if not on BusChain)
            uint8_t channel; // Channel on BusChain
        };
        struct IMUConfig : public I2CDeviceConfig {
            uint8_t accelMode;
            uint8_t gyroMode;
            uint8_t filterMode;
        };
        struct ServoConfig {
            uint8_t servoDriverId; // Servo driver id
            uint8_t channel; // Channel on servo driver
        };
        struct FOCMotorConfig {
            uint8_t port; // Built-in motor driver port (0 or 1)
        };

        void setI2CBuses(TwoWire* buses);
        void setBusChains(BusChain* busChains);

        std::size_t addBusChain(const BusChainConfig config);
        std::size_t addMagEncoder(const I2CDeviceConfig config);
        std::size_t addMagTracker(const I2CDeviceConfig config);
        std::size_t addIMU(const IMUConfig config);
        std::size_t addServoDriver(const I2CDeviceConfig config);
        std::size_t addServo(const ServoConfig config);
        std::size_t addFOCMotor(const FOCMotorConfig config);

        uint8_t numBusChains() const;
        uint8_t numMagEncoders() const;
        uint8_t numMagTrackers() const;
        uint8_t numIMUs() const;
        uint8_t numServos() const;
        uint8_t numServoDrivers() const;
        uint8_t numFOCMotors() const;
        uint8_t getSensorDataLength() const;

        BusChainConfig getBusChain(uint8_t id) const;
        I2CDeviceConfig getMagEncoder(uint8_t id) const;
        I2CDeviceConfig getMagTracker(uint8_t id) const;
        IMUConfig getIMU(uint8_t id) const;
        I2CDeviceConfig getServoDriver(uint8_t id) const;
        ServoConfig getServo(uint8_t id) const;
        FOCMotorConfig getFOCMotor(uint8_t id) const;

        void beginBusChain(uint8_t id);
        bool beginIMU(uint8_t id, IMU& imu);
        bool beginI2CDevice(const I2CDeviceConfig config, I2CDevice& i2cDevice);
    
    private:
        // Mutex for thread safety
        mutable std::mutex configMutex;

        // Pointer to I2C buses
        TwoWire* i2cBuses;

        // Pointer to BusChain objects
        BusChain* busChains;

        // Vector of bus chain configurations
        std::vector<BusChainConfig> busChainConfigs;

        // Vectors for I2C device configurations
        std::vector<I2CDeviceConfig> magEncoderConfigs;
        std::vector<I2CDeviceConfig> magTrackerConfigs;
        std::vector<IMUConfig> imuConfigs;
        std::vector<I2CDeviceConfig> servoDriverConfigs;

        // Vector of servo configurations
        std::vector<ServoConfig> servoConfigs;

        // Vector of FOC motor configurations
        std::vector<FOCMotorConfig> focMotorConfigs;

        // IMU parameters
        mpu6050_accel_range_t imuAccelRanges[4] = {
            MPU6050_RANGE_2_G, MPU6050_RANGE_4_G, MPU6050_RANGE_8_G, MPU6050_RANGE_16_G
        };
        mpu6050_gyro_range_t imuGyroRanges[4] = {
            MPU6050_RANGE_250_DEG, MPU6050_RANGE_500_DEG, MPU6050_RANGE_1000_DEG, MPU6050_RANGE_2000_DEG
        };
        mpu6050_bandwidth_t imuFilterBands[4] = {
            MPU6050_BAND_260_HZ, MPU6050_BAND_184_HZ, MPU6050_BAND_94_HZ, MPU6050_BAND_44_HZ
        };

        // Sensor data packet lengths
        const uint8_t magEncoderLen = 2;
        const uint8_t magTrackerLen = 6;
        const uint8_t imuLen = 12;
};

#endif