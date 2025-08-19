#ifndef DYNAMIC_CONFIG_H
#define DYNAMIC_CONFIG_H

//External imports
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <vector>
#include <string>

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
        struct ServoConfig {
            uint8_t servoDriverId; // Servo driver id
            uint8_t channel; // Channel on servo driver
        };
        struct FOCMotorConfig {
            uint8_t port; // Built-in motor driver port (0 or 1)
        };

        struct IMUParams {
            mpu6050_accel_range_t accelRange;
            mpu6050_gyro_range_t gyroRange;
            mpu6050_bandwidth_t filterBand;
        };

        void addBusChain(BusChainConfig config);
        void addI2CDevice(I2CDeviceConfig config);
        void addServo(ServoConfig config);
        void addFOCMotor(FOCMotorConfig config);

        uint8_t numBusChains();
        uint8_t numMagEncoders();
        uint8_t numMagTrackers();
        uint8_t numIMUs();
        uint8_t numServoDrivers();

        BusChainConfig getBusChain(uint8_t id);
        I2CDeviceConfig getMagEncoder(uint8_t id);
        I2CDeviceConfig getMagTracker(uint8_t id);
        I2CDeviceConfig getIMU(uint8_t id);
        I2CDeviceConfig getServoDriver(uint8_t id);
        ServoConfig getServo(uint8_t id);
        FOCMotorConfig getFOCMotor(uint8_t id);

        void beginBusChain(BusChainConfig config, BusChain busChain);
        void beginI2CDevice(std::string name, I2CDeviceConfig config, I2CDevice i2cDevice);

        void beginAllBusChains(std::vector<BusChain>* busChains);
        void beginAllI2CDevices(std::vector<I2CDevice>* i2cDevices);
    
    private:
        // Vector of bus chain configurations
        std::vector<BusChainConfig> busChainConfigs;

        // Vectors for I2C device configurations
        std::vector<I2CDeviceConfig> magEncoderConfigs;
        std::vector<I2CDeviceConfig> magTrackerConfigs;
        std::vector<I2CDeviceConfig> imuConfigs;
        std::vector<I2CDeviceConfig> servoDriverConfigs;

        // Vector of servo configurations
        std::vector<ServoConfig> servoConfig;

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
        IMUParams imuParams;
}

#endif