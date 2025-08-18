/**
  IMU.h - Gets IMU data
  Created by Carson G. Ray
*/

#ifndef IMU_h
#define IMU_h

//External imports
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

//Local imports
#include "BusChain.h"

class IMU {
    private:
        //IMU object
        Adafruit_MPU6050 imu;

        //Accelerometer range
        mpu6050_accel_range_t accelRange = MPU6050_RANGE_2_G;
        //Gyro range
        mpu6050_gyro_range_t gyroRange = MPU6050_RANGE_250_DEG;
        //Filter bandwidth
        mpu6050_bandwidth_t filterBand = MPU6050_BAND_260_HZ;

        //Sensor I2C address
        uint8_t i2cAddress = 0x68;

        //BusChain object
        BusChain* busChain;
        //Sensor port on BusChain
        uint8_t sensorPort;

        //I2C port
        TwoWire *i2cPort;

        //Spinlock for RTOS
        portMUX_TYPE* spinlock;

        sensors_event_t a, g, temp;

        // Whether using BusChain
        bool busChainEnable = false;

    public:
        IMU();
        bool begin(uint8_t sensorPort, BusChain *busChain);
        bool begin(TwoWire* wire);
        void setParameters(mpu6050_accel_range_t accelRange, mpu6050_gyro_range_t gyroRange, mpu6050_bandwidth_t filterBand);
        void update();
        void getAccel(float *x, float *y, float *z);
        void getGyro(float *x, float *y, float *z);
        void getTemp(float *temp);
        void getRawAccel(int16_t* x, int16_t* y, int16_t* z);
        void getRawGyro(int16_t* x, int16_t* y, int16_t* z);
        void getRawTemp(int16_t* temp);
};

#endif