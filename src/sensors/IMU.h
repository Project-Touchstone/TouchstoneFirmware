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
#include "../comms/BusChain.h"

//Accelerometer range in Gs
#define ACCEL_RANGE MPU6050_RANGE_2_G
//Gyro range in degrees per second
#define GYRO_RANGE MPU6050_RANGE_250_DEG
//Filter bandwidth frequency
#define FILTER_BAND MPU6050_BAND_260_HZ

class IMU {
    private:
        //IMU object
        Adafruit_MPU6050 imu;

        //BusChain object
        BusChain* busChain;
        //Sensor port on BusChain
        uint8_t sensorPort;

        //I2C port
        TwoWire *i2cPort;

        //Spinlock for RTOS
        portMUX_TYPE* spinlock;

    public:
        IMU();
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