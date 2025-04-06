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

        sensors_event_t a, g, temp;

    public:
        IMU();
        bool begin(uint8_t sensorPort, BusChain *busChain);
        void update();
        void getAccel(float *x, float *y, float *z);
        void getGyro(float *x, float *y, float *z);
        void getTemp(float *temp);
        void getRawAccel(int16_t* x, int16_t* y, int16_t* z);
        void getRawGyro(int16_t* x, int16_t* y, int16_t* z);
        void getRawTemp(int16_t* temp);
};

#endif