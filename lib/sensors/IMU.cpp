/**
  IMU.h - Gets IMU data
  Created by Carson G. Ray
*/

#include "IMU.h"

/// @brief Default constructor
IMU::IMU() {
	//Dynamic memory allocation for spinlock
	spinlock = (portMUX_TYPE*) malloc(sizeof(portMUX_TYPE));
	// Initialize the spinlock dynamically
	portMUX_INITIALIZE(spinlock);
	busChainEnable = false;
}

/// @brief Initializes imu object using BusChain
/// @param sensorPort Port number of sensor on BusChain
/// @param busChain BusChain object
/// @return true (successful), false (error)
bool IMU::begin(uint8_t sensorPort, BusChain* busChain) {
	this->sensorPort = sensorPort;
	this->busChain = busChain;

    // Ensures busChain is used
	busChainEnable = true;

	busChain->selectPort(sensorPort);
	bool ret = begin(busChain->getI2CPort());
	busChain->release();

	return ret;
}

/// @brief Initializes imu object using direct I2C
/// @param wire I2C port
/// @return true (successful), false (error)
bool IMU::begin(TwoWire* wire) {
	this->i2cPort = wire;
	
	bool ret = imu.begin(i2cAddress, i2cPort);
    if (ret) {
        imu.setAccelerometerRange(accelRange);
        imu.setGyroRange(gyroRange);
        imu.setFilterBandwidth(filterBand);
    }
	return ret;
}

void IMU::setParameters(mpu6050_accel_range_t accelRange, mpu6050_gyro_range_t gyroRange, mpu6050_bandwidth_t filterBand) {
    this->accelRange = accelRange;
    this->gyroRange = gyroRange;
    this->filterBand = filterBand;
}

/// @brief Updates sensor data
void IMU::update() {
    // If using BusChain, select the port before reading
  if (busChainEnable) {
    busChain->selectPort(sensorPort);
  }
  imu.getEvent(&a, &g, &temp);
  // Release the BusChain after reading
  if (busChainEnable) {
    busChain->release();
  }
}

void IMU::getAccel(float* x, float* y, float* z) {
    *x = a.acceleration.x;
    *y = a.acceleration.y;
    *z = a.acceleration.z;
}

void IMU::getGyro(float* x, float* y, float* z) {
    *x = g.gyro.x;
    *y = g.gyro.y;
    *z = g.gyro.z;
}

void IMU::getTemp(float* temp) {
    *temp = this->temp.temperature;
}

void IMU::getRawAccel(int16_t* x, int16_t* y, int16_t* z) {
    imu.getRawAccel(x, y, z);
}

void IMU::getRawGyro(int16_t* x, int16_t* y, int16_t* z) {
    imu.getRawGyro(x, y, z);
}

void IMU::getRawTemp(int16_t* temp) {
    imu.getRawTemp(temp);
}