#ifndef SerialHeaders_h
#define SerialHeaders_h

namespace SerialHeaders {
	//Headers from master to controller

	//Pings microcontroller
	#define PING 0x1 // 0 bytes
	//Servo power update
	#define SERVO_SIGNAL 0x2 // 3 bytes (1 byte motor id, 2 byte signal value from -1 to 1)

	//Headers from controller to master

	//Acknowledges ping      
	#define PING_ACK 0x1 // 0 bytes
	//PWM cycle start
	#define PWM_CYCLE 0x2 // 0 bytes
	//Sends magnetic encoder data
	#define MAGENCODER_DATA 0xA0 // 5 bytes (1 byte sensor id, 4 byte Y and Z axes)
	//Sends magnetic tracker data
	#define MAGTRACKER_DATA 0xA1 // 7 bytes (1 byte sensor id, 6 byte X, Y and Z axes)
	//Sends IMU data
	#define IMU_DATA 0xA2 // 13 bytes (1 byte sensor id, 6 byte 3-axis accel data, 6 byte 3-axis gyro data)                                                                                           
}

#endif // SerialHeaders_h