#include "ServoController.h"
#include "BusChain.h"
#include "MagSensor.h"
#include "SerialInterface.h"
#include <math.h>
#include <Wire.h>

// Cores to pin RTOS tasks to
#define CORE_0 0
#define CORE_1 1

// I2C pins
#define I2C_SDA_0 21
#define I2C_SCL_0 22
#define I2C_SDA_1 32
#define I2C_SCL_1 33

// BusChain address identifiers
uint8_t BUSCHAIN_0 = 0;
uint8_t BUSCHAIN_1 = 1;

// PWM output pin on channel 0 of servo driver used for servo synchronization
#define interruptPin 5

// Servo driver port index
#define servoDriverPort 3

// DRIFT motors to configure
#define NUM_MOTORS 4

namespace SerialHeaders {
  //Headers from master to controller

	//Pings microcontroller
	#define PING 0x1
	//Requests sensor data from microcontroller
	#define REQUEST_DATA 0x2
  //Servo power update
  #define SERVO_POWER 0x3

	//Headers from controller to master

  //Acknowledges ping      
  #define PING_ACK 0x1
  //Sends sensor data
  #define SENSOR_DATA 0x2
  //Sensor data ready
  #define DATA_READY 0xA
  //PWM cycle start
  #define PWM_CYCLE 0xB                                                                                                        
}

using namespace SerialHeaders;

// Servo channels for DRIFT motors
const uint8_t servoChannels[NUM_MOTORS] = {0, 1, 2, 3};

// Encoder ports on BusChain (servo, spool) per DRIFT motor
const uint8_t encoderPorts[NUM_MOTORS][2] = {{0, 1}, {7, 6}, {8, 9}, {5, 4}};

// BusChain objects
BusChain busChains[2];

// Sensor objects
MagSensor* magSensors;

// List of sensor numbers on each buschain
uint8_t sensorsByBus[2][8];

// Number of sensors on each bus
uint8_t sensorCountByBus[2] = {0, 0};

// Define task handles
TaskHandle_t schedulerHandles[2] = {NULL, NULL};
TaskHandle_t pwmCycleHandle;
TaskHandle_t sensorReadHandles[2] = {NULL, NULL};
TaskHandle_t servoControllerHandle;
TaskHandle_t serialInterfaceHandle;

// Task and interrupt function prototypes
void IRAM_ATTR onPWMStart();
void TaskScheduler(void *pvParameters);
void TaskPWMCycle(void *pvParameters);
void TaskSensorRead(void *pvParameters);
void TaskServoController(void *pvParameters);
void TaskSerialInterface(void *pvParameters);

// Define sensor data queue
typedef uint8_t sensorID_t;
QueueHandle_t sensorDataQueue;

uint64_t startTime;

// The setup function runs once when you press reset or power on the board.
void setup() {
  // Initialize serial communication at 921600 bits per second:
  SerialInterface::begin(115200);
  
  // Initialize I2C ports
  Wire.begin(I2C_SDA_0, I2C_SCL_0);
  Wire1.begin(I2C_SDA_1, I2C_SCL_1);

  // Sets bus parameters
  Wire.setTimeout(1000);
	Wire.setClock(1000000);
  Wire1.setTimeout(1000);
	Wire1.setClock(1000000);

  //Initializes buschain objects
  busChains[0].begin(&BUSCHAIN_0, &Wire);
  busChains[1].begin(&BUSCHAIN_1, &Wire1);

  //Initializes servo driver board, flags connection error
  if (!ServoController::begin(servoDriverPort%8, &busChains[servoDriverPort>>3])) {
    Serial.println("Error connecting to servo driver");
    while (true) {
      
    }
  }

  //Initializes encoders in order of id
  MagSensor* sensors = new MagSensor[NUM_MOTORS * 2];
  for (uint8_t i = 0; i < NUM_MOTORS*2; i++) {
    // Finds encoder port for sensor id
    uint8_t encoderPort = encoderPorts[i/2][i%2];
    uint8_t bus = encoderPort>>3;

    // Adds sensor id to list by bus
    sensorsByBus[bus][sensorCountByBus[bus]++] = i;

    // Attempts to connect through associated port and buschain
    if (!sensors[i].begin(encoderPort%8, &busChains[bus])) {
      Serial.print("Error connecting to encoder port: ");
      Serial.println(encoderPort);
      while (true) {

      }
    }
  }
  magSensors = sensors;

  sensorDataQueue = xQueueCreate(NUM_MOTORS*2, sizeof(sensorID_t));

  xTaskCreatePinnedToCore(TaskSerialInterface, "Serial Interface", 2048, NULL, 5, &serialInterfaceHandle, tskNO_AFFINITY);
  
  xTaskCreatePinnedToCore(TaskServoController, "Servo Controller", 2048, NULL, 4, &servoControllerHandle, servoDriverPort>>3);

  xTaskCreatePinnedToCore(TaskSensorRead, "Sensor Read 0", 2048, (void *)0, 3, &sensorReadHandles[0], 0);
  xTaskCreatePinnedToCore(TaskSensorRead, "Sensor Read 1", 2048, (void *)1, 3, &sensorReadHandles[1], 1);

  xTaskCreatePinnedToCore(TaskScheduler, "Scheduler 0", 2048, (void *)0, 1, &schedulerHandles[0], 0);
  xTaskCreatePinnedToCore(TaskScheduler, "Scheduler 1", 2048, (void *)1, 1, &schedulerHandles[1], 1);
  
  xTaskCreatePinnedToCore(TaskPWMCycle, "PWM Cycle", 2048, NULL, 2, &pwmCycleHandle, tskNO_AFFINITY);

  attachInterrupt(interruptPin, onPWMStart, RISING);
}

// Called on rising pwm interrupt pin
void IRAM_ATTR onPWMStart() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  //Updates pwm cycle timer for servo synchronization
  ServoController::updatePWMTime();

  // Notifies pwm cycle task to wake
  vTaskNotifyGiveFromISR(pwmCycleHandle, &xHigherPriorityTaskWoken);

  // Does context switching if notification wakes higher priority task than current
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );  
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/
void TaskScheduler(void *pvParameters) {
  uint8_t core = *((uint8_t *) pvParameters);
  for (;;) {
    if (Serial.available() > 0) {
      // If serial data needs to be received, yields to serial interface
      xTaskNotifyGive(serialInterfaceHandle);
    } else {
      //Otherwise yields to sensor reading
      xTaskNotifyGive(sensorReadHandles[core]);
    }
    vTaskDelay(1);
  }
}

void TaskPWMCycle(void *pvParameters) {
  (void)pvParameters;
  for (;;) {
    // Waits for notification from interrupt
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Sends PWM cycle start header
    SerialInterface::sendByte(PWM_CYCLE);

    // Updates PWM ranges based on servo powers
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      ServoController::updatePWMCompute(servoChannels[i]);
    }
  }
}

void TaskSensorRead(void *pvParameters) {
  uint8_t bus = *((uint8_t *) pvParameters);
  for (;;) {
    for (uint8_t i = 0; i < sensorCountByBus[bus]; i++) {
      //Gets sensor id
      sensorID_t sensorID = sensorsByBus[bus][i];
      // Waits for notification from scheduler before every I2C transaction
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      magSensors[sensorID].update();

      // Adds sensor data to queue
      xQueueSend(sensorDataQueue, &sensorID, 0);

      // Sends data ready header
      SerialInterface::sendByte(DATA_READY);
    }
  }
}

void TaskServoController(void *pvParameters) {
  (void)pvParameters;
  for (;;) {
    // Waits for notification from serial interface that powers have been updated
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Sends PWM ranges to PWM driver over I2C
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      ServoController::updatePWMDriver(servoChannels[i]);
    }
  }
}

void TaskSerialInterface(void *pvParameters) {
  (void)pvParameters;
  for (;;) {
    // Waits for notification from scheduler that serial data is available
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (SerialInterface::processHeader()) {
      switch (SerialInterface::getHeader()) {
        case PING:
          // Sends ping acknowledgement
          SerialInterface::sendByte(PING_ACK);
          SerialInterface::clearHeader();
          break;
        case REQUEST_DATA:
          //Sends sensor data in queue
          //Sends data header
          SerialInterface::sendByte(SENSOR_DATA);
          while (uxQueueMessagesWaiting(sensorDataQueue) > 0) {
            sensorID_t sensorID;
            xQueueReceive(sensorDataQueue, &sensorID, 0);
            // Sends sensor id
            SerialInterface::sendByte(sensorID);
            // Sends sensor data
            SerialInterface::sendData<int16_t>(magSensors[sensorID].rawY());
            SerialInterface::sendData<int16_t>(magSensors[sensorID].rawZ());
            // Sends end of data frame
            SerialInterface::sendEnd();
          }
          SerialInterface::clearHeader();
          break;
        case SERVO_POWER:
          // Updates servo controller
          if (Serial.available() > 5 && !SerialInterface::isEnded()) {
            // Reads servo number and power
            uint8_t servoNum = SerialInterface::readByte();
            float power = SerialInterface::readFloat();
            ServoController::setPower(servoChannels[servoNum], power);
          } else if (SerialInterface::isEnded()) {
            // Clears header if end of data frame reached
            SerialInterface::clearHeader();
            // Notifies servo controller to update PWM ranges
            xTaskNotifyGive(servoControllerHandle);
          }
          break;
      }
    }
  }
}

void loop() {

}
