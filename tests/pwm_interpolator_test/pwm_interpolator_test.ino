//Servo pin
#define servoPin 5

const uint16_t interResolution = 5;
const uint64_t servoInterval = 20000;
const uint64_t rangeCenter = 1500;
const uint64_t deadZone = 30;
const uint64_t rangeLength = 500;
const uint16_t rangeInterval = 5;

uint64_t pwmLevel[2] = {1500, 1460};
uint8_t criticalCount = 1;
volatile bool pwmState = true;
volatile uint16_t pulseCount = 0;

hw_timer_t* timer0 = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux0);
  digitalWrite(servoPin, pwmState);
  if (pwmState) {
    if (pulseCount < criticalCount) {
      timerAlarm(timer0, pwmLevel[1], false, 0);
    } else {
      timerAlarm(timer0, pwmLevel[0], false, 0);
    }
    pulseCount++;
    if (pulseCount == interResolution - 1) {
      pulseCount = 0;
    }
  } else {
    timerAlarm(timer0, servoInterval, true, 0);
  }
  pwmState = !pwmState;
  portEXIT_CRITICAL_ISR(&timerMux0);
}

void setup() {
  Serial.begin(115200);

  pinMode(servoPin, OUTPUT);

  timer0 = timerBegin(1000000);
  timerAttachInterrupt(timer0, &onTimer);
  timerAlarm(timer0, 1000000, true, 0);
}

void loop() {
  for (uint64_t speed = rangeCenter+deadZone; speed < rangeCenter+deadZone+rangeLength; speed++) {
    uint8_t intervals = (speed-(rangeCenter+deadZone))/rangeInterval;
    pwmLevel[0] = rangeCenter + deadZone + intervals*rangeInterval;
    pwmLevel[1] = pwmLevel[0] + rangeInterval;
    criticalCount = (speed - pwmLevel[0])*interResolution/rangeInterval;
    Serial.println(speed);
    delay(100);
  }
}
