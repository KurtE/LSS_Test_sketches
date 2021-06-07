#include <LSS.h>

// ID set to default LSS ID = 0
#define LSS_ID		(0)
#define LSS_BAUD	(LSS_DefaultBaud)
// Choose the proper serial port for your platform
//#define LSS_SERIAL	(Serial)	// ex: Many Arduino boards
#define LSS_SERIAL	(Serial1)	// ex: Teensy


LSS myLSS = LSS(LSS_ID);
#define SERVO_MAX 1800
#define SERVO_MIN -1800
#define NUMBER_STEPS 36
#define DELTA_PER_MOVE ((SERVO_MAX - SERVO_MIN)/ NUMBER_STEPS)
#define TIME_PER_STEP (4000/NUMBER_STEPS)
void setup()
{
  while (!Serial && millis() < 5000) ;
  Serial.println("Servo continuous time moves test");
  Serial.println("Press any key to pause test");
  // Initialize the LSS bus
  LSS::initBus(LSS_SERIAL, LSS_BAUD);

  // Initialize LSS to position 0.0 °
  myLSS.move(SERVO_MIN);

  // Wait for it to get there
  delay(2000);
}

void pause_test() {
  if (Serial.available()) {
    Serial.println("Test Paused, press any key to continue");
    while (Serial.read() != -1) ;
    while (Serial.read() == -1) ;
    while (Serial.read() != -1) ;
  }
}
void loop()
{
  for (int servo_pos = SERVO_MIN; servo_pos < SERVO_MAX; servo_pos += DELTA_PER_MOVE) {
    pause_test();
    myLSS.moveT(servo_pos, TIME_PER_STEP);
    delay(TIME_PER_STEP);
  }
  for (int servo_pos = SERVO_MAX; servo_pos > SERVO_MIN; servo_pos -= DELTA_PER_MOVE) {
    pause_test();
    myLSS.moveT(servo_pos, TIME_PER_STEP);
    delay(TIME_PER_STEP);
  }
}
