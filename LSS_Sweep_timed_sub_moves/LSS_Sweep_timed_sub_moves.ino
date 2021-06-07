#include <LSS.h>
// ID set to default LSS ID = 0
#define LSS_ID		(0)
#define LSS_BAUD	(LSS_DefaultBaud)
#define SERVO_MAX 1800
#define SERVO_MIN -1800
#define NUMBER_STEPS 36
#define DELTA_PER_MOVE ((SERVO_MAX - SERVO_MIN)/ NUMBER_STEPS)
#define TIME_PER_STEP (4000/NUMBER_STEPS)

#define USE_USB_RC_SERVOS
#define RC_SERVO_MIN 600
#define RC_SERVO_MAX 2400

// Choose the proper serial port for your platform
//#define LSS_SERIAL	(Serial)	// ex: Many Arduino boards
#define LSS_SERIAL	(Serial1)	// ex: Teensy

#if defined( USE_USB_RC_SERVOS)
#if defined(ARDUINO_TEENSY36) || defined(__IMXRT1062__)
#include "USBHost_t36.h"
#define USBBAUD 115200
uint32_t baud = USBBAUD;
uint32_t format = USBHOST_SERIAL_8N1;
USBHost myusb;
USBHub hub1(myusb);
USBHub hub2(myusb);
USBHIDParser hid1(myusb);
USBHIDParser hid2(myusb);
USBSerial userial(myusb);  // works only for those Serial devices who transfer <=64 bytes (like T3.x, FTDI...)
bool userial_prev = false;
#else
#undef USE_USB_RC_SERVOS
#endif
#endif


LSS myLSS = LSS(LSS_ID);
void setup()
{
  while (!Serial && millis() < 5000) ;
  Serial.println("Servo continuous time moves test");
  Serial.println("Press any key to pause test");
#if defined(ARDUINO_TEENSY36) || defined(__IMXRT1062__)
  myusb.begin();
#endif

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

void moveTSSC32(uint8_t servo, int pos, uint32_t time) {
#if defined(USE_USB_RC_SERVOS)
  int pos_pulse = map(pos, SERVO_MIN, SERVO_MAX, RC_SERVO_MIN, RC_SERVO_MAX);
  userial.printf("#%uP%uT%u\r\n", servo, pos_pulse, time);
  userial.flush();
#endif
}

void loop()
{
#if defined(USE_USB_RC_SERVOS)
  myusb.Task();
  if (userial && !userial_prev) {
    userial_prev = true;
    userial.begin(115200);
    Serial.println("USB Serial connected");
    moveTSSC32(0, SERVO_MIN, 2000);
    delay(2000);
  }
#endif
  for (int servo_pos = SERVO_MIN; servo_pos < SERVO_MAX; servo_pos += DELTA_PER_MOVE) {
    pause_test();
    myLSS.moveT(servo_pos, TIME_PER_STEP);
    moveTSSC32(0, servo_pos, TIME_PER_STEP);
    delay(TIME_PER_STEP);
  }
  for (int servo_pos = SERVO_MAX; servo_pos > SERVO_MIN; servo_pos -= DELTA_PER_MOVE) {
    pause_test();
    myLSS.moveT(servo_pos, TIME_PER_STEP);
    moveTSSC32(0, servo_pos, TIME_PER_STEP);
    delay(TIME_PER_STEP);
  }
}
