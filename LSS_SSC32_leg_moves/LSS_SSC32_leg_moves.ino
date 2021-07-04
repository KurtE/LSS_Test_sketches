//====================================================================================================
// Kurts Test program to try out different ways to manipulate the AX12 servos on the PhantomX
// This is a test, only a test...
//
// This version for Robotis OpenCM9.04
//====================================================================================================
//============================================================================
// Global Include files
//=============================================================================
#include <LSS.h>
//=============================================================================
// Options...
//=============================================================================
#define LSS_SERIAL Serial1
#define LSS_BAUD   500000
#define MAX_SERVO_NUM 32
#define LSS_ID    0
#define DEFAULT_FRAMES_PER_SECOND  50  // used for code that bypass Servo firmware

// Could use on systems with normal Serial port
//#define SSC32_SERIAL Serial2
#define USE_USB_RC_SERVOS
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
//#define SSC32_SERIAL Serial2
bool userial_prev = false;
bool use_em_control = false;
#else
#undef USE_USB_RC_SERVOS
#endif
#endif


//=============================================================================
// Define differnt robots..
//=============================================================================

// Protocol version

// Constants
/* Servo IDs */
#define     RF_COXA       2
#define     RF_FEMUR      4
#define     RF_TIBIA      6

#define     RM_COXA      14
#define     RM_FEMUR     16
#define     RM_TIBIA     18

#define     RR_COXA       8
#define     RR_FEMUR     10
#define     RR_TIBIA     12

#ifdef SERVO1_SPECIAL
#define     LF_COXA       19
#else
#define     LF_COXA       1
#endif
#define     LF_FEMUR      3
#define     LF_TIBIA      5

#define     LM_COXA      13
#define     LM_FEMUR     15
#define     LM_TIBIA     17

#define     LR_COXA       7
#define     LR_FEMUR      9
#define     LR_TIBIA     11

#ifdef TURRET
#define     TURRET_ROT    20
#define     TURRET_TILT   21
#endif

static const byte pgm_axdIDs[] = {
  LF_COXA, LF_FEMUR, LF_TIBIA,
#ifndef QUAD_MODE
  LM_COXA, LM_FEMUR, LM_TIBIA,
#endif
  LR_COXA, LR_FEMUR, LR_TIBIA,
  RF_COXA, RF_FEMUR, RF_TIBIA,
#ifndef QUAD_MODE
  RM_COXA, RM_FEMUR, RM_TIBIA,
#endif
  RR_COXA, RR_FEMUR, RR_TIBIA
#ifdef TURRET
  , TURRET_ROT, TURRET_TILT
#endif
};

#define NUM_SERVOS ((int)(sizeof(pgm_axdIDs)/sizeof(pgm_axdIDs[0])))
const char* IKPinsNames[] = {
  "LFC", "LFF", "LFT",
#ifndef QUAD_MODE
  "LMC", "LMF", "LMT",
#endif
  "LRC", "LRF", "LRT",
  "RFC", "RFF", "RFT",
#ifndef QUAD_MODE
  "RMC", "RMF", "RMT",
#endif
  "RRC", "RRF", "RRT",
#ifdef TURRET
  "T-ROT", "T-TILT"
#endif
};

//====================================
//set MJS RF config Gait Test Values
// and Mucked up by KJE ;)
//====================================
typedef struct {
  uint8_t         id;
  LSS_ConfigGyre  gyre;
  int16_t         offset;
  int16_t         max_speed;
  uint8_t         ssc32_pin;
  uint8_t         ssc32_inv;  // phoenix
  LSS_Status      move_status;
  int32_t         time_position;
} servo_info_t;
typedef struct {
  const char    *leg_name;
  servo_info_t  coxa;
  servo_info_t  femur;
  servo_info_t  tibia;
  bool          leg_found;
} leg_info_t;

leg_info_t legs[] = {
  {"Left Front",  {LF_COXA, LSS_GyreClockwise, 0, 600, 24, 0}, {LF_FEMUR, LSS_GyreClockwise, -104, 600, 25, 0}, {LF_TIBIA, LSS_GyreClockwise, -137, 600, 26, 0}},
  {"Left Middle", {LM_COXA, LSS_GyreClockwise, 0, 600, 20, 0}, {LM_FEMUR, LSS_GyreClockwise, -104, 600, 21, 0}, {LM_TIBIA, LSS_GyreClockwise, -137, 600, 21, 0}},
  {"Left Rear",   {LF_COXA, LSS_GyreClockwise, 0, 600, 16, 0}, {LR_FEMUR, LSS_GyreClockwise, -104, 600, 17, 0}, {LR_TIBIA, LSS_GyreClockwise, -137, 600, 18, 0}},

  {"Right Front",  {RF_COXA, LSS_GyreCounterClockwise, 8, 600, 1}, {RF_FEMUR, LSS_GyreCounterClockwise, -104, 600, 9, 1}, {RF_TIBIA, LSS_GyreCounterClockwise, -137, 600, 10, 1}},
  {"Right Middle", {RM_COXA, LSS_GyreCounterClockwise, 4, 600, 1}, {RM_FEMUR, LSS_GyreCounterClockwise, -104, 600, 5, 1}, {RM_TIBIA, LSS_GyreCounterClockwise, -137, 600, 6, 1}},
  {"Right Rear",   {RF_COXA, LSS_GyreCounterClockwise, 0, 600, 1}, {RR_FEMUR, LSS_GyreCounterClockwise, -104, 600, 1, 1}, {RR_TIBIA, LSS_GyreCounterClockwise, -137, 600, 2, 1}}
};
#define COUNT_LEGS (sizeof(legs)/sizeof(legs[0]))


#define RF_COXA_MaxSpeed  600
#define RF_COXA_Gyre      LSS_GyreCounterClockwise
#define RF_COXA_Offset      0

#define RF_FEMUR_MaxSpeed  600
#define RF_FEMUR_Gyre      LSS_GyreCounterClockwise
#define RF_FEMUR_Offset    -104

#define RF_TIBIA_MaxSpeed  600
#define RF_TIBIA_Gyre      LSS_GyreCounterClockwise
#define RF_TIBIA_Offset    -137

//Time delays for Gait MoveT commands
uint16_t delay1 = 450;
uint16_t servo_move_time = 500; //250;

// When doing cycle stances how to do each move?
enum {CYCLE_MOVET = 0, CYCLE_MOVES, CYCLE_MOVE};
uint8_t g_cycle_move_type = CYCLE_MOVET; // defauult for ow...


//LSS myLSS = LSS(LSS_ID);

#define RF_STANCE_COUNT 7
int16_t rf_stance_smooth [RF_STANCE_COUNT][3] =
{
  {0, -600, -600},       //Low
  // 0, -900, 510 degrees
  {100, -300, -300},      //mid Low
  {200,  0,  0},          //Med
  {0,  300, 300},         //High
  { -100,  0,  0},        //Med
  { -200, -300, -300},    //mid Low
  {0, -600, -600}       //Low
};

elapsedMillis emServos;

int move_time = 500; // start off time
int move_time_change = 250; // how much to change per loop

static const int MOVE_TIME_MAX = 1000;
static const int MOVE_TIME_MIN = 250;
bool ssc_connected = false;
#define cPwmDiv       991  //old 1059;
#define cPFConst      592  //old 650 ; 900*(1000/cPwmDiv)+cPFConst must always be 1500
// A PWM/deg factor of 10,09 give cPwmDiv = 991 and cPFConst = 592
// For a modified 5645 (to 180 deg travel): cPwmDiv = 1500 and cPFConst = 900.

void inline ConvertAndOutputSSCServoPos(uint8_t servo_id, int pos, int invert) {
  if (invert) pos = -pos;
  int32_t  pwm_pos = ((long)(pos + 900)) * 1000 / cPwmDiv + cPFConst;
  userial.printf("#%uP%u", servo_id, pwm_pos);
}

void setup()
{
  while (!Serial && millis() < 5000) ;
  Serial.println("Servo continuous time moves test");
  Serial.println("Press any key to pause test");
#if defined(ARDUINO_TEENSY36) || defined(__IMXRT1062__)
  myusb.begin();
#endif

  Serial.println("Cycle Smooth - speed changes each loop");
  Serial.println("Waiting for SSC-32...");
}



void loop()
{
#if defined(USE_USB_RC_SERVOS)
  myusb.Task();
  if (userial) {

    if (!userial_prev) {
      userial_prev = true;
      userial.begin(115200);
      Serial.println("USB Serial connected");
      ssc_connected = true;
      for (uint8_t leg = 0; leg < COUNT_LEGS; leg++) {
        ConvertAndOutputSSCServoPos(legs[leg].coxa.ssc32_pin, rf_stance_smooth[0][0], legs[leg].coxa.ssc32_inv);
        ConvertAndOutputSSCServoPos(legs[leg].femur.ssc32_pin, rf_stance_smooth[0][1], legs[leg].femur.ssc32_inv);
        ConvertAndOutputSSCServoPos(legs[leg].tibia.ssc32_pin, rf_stance_smooth[0][2], legs[leg].tibia.ssc32_inv);
      }
      userial.printf("T%u\r", move_time);
      delay(2000);
      emServos = 0;  // First one should start soon...
    }
  } else {
    if (userial_prev) {
      Serial.println("Waiting for SSC-32...");
      userial_prev = false;
    }
    return;
  }
#endif
  if (Serial.available()) {
    Serial.println("Test Paused, press any key to continue");
    while (Serial.read() != -1) ;
    while (Serial.read() == -1) ;
    while (Serial.read() != -1) ;
   emServos = 0;  // First one should start soon...
  }

  Serial.printf("Cycle Move Times %d\n", move_time);
  for (uint8_t position = 0; (position < RF_STANCE_COUNT) && !Serial.available(); position++) {
    for (uint8_t leg = 0; leg < COUNT_LEGS; leg++) {
      ConvertAndOutputSSCServoPos(legs[leg].coxa.ssc32_pin, rf_stance_smooth[position][0], legs[leg].coxa.ssc32_inv);
      ConvertAndOutputSSCServoPos(legs[leg].femur.ssc32_pin, rf_stance_smooth[position][1], legs[leg].femur.ssc32_inv);
      ConvertAndOutputSSCServoPos(legs[leg].tibia.ssc32_pin, rf_stance_smooth[position][2], legs[leg].tibia.ssc32_inv);
    }
    while (emServos < (uint32_t)move_time) ; // wait for the right time
    userial.printf("T%u\r", move_time);
    emServos = 0;
  }
  if ((move_time == MOVE_TIME_MAX) || (move_time == MOVE_TIME_MIN)) move_time_change = -move_time_change;
  move_time += move_time_change;
}

