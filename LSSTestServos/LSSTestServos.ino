
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
#define LSS_BAUD   250000
#define MAX_SERVO_NUM 32
#define LSS_ID    0
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




//=============================================================================
// Globals
//=============================================================================
// Global objects

LSS myLSS = LSS(LSS_ID);


uint32_t           g_wVoltage;
uint8_t        g_servo_index_voltage = 0;
char           g_aszCmdLine[80];
uint8_t        g_iszCmdLine;
boolean        g_fTrackServos = false;

uint8_t g_ids[MAX_SERVO_NUM];
uint8_t g_count_servos_found = 0;

// Values to use for servo position...
byte          g_bServoID;
int32_t          g_wServoGoalPos;
uint32_t          g_wServoMoveTime;

//====================================================================================================
// forward reference
//====================================================================================================
extern bool IsValidServo(uint8_t servo_id);
extern void initMemoryUsageTest();
extern void PrintServoValues(void);
extern void PrintServoValues2(void);
extern void SetBaudRate();

//====================================================================================================
// Setup
//====================================================================================================
void setup() {
  while (!Serial && (millis() < 3000)) ;  // Give time for Teensy and other USB arduinos to create serial port
  Serial.begin(38400);  // start off the serial port.
  initMemoryUsageTest();
  Serial.println("\nLSS Servo Test program");


  delay(1000);
  // Lets start of trying to locate all servos.
  // Initialize the LSS bus
  LSS::initBus(LSS_SERIAL, LSS_BAUD);

  FindServos();

  PrintServoVoltage();

  tmpSpeed(300);

}


//====================================================================================================
// Loop
//====================================================================================================
void loop() {
  // Output a prompt
  PrintServoVoltage();
  //  printMemoryUsage();

  // lets toss any charcters that are in the input queue
  while (Serial.read() != -1)
    ;

  Serial.println("0 - All Servos off");
  Serial.println("1 - All Servos center");
  Serial.println("2 - Set Servo position [<Servo>] <Position> [<Time>]");
  Serial.println("3 - Set Servo Angle");
  Serial.println("4 - Get Servo Positions");
  Serial.println("5 - Find All Servos");
  //  Serial.println("6 - Set Servo return delay time");
  Serial.println("7 - Set ID: <old> <new>");
  Serial.println("9 - Print Servo Values");
  Serial.println("9 - Print Servo Values fill tx");
  Serial.println("b - Baud <new baud>");
  Serial.println("t - Toggle track Servos");
  Serial.println("h - hold [<sn>]");
  Serial.println("f - free [<sn>]");
  Serial.println("g - Gait Sim RF");
  Serial.println("m - move all servos");
  Serial.println("q - test/time Q command");
  Serial.println("r - Reboot [<sn>]");

  Serial.print(":");
  Serial.flush();  // make sure the complete set of prompts has been output...
  // Get a command
  if (GetCommandLine()) {
    Serial.println("");
    Serial.print("Cmd: ");
    Serial.println(g_aszCmdLine);
    g_iszCmdLine = 1;  // skip over first byte...
    switch (g_aszCmdLine[0]) {
      case '0':
        AllServosOff();
        break;
      case '1':
        AllServosCenter();
        break;
      case '2':
        SetServoPosition();
        break;
      case '3':
        break;
      case '4':
        GetServoPositions();
        break;
      case '5':
        FindServos();
        break;
      case '7':  
        SetServoID();
        break;
      case '8':
        PrintServoValues2();
        break;
      case '9':
        PrintServoValues();
        break;
      case 'b':
      case 'B':
        SetBaudRate();
        break;
      case 'f':
      case 'F':
        HoldOrFreeServos(0);
        break;
      case 'g':
	  case 'G':
        generateGait();
        break;
      case 'h':
      case 'H':
        HoldOrFreeServos(1);
        break;
      case 'q':
      case 'Q':
        QueryAllServos();
        break;
      case 'r':
      case 'R':
        RebootServos();
        break;
      case 't':
      case 'T':
        g_fTrackServos = !g_fTrackServos;
        if (g_fTrackServos) {
          Serial.println("Tracking On");
          TrackServos(true);  // call to initialize all of the positions.
        }
        else
          Serial.println("Tracking Off");
        TrackPrintMinsMaxs();
        break;
      case 'm':
      case 'M':
        MoveAllServos();
        break;
      case 'w':
      case 'W':
        WriteServoValues();
        break;
    }
  }
}

//====================================================================================================
void PrintServoVoltage() {
  return; // bypass for now
  // Lets try reading in the current voltage for the next servo we found...
  if (g_count_servos_found == 0) return; // no servos found
  Serial.println("PrintServo Voltage called");
  g_servo_index_voltage++;    // will wrap around...
  if (g_servo_index_voltage >= g_count_servos_found) g_servo_index_voltage = 0;
  myLSS.setServoID(g_ids[g_servo_index_voltage]);

  uint16_t wNewVoltage = myLSS.getVoltage();
  if (wNewVoltage != g_wVoltage) {
    g_wVoltage = wNewVoltage;
    Serial.printf("Servo: %u(%u) Voltage: %u\n", g_ids[g_servo_index_voltage],
                  g_servo_index_voltage, g_wVoltage);
  }
}


//====================================================================================================
// Helper function to read in a command line
uint8_t GetCommandLine(void) {
  int ch;
  uint8_t ich = 0;
  g_iszCmdLine = 0;

  for (;;) {
    // throw away any thing less than CR character...
    ch = Serial.read();
    if ((ch >= 10) && (ch <= 15)) {
      g_aszCmdLine[ich] = 0;
      return ich;
    }
    if (ch != -1)
      g_aszCmdLine[ich++] = ch;

    if (g_fTrackServos)
      TrackServos(false);
  }
}

//=======================================================================================
boolean FGetNextCmdNum(int32_t * pw ) {
  // Skip all leading none number characters...
  while (((g_aszCmdLine[g_iszCmdLine] < '0') || (g_aszCmdLine[g_iszCmdLine] > '9')) 
      && (g_aszCmdLine[g_iszCmdLine] != '-')) {
    if (g_aszCmdLine[g_iszCmdLine] == 0)
      return false;  // end of the line...
    g_iszCmdLine++;
  }
  *pw = 0;
  int32_t sign = 1;
  if (g_aszCmdLine[g_iszCmdLine] == '-') {
    sign = -1;
    g_iszCmdLine++;
  }
  
  while ((g_aszCmdLine[g_iszCmdLine] >= '0') && (g_aszCmdLine[g_iszCmdLine] <= '9')) {
    *pw = *pw * 10 + (g_aszCmdLine[g_iszCmdLine] - '0');
    g_iszCmdLine++;
  }
  *pw *= sign;
  return true;
}


//=======================================================================================
void AllServosOff(void) {
  // Quick and dirty way to do it by broadcast...

  // See if we can do by broadcast
  LSS::genericWrite(LSS_BroadcastID, LSS_ActionLimp); // Tell all of the servos to go limp
}

void AllServosOn(void) {
  // Quick and dirty way to do it by broadcast...

  // See if we can do by broadcast
  LSS::genericWrite(LSS_BroadcastID, LSS_ActionHold); // Tell all of the servos to go limp
}

//=======================================================================================
void AllServosCenter(void) {
  // Tell all servos to turn on.
  AllServosOn();
  LSS::genericWrite(LSS_BroadcastID, LSS_ActionMove, 0,
                    LSS_ActionParameterTime, 500);  // move in half second
}

//=======================================================================================
void MoveAllServos(void) {
  // first move all to center and on
  AllServosCenter();

  static int MIN_SERVO_POS = -200;
  static int MAX_SERVO_POS = 200; 
  int servo_angle = 0;
  int servo_increment = 5;

  int positions[NUM_SERVOS];
  int voltages[NUM_SERVOS];
  int temps[NUM_SERVOS];
  int index_print = 0;
  Serial.println("Move All servos: Enter any key to exit");
  while (Serial.read() != -1);

  while (!Serial.available()) {
    elapsedMicros em = 0;
    servo_angle += servo_increment;
    if (servo_angle >= MAX_SERVO_POS) servo_increment = -5;
    if (servo_angle <= MIN_SERVO_POS) servo_increment = 5;
    for (int j = 0; j < NUM_SERVOS; j++) {
      myLSS.setServoID(pgm_axdIDs[j]);    // So first is which servo
      myLSS.moveT(servo_angle, 100);
    }
    uint32_t time_send_positions = em;
    for (int j = 0; j < NUM_SERVOS; j++) {
      myLSS.setServoID(pgm_axdIDs[j]);    // So first is which servo
      positions[j] = myLSS.getPosition();
      voltages[j] = myLSS.getVoltage();
      temps[j] = myLSS.getTemperature();
    }
    uint32_t time_loop = em;
    // Now lets look how long it took plus print out some of it...
    Serial.printf("%u %u (%u) P:%d V:%u T:%u ", time_send_positions, time_loop, 1000000/time_loop,
        servo_angle, voltages[index_print], temps[index_print]);
    if (++index_print == NUM_SERVOS) index_print = 0;

    for (int j = 0; j < NUM_SERVOS; j++) {
      Serial.printf(" %u:%d", j, positions[j]);
    }
    // BUGBUG:: quick test to see how long it would take with 3 query in one...
    em = 0;
    for (int j = 0; j < NUM_SERVOS; j++) {
      myLSS.setServoID(pgm_axdIDs[j]);    // So first is which servo
      LSS_SERIAL.printf("#%uQD\r#%uQV\r#%uQT\r", pgm_axdIDs[j], pgm_axdIDs[j], pgm_axdIDs[j]);
      uint8_t cnt_left = 3;
      elapsedMicros em_timeout;
      while (cnt_left && em_timeout < 5000) {
        if (LSS_SERIAL.read() == '\r') cnt_left--;
      }
    }

    Serial.printf("TQS:%u\n", (uint32_t)em);

    while (em < 100) ;
  }
}


//=======================================================================================
void HoldOrFreeServos(byte fHold) {
  int32_t iServo;
  if (!FGetNextCmdNum(&iServo)) {
    if (fHold) AllServosOn();
    else AllServosOff();
  }
  else {
    if (fHold)
      LSS::genericWrite(iServo, LSS_ActionHold); // Tell all of the servos to hold
    else
      LSS::genericWrite(iServo, LSS_ActionLimp); // Tell all of the servos to go limp
  }
}

//=======================================================================================
//=======================================================================================
void RebootServos() {
  LSS::genericWrite(LSS_BroadcastID, LSS_ActionReset); // Tell all of the servos to reset
}

//=======================================================================================
void SetServoPosition(void) {
  int32_t w1;
  int32_t w2;
  g_wServoMoveTime = 0;
  if (!FGetNextCmdNum(&w1))
    return;    // no parameters so bail.

  Serial.println("Set Servo Position");
  if (FGetNextCmdNum(&g_wServoGoalPos)) {  // We have at least 2 parameters
    myLSS.setServoID(w1);    // So first is which servo
    if (FGetNextCmdNum(&w2)) {  // We have at least 3 parameters
      g_wServoMoveTime = w2;
    }
  } else {
    g_wServoGoalPos = w1;
  }

  Serial.printf(" ID: %d %POS: %d", myLSS.getServoID(), g_wServoGoalPos);
  if (g_wServoMoveTime) {
    Serial.printf(" Time: %d\n", g_wServoMoveTime);
    myLSS.moveT(g_wServoGoalPos, g_wServoMoveTime);
  } else {
    Serial.print("\r");
    myLSS.move(g_wServoGoalPos);
  }
}

//=======================================================================================
bool IsValidServo(uint8_t servo_id) {
  return true;
}

void SetServoID(void) {
  int32_t wIDFrom;
  int32_t wIDTo;

  if (!FGetNextCmdNum(&wIDFrom))
    return;    // no parameters so bail.

  if (!FGetNextCmdNum(&wIDTo))
    return;    // no parameters so bail.


  Serial.print("Set Servo ID From: ");
  Serial.print(wIDFrom, DEC);
  Serial.print(" To: ");
  Serial.println(wIDTo, DEC);

  if (!IsValidServo(wIDFrom)) {
    Serial.print("Servo: ");
    Serial.print(wIDFrom, DEC);
    Serial.println("Was not found");
    return;
  }

  // Now lets try to update the servo ID
  LSS_SERIAL.printf("#%uCID%u\r", wIDFrom, wIDTo);
  LSS_SERIAL.flush(); 
  delay(250);
  LSS_SERIAL.printf("#254RESET\r");
  LSS_SERIAL.flush(); 
//  myLSS.setServoID(254);    // So first is which servo
//  myLSS.reset();
  delay(1000);  // wait for servos to reset. 
}



//=======================================================================================
void GetServoPositions(void) {

  unsigned long ulBefore;
  unsigned long ulDelta;
  int32_t pos;


  if (!g_count_servos_found) {
    Serial.println("Previous Find Servos failed to locate any servos: so retry");
    FindServos();
    return;
  }

  for (int i = 0; i < g_count_servos_found; i++) {
    myLSS.setServoID(g_ids[i]);
    Serial.print(g_ids[i], DEC);
    Serial.print(":");
    ulBefore = micros();
    pos = myLSS.getPosition();
    ulDelta = micros() - ulBefore;
    Serial.print(pos, DEC);
    Serial.print(" ");
    Serial.println(ulDelta, DEC);
  }
}

void QueryAllServos() {
  LSS_Status servo_status[MAX_SERVO_NUM];
  uint32_t query_time[MAX_SERVO_NUM];

  if (!g_count_servos_found) {
    Serial.println("Previous Find Servos failed to locate any servos: so retry");
    FindServos();
    return;
  }

  Serial.print("\nDo Query(Q) command on all servos ");
  elapsedMicros emTotal = 0;
  for (int i = 0; i < g_count_servos_found; i++) {
    elapsedMicros em = 0;
    myLSS.setServoID(g_ids[i]);
    servo_status[i] = myLSS.getStatus();
    query_time[i] = em;
  }
  Serial.printf("total time: %u\n", (uint32_t)emTotal);
  for (int i = 0; i < g_count_servos_found; i++) {
    Serial.printf("  %u: %u T:%u\n", g_ids[i], servo_status[i], query_time[i]);
  }
}


//=======================================================================================

void FindServos(void) {

  g_count_servos_found = 0;
  int32_t pos;
  Serial.println("\nSearch for all servos");

  // Initialize to no servos...
  for (int i = 0; i < MAX_SERVO_NUM; i++) {
    myLSS.setServoID(i);
    pos = myLSS.getPosition();
    if (myLSS.getLastCommStatus() == LSS_CommStatus_ReadSuccess) {
      g_ids[g_count_servos_found++] = i;
      Serial.print("    ");
      Serial.print(i, DEC);
      Serial.print(" - ");
      Serial.println(pos, DEC);
    }
  }
  Serial.println("  Done");
}


//=======================================================================================
int g_asPositionsPrev[255];
int g_asMins[255];
int g_asMaxs[255];

void TrackServos(boolean fInit) {
  int pos;
  bool fSomethingChanged = false;

  // Lets only use the found servos.
  for (int i = 0; i < g_count_servos_found; i++) {
    myLSS.setServoID(g_ids[i]);
    pos = myLSS.getPosition();
    if (myLSS.getLastCommStatus() == LSS_CommStatus_ReadSuccess) {

      if (fInit) {
        g_asMins[i] = pos;
        g_asMaxs[i] = pos;
      }
      if (pos != g_asPositionsPrev[i]) {
        if (!fInit) {
          // only print if we moved more than some deltas
          if (abs(pos - g_asPositionsPrev[i]) > 5) {
            for (int j = 0; j < NUM_SERVOS; j++) {
              if (g_ids[i] == pgm_axdIDs[j]) {
                Serial.print(IKPinsNames[j]);
                break;
              }
            }
            Serial.print("(");
            Serial.print(g_ids[i], DEC);
            Serial.print("):");
            Serial.print(pos, DEC);
            /*          Serial.print("(");
                      Serial.print((((long)(w - 512)) * 375L) / 128L, DEC);
                      Serial.print(") "); */
            Serial.print(" ");
            fSomethingChanged = true;
          }
        }
        g_asPositionsPrev[i] = pos;
        if (g_asMins[i] > pos)
          g_asMins[i] = pos;

        if (g_asMaxs[i] < pos)
          g_asMaxs[i] = pos;
      }
    }
  }
  if (fSomethingChanged)
    Serial.println();
}

void TrackPrintMinsMaxs(void) {

  for (int i = 0; i < g_count_servos_found; i++) {
    Serial.print(g_ids[i], DEC);
    Serial.print(":");
    Serial.print(g_asMins[i], DEC);
    Serial.print(" - ");
    Serial.println(g_asMaxs[i], DEC);
  }
}


//=======================================================================================
typedef enum {
  LSQ_S16,
  LSQ_STR
} LSSQRT;

typedef struct {
  LSSQRT lsqrt;
  const char *str;
  int16_t param; 
} LSSQLIST;

const LSSQLIST query_list[] = {
  {LSQ_S16, LSS_QueryStatus, -1},
  {LSQ_S16, LSS_QueryOriginOffset, -1},
  {LSQ_S16, LSS_QueryAngularRange, -1},
  {LSQ_S16, LSS_QueryPositionPulse, -1},
  {LSQ_S16, LSS_QueryPosition, -1},
  {LSQ_S16, LSS_QuerySpeed, -1},
  {LSQ_S16, LSS_QuerySpeedRPM, -1},
  {LSQ_S16, LSS_QuerySpeedPulse, -1},
  {LSQ_S16, LSS_QueryMaxSpeed, -1},
  {LSQ_S16, LSS_QueryMaxSpeedRPM, -1},
  {LSQ_S16, LSS_QueryColorLED, -1},
  {LSQ_S16, LSS_QueryGyre, -1},
  {LSQ_S16, LSS_QueryID, -1},
  {LSQ_S16, LSS_QueryBaud, -1},
  {LSQ_STR, LSS_QueryFirstPosition, -1},
  {LSQ_STR, LSS_QueryModelString, -1},
//  {LSQ_STR, LSS_QuerySerialNumber, -1},
  {LSQ_S16, LSS_QueryFirmwareVersion, -1},
  {LSQ_S16, LSS_QueryVoltage, -1},
  {LSQ_S16, LSS_QueryTemperature, -1},
  {LSQ_S16, LSS_QueryCurrent, -1},
//  {LSQ_S16, LSS_QueryAnalog, 0},
  {LSQ_S16, LSS_QueryAngularStiffness, -1},
  {LSQ_S16, LSS_QueryAngularHoldingStiffness, -1},
  {LSQ_S16, LSS_QueryAngularAcceleration, -1},
  {LSQ_S16, LSS_QueryAngularDeceleration, -1},
  {LSQ_S16, LSS_QueryEnableMotionControl, -1},
  {LSQ_STR, LSS_QueryFirmwareVersion, 1},
  {LSQ_STR, LSS_QueryFirmwareVersion, 2},
  {LSQ_STR, LSS_QueryFirmwareVersion, 3}
//  {LSQ_S16, LSS_QueryBlinkingLED}
};


void PrintServoValues2(void) {
#if 1
  int32_t wID;
  if (!FGetNextCmdNum(&wID))
    return;
  Serial.printf("\nServo %u values\n", wID);
  LSS_SERIAL.setTimeout(10);  // 10ms timeout
  elapsedMicros emWholeList = 0;
  elapsedMillis emTimeout = 0;
  uint8_t index_command_tx = 0;
  uint8_t index_command_rx = 0;
  uint8_t stringBuffer[80];
  while (index_command_rx < (sizeof(query_list) / sizeof(query_list[0])) && emTimeout < 10) {
    if (index_command_tx < (sizeof(query_list) / sizeof(query_list[0]))) {
      if (Serial.availableForWrite() > 8) {
        bool success;
        if (query_list[index_command_tx].param == -1) success = LSS::genericWrite(wID, query_list[index_command_tx].str);
        else  success = LSS::genericWrite(wID, query_list[index_command_tx].str, query_list[index_command_tx].param );
        
        if (!success)
        {
          Serial.printf("  Failed genericWrite %s\n", query_list[index_command_tx]);
          break;
        }
        index_command_tx++;
      }
    }
    if (LSS_SERIAL.available()) {
      if (LSS_SERIAL.find("*")) {
        LSS_SERIAL.parseInt();
        LSS_SERIAL.readBytesUntil('\r', stringBuffer, sizeof(stringBuffer));
        if (memcmp(stringBuffer, query_list[index_command_rx].str, strlen( query_list[index_command_rx].str)) == 0) {
          Serial.printf("%s(%d) - %s\n", query_list[index_command_rx].str, query_list[index_command_rx].param, stringBuffer + strlen( query_list[index_command_rx].str));
        } else {
          Serial.printf("Return? %s != %s\n", query_list[index_command_rx].str, stringBuffer + strlen( query_list[index_command_rx].str));
        }
        index_command_rx++;
      }
      emTimeout = 0;
    }
  }
  Serial.printf("Total Time: %u\n", (uint32_t)emWholeList);
#endif
}

void PrintServoValues(void) {

  int32_t wID;
  int servo_index = 256;
  if (!FGetNextCmdNum(&wID))
    return;
  elapsedMicros emWholeList = 0;
  if (wID == -1 ) {
    servo_index = 0;
  }

  do {
    if (servo_index < 256) wID = g_ids[servo_index++];
    Serial.printf("\nServo %u values\n", wID);
    for (uint8_t i = 0; i < (sizeof(query_list) / sizeof(query_list[0])); i++) {
      // Variables
      int16_t value = 0;

      // Ask servo for status; exit if it failed
      elapsedMicros em = 0;
      bool success;
      if (query_list[i].param == -1) success = LSS::genericWrite(wID, query_list[i].str);
      else  success = LSS::genericWrite(wID, query_list[i].str, query_list[i].param );
      
      if (!success)
      {
        Serial.printf("  Failed genericWrite %s\n", query_list[i].str);
        break;
      }

      // Read response from servo
      if (query_list[i].lsqrt == LSQ_S16) {
        value = (int16_t) LSS::genericRead_Blocking_s16(wID, query_list[i].str);
        uint32_t delta_time = em;
        LSS_LastCommStatus comm_status = myLSS.getLastCommStatus();
        if (comm_status != LSS_CommStatus_ReadSuccess) {
          Serial.printf("  %s(%d) - %d failed(%d) t:%u\n", query_list[i].str, query_list[i].param, value, (uint32_t)comm_status, delta_time);
        } else {
          Serial.printf("  %s(%d) - %d t:%u\n", query_list[i].str, query_list[i].param, value, delta_time);
        }

      } else {
        const char *valueStr = LSS::genericRead_Blocking_str(wID, query_list[i].str);
        uint32_t delta_time = em;
        LSS_LastCommStatus comm_status = myLSS.getLastCommStatus();
        if (comm_status != LSS_CommStatus_ReadSuccess) {
          Serial.printf("  %s(%d) - %s failed(%d) t:%u\n", query_list[i].str, query_list[i].param, valueStr, (uint32_t)comm_status, delta_time);
        } else {
          Serial.printf("  %s(%d) - %s t:%u\n", query_list[i].str, query_list[i].param, valueStr, delta_time);
        }

      }


    }

    Serial.printf("Total Time: %u\n", (uint32_t)emWholeList);
  } while (servo_index < g_count_servos_found);
}

//=======================================================================================
void WriteServoValues() {
#ifdef LATER
  int32_t wID;
  int32_t wReg;
  int32_t wVal;
  uint8_t error;
  int retval;
  if (!FGetNextCmdNum(&wID))
    return;    // no parameters so bail.

  if (!IsValidServo(wID)) {
    Serial.print("Write register ID: ");
    Serial.print(wID, DEC);
    Serial.println(" Servo not found");
    return;
  }
  dynamixel::PortHandler *portHandler = portHandlers[g_servo_protocol[wID].b.port];


  if (!FGetNextCmdNum(&wReg))
    return;    // no parameters so bail.

  while (FGetNextCmdNum(&wVal)) {
    Serial.print("Write register ID: ");
    Serial.print(wID, DEC);
    Serial.print(" Reg: ");
    Serial.print(wReg, DEC);
    Serial.print(" Val: ");
    Serial.print(wVal, DEC);
    if (g_servo_protocol[wID].b.protocol == SERVO_PROTOCOL1) {
      retval = packetHandler1->write1ByteTxRx(portHandler, wID, wReg, wVal, &error);
    } else {
      retval = packetHandler2->write1ByteTxRx(portHandler, wID, wReg, wVal, &error);
    }
    if (!ReportAnyErrors(" Write Reg", wID, retval, error)) {
      Serial.println(" Success");
    } else {
      Serial.println();
    }
    wReg++;   // get to the next reg
  }
#endif
}





//=======================================================================================
void SetBaudRate()
{
  int32_t wBaud;

  if (!FGetNextCmdNum(&wBaud))
    return;    // no parameters so bail.
  Serial.print("Setting Baud to: ");
  Serial.println(wBaud);
  LSS::initBus(LSS_SERIAL, wBaud);

  Serial.println("Doing new Servo Scan");
  delay(2000);
  FindServos();
}

//=================================================================================
// Lets initialize our memory usage code, to get an idea of how much has been
// used
register uint8_t * stack_ptr asm("sp");
extern char end asm("end");

uint32_t g_end_stack_pointer;
uint32_t g_start_heap_pointer;

void initMemoryUsageTest()
{
  // Guess on start of stack. // probably using less than 100 bytes of stack space...
#if 0
  g_end_stack_pointer = ((uint32_t)stack_ptr + 100) & 0xfffff000;

  // get the start of the heap ponter
  g_start_heap_pointer = (uint32_t)&end;

  // Print out some memory information
  Serial.printf("Estimated global data size: %d\n", g_start_heap_pointer & 0xffff);
  //  Serial.printf("starting Heap info: start: %x current: %x\n", g_start_heap_pointer, (uint32_t)_sbrk(0));
  Serial.printf("Start Stack info: end: %x current: %x\n", g_end_stack_pointer, (uint32_t)stack_ptr);
  Serial.println("Try to init memory");
  Serial.flush(); // make sure it has chance to write out.
  uint8_t *sp_minus = stack_ptr - 10;  // leave a little slop
  for (uint8_t *p = (uint8_t*)_sbrk(0); p < sp_minus; p++) *p = 0xff; // init to ff
  Serial.println("After init memory");
#endif
}

//=================================================================================
void printMemoryUsage()
{
  //uint8_t *current_heap_ptr = (uint8_t*)_sbrk(0);
  //Serial.printf("Heap ptr: %x  Usage: %d\n", (uint32_t)current_heap_ptr,
  //              (uint32_t)current_heap_ptr - g_start_heap_pointer);
#if 0
  // stack info
  uint8_t *sp_minus = stack_ptr - 10;  // leave a little slop
  uint8_t *p = current_heap_ptr;

  // try to find out how far the stack has been used
  while ((p < sp_minus) && (*p == 0xff)) p++;
  Serial.printf("Stack Max: %x, usage: %d\n", p, g_end_stack_pointer - (uint32_t)p);
  Serial.printf("Estimated unused memory: %d\n", (uint32_t)(p - current_heap_ptr));
#endif
}

//================================================================================
// RF leg array pins
uint16_t rf_ids [] = { RF_COXA, RF_FEMUR, RF_TIBIA }; 
int16_t rf_rip6_gait [5][3] =
	{ 
		{-48, 11, 110}, 	//Start position 1 - start position
                      // -4.8, 1.1, 11 degrees
		{-26,  3,  51},		//position2
                      //-2.6, 0.3, 5.1
		{  0,  0,   0},		//position3
                      // 0, 0, 0
		{ 31,  3,  -44},		//position4
                      // 3.1, 0.3, -4.4
		{ 67, 10, -82}		//position5 - end position
                      // 6.7, 1, -8.2
	};

uint16_t delay1 = 50;
uint16_t servo_move_time = 250;
//=================================================================================
void generateGait()
{
  for(uint8_t count = 0; count < 5; count++) {
  	for(uint8_t position = 0; position < 5; position++) {
  		 for(uint8_t ids = 0; ids < 3; ids++){
  			myLSS.setServoID(rf_ids[ids]);
  			myLSS.moveT(rf_rip6_gait[position][ids], servo_move_time);
  		 }
       checkStatus();
       delay(delay1);
  	}
  }
/*
for(int i=0; i < 5; i++) {
	//Start position 1 - start position
	// -4.8, 1.1, 11 degrees
  myLSS.setServoID(RF_COXA);
  myLSS.moveT(-48, servo_move_time);
  myLSS.setServoID(RF_FEMUR);
  myLSS.moveT(11, servo_move_time);
  myLSS.setServoID(RF_TIBIA);
  myLSS.moveT(110, servo_move_time);
  //Serial.println("Position 1: ");
  checkStatus();
  //GetServoPositions();
  delay(delay1);
  
	//position2
	//-2.6, 0.3, 5.1
	myLSS.setServoID(RF_COXA);
	myLSS.moveT(-26,servo_move_time);
	myLSS.setServoID(RF_FEMUR);
	myLSS.moveT(3, servo_move_time);
	myLSS.setServoID(RF_TIBIA);
	myLSS.moveT(51, servo_move_time);
  //Serial.println("Position 2: ");
  checkStatus();
  //GetServoPositions();
  delay(delay1);
  
	//position3
	// 0, 0, 0
	myLSS.setServoID(RF_COXA);
	myLSS.moveT(0, servo_move_time);
	myLSS.setServoID(RF_FEMUR);
	myLSS.moveT(0, servo_move_time);
	myLSS.setServoID(RF_TIBIA);
	myLSS.moveT(0, servo_move_time);
  //Serial.println("Position 3: ");
  checkStatus();
  //GetServoPositions();
  delay(delay1);
  
	//position4
	// 3.1, 0.3, -4.4
	myLSS.setServoID(RF_COXA);
	myLSS.moveT(31, servo_move_time);
	myLSS.setServoID(RF_FEMUR);
	myLSS.moveT(3, servo_move_time);
	myLSS.setServoID(RF_TIBIA);
	myLSS.moveT(-44, servo_move_time);
  //Serial.println("Position 4: ");
  checkStatus();
  //GetServoPositions();
  delay(delay1);
  
	//position5 - end position
	// 6.7, 1, -8.2
	myLSS.setServoID(RF_COXA);
	myLSS.moveT(67, servo_move_time);
	myLSS.setServoID(RF_FEMUR);
	myLSS.moveT(10, servo_move_time);
	myLSS.setServoID(RF_TIBIA);
	myLSS.moveT(-82, servo_move_time);
  //Serial.println("Position 5: ");
  checkStatus();
  //GetServoPositions();
  delay(delay1);
}
*/

	//leg up start position
	//6.7, -31.2, -34.3
	myLSS.setServoID(RF_COXA);
	myLSS.moveT(67, servo_move_time);
	myLSS.setServoID(RF_FEMUR);
	myLSS.moveT(-312, servo_move_time);
	myLSS.setServoID(RF_TIBIA);
	myLSS.moveT(343, servo_move_time);
  //Serial.println("Position Start Leg Up: ");
  checkStatus();
  delay(delay1);
  GetServoPositions();
}

void checkStatus()
{
  int8_t status1 = -1, status2 = -1, status3 = -1;
  uint32_t statusTime = 0;
  while(status1 != 6 || status2 != 6 || status3 != 6) {
    myLSS.setServoID(RF_COXA);
    if(status1 != 6) status1 = myLSS.getStatus();
    myLSS.setServoID(RF_FEMUR);
    if(status2 != 6) status2 = myLSS.getStatus();
    myLSS.setServoID(RF_TIBIA);
    if(status3 != 6) status3 = myLSS.getStatus();
    delay(1);
    statusTime += 1;
  }
  //Serial.printf("Status: %d: %d, %d, %d\n", statusTime, status1, status2, status3);
}

void tmpSpeed(uint16_t temp)
{
  myLSS.setServoID(RF_COXA);
  myLSS.setMaxSpeed(temp, LSS_SetSession);
  myLSS.setServoID(RF_FEMUR);
  myLSS.setMaxSpeed(temp, LSS_SetSession);
  myLSS.setServoID(RF_TIBIA);
  myLSS.setMaxSpeed(temp, LSS_SetSession);

}
