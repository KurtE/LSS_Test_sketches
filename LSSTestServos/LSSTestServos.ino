
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
#define LSS_SERIAL_PORT     Serial1
#define LSS_BAUDRATE    500000
#define LSS_ID          0

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
  //  Serial.println("8 - Set ID: <old> <new>");
  Serial.println("9 - Print Servo Values");
  Serial.println("b - Baud <new baud>");
  Serial.println("t - Toggle track Servos");
  Serial.println("h - hold [<sn>]");
  Serial.println("f - free [<sn>]");
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
      case 'h':
      case 'H':
        HoldOrFreeServos(1);
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
} LSSQLIST;

const LSSQLIST query_list[] = {
  {LSQ_S16, LSS_QueryStatus},
  {LSQ_S16, LSS_QueryOriginOffset},
  {LSQ_S16, LSS_QueryAngularRange},
  {LSQ_S16, LSS_QueryPositionPulse},
  {LSQ_S16, LSS_QueryPosition},
  {LSQ_S16, LSS_QuerySpeed},
  {LSQ_S16, LSS_QuerySpeedRPM},
  {LSQ_S16, LSS_QuerySpeedPulse},
  {LSQ_S16, LSS_QueryMaxSpeed},
  {LSQ_S16, LSS_QueryMaxSpeedRPM},
  {LSQ_S16, LSS_QueryColorLED},
  {LSQ_S16, LSS_QueryGyre},
  {LSQ_S16, LSS_QueryID},
  {LSQ_S16, LSS_QueryBaud},
  {LSQ_STR, LSS_QueryFirstPosition},
  {LSQ_STR, LSS_QueryModelString},
  {LSQ_STR, LSS_QuerySerialNumber},
  {LSQ_S16, LSS_QueryFirmwareVersion},
  {LSQ_S16, LSS_QueryVoltage},
  {LSQ_S16, LSS_QueryTemperature},
  {LSQ_S16, LSS_QueryCurrent},
  {LSQ_S16, LSS_QueryAnalog},
  {LSQ_S16, LSS_QueryAngularStiffness},
  {LSQ_S16, LSS_QueryAngularHoldingStiffness},
  {LSQ_S16, LSS_QueryAngularAcceleration},
  {LSQ_S16, LSS_QueryAngularDeceleration},
  {LSQ_S16, LSS_QueryEnableMotionControl},
  {LSQ_S16, LSS_QueryBlinkingLED}
};


void PrintServoValues(void) {

  int32_t wID;
  if (!FGetNextCmdNum(&wID))
    return;
  Serial.printf("\nServo %u values\n", wID);
  for (uint8_t i = 0; i < (sizeof(query_list) / sizeof(query_list[0])); i++) {
    // Variables
    int16_t value = 0;

    // Ask servo for status; exit if it failed
    if (!(LSS::genericWrite(wID, query_list[i].str, LSS_QuerySession)))
    {
      Serial.printf("  Failed genericWrite %s\n", query_list[i]);
      break;
    }

    // Read response from servo
    if (query_list[i].lsqrt == LSQ_S16) {
      value = (int16_t) LSS::genericRead_Blocking_s16(wID, query_list[i].str);
      LSS_LastCommStatus comm_status = myLSS.getLastCommStatus();
      if (comm_status != LSS_CommStatus_ReadSuccess) {
        Serial.printf("  %s - %d failed(%d)\n", query_list[i].str, value, (uint32_t)comm_status);
      } else {
        Serial.printf("  %s - %d\n", query_list[i].str, value);
      }

    } else {
      const char *valueStr = LSS::genericRead_Blocking_str(wID, query_list[i].str);
      LSS_LastCommStatus comm_status = myLSS.getLastCommStatus();
      if (comm_status != LSS_CommStatus_ReadSuccess) {
        Serial.printf("  %s - %s failed(%d)\n", query_list[i].str, valueStr, (uint32_t)comm_status);
      } else {
        Serial.printf("  %s - %s\n", query_list[i].str, valueStr);
      }

    }


  }

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
