
//====================================================================================================
// Kurts Test program to try out different ways to manipulate the AX12 servos on the PhantomX
// This is a test, only a test...
//
// This version for Lynxmotion LSS servos
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
#define COUNT_SERVOS 32
#define LSS_ID    0

// 0 = don't 1 = after 2 = before
#define WHEN_TO_CHEW 2

//=============================================================================
// Globals
//=============================================================================
// Global objects

LSS myLSS = LSS(LSS_ID);


uint8_t g_ids[COUNT_SERVOS];
uint8_t g_count_servos_found = 0;

// by servo ids...
int servo_gyre[] = {0,
  LSS_GyreClockwise, LSS_GyreCounterClockwise, LSS_GyreClockwise, LSS_GyreCounterClockwise, LSS_GyreClockwise, LSS_GyreCounterClockwise,
  LSS_GyreClockwise, LSS_GyreCounterClockwise, LSS_GyreClockwise, LSS_GyreCounterClockwise, LSS_GyreClockwise, LSS_GyreCounterClockwise,
  LSS_GyreClockwise, LSS_GyreCounterClockwise, LSS_GyreClockwise, LSS_GyreCounterClockwise, LSS_GyreClockwise, LSS_GyreCounterClockwise
};


//====================================================================================================
// forward reference
//====================================================================================================
extern int16_t LSS_Read_Servo_s16(uint8_t &id, char *cmd);

//====================================================================================================
// Setup
//====================================================================================================
void setup() {
  while (!Serial && (millis() < 3000)) ;  // Give time for Teensy and other USB arduinos to create serial port
  Serial.begin(38400);  // start off the serial port.
  Serial.println("\nLSS Servo Test program");


  delay(1000);
  // Lets start of trying to locate all servos.
  // Initialize the LSS bus
  LSS::initBus(LSS_SERIAL, LSS_BAUD);
#ifdef LSS_SupportsSettingTimeouts
  LSS::setReadTimeouts(20, 5); // define we will wait for 20ms for response to start and 5ms for in message characters
#endif
  RebootServos(); // make sure they all are reset...
  delay(2000);

  FindServos();
}


//====================================================================================================
// Loop
//====================================================================================================
void loop() {

  // lets toss any charcters that are in the input queue
  while (Serial.read() != -1)  ;
  Serial.println("Press any key to start moving servos");
  while (Serial.read() == -1) ;
  while (Serial.read() != -1)  ;
  FindServos();
  MoveAllServos();

  RebootServos(); // make sure they all are reset...
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

  static int MIN_SERVO_POS = -300;
  static int MAX_SERVO_POS = 300;
  int servo_angle = 0;
  int servo_increment = 50;

  int voltages[COUNT_SERVOS];
  int temps[COUNT_SERVOS];
  uint32_t move_time = 250;
  Serial.println("Move All servos: Enter any key to exit");
  while (Serial.read() != -1);

  while (!Serial.available()) {
    elapsedMillis em = 0;
    servo_angle += servo_increment;
    if (servo_angle >= MAX_SERVO_POS) servo_increment = -50;
    if (servo_angle <= MIN_SERVO_POS) servo_increment = 50;
#if WHEN_TO_CHEW == 2
    // Lets ask for Voltage and Temp
    Serial1.print("#254QV0QT0\r");
#endif

    for (int j = 0; j < g_count_servos_found; j++) {
      Serial1.printf("#%uD%dT%u", g_ids[j], servo_angle, move_time);
    }
    Serial1.print("\r"); // output terminator to execute the command.
    char cmd_str[10];
    uint8_t responses_remaining = g_count_servos_found * 2;
    uint8_t servo_index = 0;
#if WHEN_TO_CHEW == 1
    // Lets ask for Voltage and Temp
    Serial1.print("#254QV0QT0\r");
#endif
#if WHEN_TO_CHEW > 0
    while (responses_remaining && (em < 2 * move_time)) {
      uint8_t servo_id;
      int16_t servo_value = LSS_Read_Servo_s16(servo_id, cmd_str);
      if (servo_id != g_ids[servo_index]) {
        for (servo_index = 0; servo_index < g_count_servos_found; servo_index++) {
          if (servo_id == g_ids[servo_index]) break;
        }
      }
      if (servo_index < g_count_servos_found) {
        if (strcmp(cmd_str, "QV") == 0) voltages[servo_index] = servo_value;
        else if (strcmp(cmd_str, "QT") == 0) temps[servo_index] = servo_value;
        responses_remaining--;
        servo_index++;
        if (servo_index == g_count_servos_found) servo_index = 0;
      }
    }
#endif
    Serial.printf("servo_angle:%d QT: %u RR:%u\n", servo_angle, (uint32_t)em, responses_remaining);
    while (em < (2 * move_time)) ;
  }
}


//=======================================================================================
void RebootServos() {
  Serial.println("Sending Broadcast RESET");
  LSS::genericWrite(LSS_BroadcastID, LSS_ActionReset); // Tell all of the servos to reset
  delay(2000);
  Serial.println("Reset Sent");
}



//=======================================================================================

void FindServos(void) {

  g_count_servos_found = 0;
  int32_t pos;
  Serial.println("\nSearch for all servos");

  // Initialize to no servos...
  for (int i = 0; i < COUNT_SERVOS; i++) {
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
  // Now lets update the slots.
  Serial1.printf("#254SLOTCOUNT%u\r", g_count_servos_found);
  for (int i = 0; i < g_count_servos_found; i++) {
    LSS_SERIAL.printf("#%uSLOT%d\r", g_ids[i], i);
    LSS_SERIAL.printf("#%uG%u\r",  g_ids[i], servo_gyre[g_ids[i]]);

  }
  Serial.println("  Done Slots and gyre updated");
}


//==============================================================================
char receive_buffer[256];

char * LSS_ReceiveServoString(uint8_t &id)
{

  if (!(Serial1.find(LSS_CommandReplyStart)))
  {
    return ((char *) nullptr);
  }

  // Ok we have the * now now lets get the servo ID from the message.
  uint8_t readID = 0;
  int c;
  bool valid_field = 0;
  while ((c = LSS::timedRead()) >= 0)
  {
    if ((c < '0') || (c > '9')) break;  // not a number character
    readID = readID * 10 + c - '0';
    valid_field = true;
  }

  if (!valid_field)
  {
    return ((char *) nullptr);
  }

  id = readID;  // pass it back.

  // We don't know for sure which command, so will simply read in rest of buffer.
  size_t index = 0;
  while (index < sizeof(receive_buffer))
  {
    c = LSS::timedRead();
    if (c < 0 || c == LSS_CommandEnd[0])
      break;
    receive_buffer[index] = (char) c;
    index++;
  }
  receive_buffer[index] = '\0';

  // Return value (success)
  if (c < 0)
  {
    // did not get the ending CR
    return ((char *) nullptr);
  }

  return receive_buffer;
}

//==============================================================================
int16_t LSS_Read_Servo_s16(uint8_t &id, char *cmd)
{
  // Let the string function do all of the main parsing work.
  char *valueStr = LSS_ReceiveServoString(id);
  *cmd = 0;  // so caller can find where the CMD started..
  if (valueStr == (char *) nullptr) return 0;

  // convert the value string to value
  // Lets skip over everything that is likely cmd...
  while (*valueStr && !((*valueStr >= '0') && (*valueStr <= '9'))) *cmd++ = valueStr++;
  *cmd = 0; // NULL terminate.
  int16_t value = 0;
  int16_t value_sign = 1;
  for (;;)
  {
    if ((*valueStr >= '0') && (*valueStr <= '9'))
      value = value * 10 + *valueStr - '0';
    else if (*valueStr == '-')
      value_sign = -1;
    else
      break;
    valueStr++;
  }
  // now see if we exited with valid number
  if (*valueStr != '\0') return 0;
  // return the computed value
  return value * value_sign;
}
