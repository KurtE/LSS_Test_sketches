//====================================================================
//Project Lynxmotion Phoenix
//
// Servo Driver - This version is setup to use Lynxmotion type servos using the
//====================================================================
#include <Arduino.h>
#include <TeensyDebug.h>
#include <EEPROM.h>

#include <avr/pgmspace.h>

#include "LSS_Phoenix.h"
#include <LSS.h>


#ifdef c4DOF
#define NUMSERVOSPERLEG 4
#else
#define NUMSERVOSPERLEG 3
#endif

#ifdef cTurretRotPin
#define NUMSERVOS (NUMSERVOSPERLEG*CNT_LEGS +2)
#else
#define NUMSERVOS (NUMSERVOSPERLEG*CNT_LEGS)
#endif

#define cPwmMult      128
#define cPwmDiv       375
#define cPFConst      512    // half of our 1024 range

// Some defines for Voltage processing
#define VOLTAGE_MIN_TIME_UNTIL_NEXT_INTERPOLATE 4000  // Min time in us Until we should do next interpolation, as to not interfer.
#define VOLTAGE_MIN_TIME_BETWEEN_CALLS 150      // Max 6+ times per second
#define VOLTAGE_MAX_TIME_BETWEEN_CALLS 1000    // call at least once per second...
#define VOLTAGE_TIME_TO_ERROR          3000    // Error out if no valid item is returned in 3 seconds...



// Current positions in AX coordinates
int16_t      g_cur_servo_pos[NUMSERVOS];
int16_t      g_goal_servo_pos[NUMSERVOS];

#ifdef DBGSerial
//#define DEBUG
// Only allow debug stuff to be turned on if we have a debug serial port to output to...
#define DEBUG_SERVOS
#endif

#ifdef DEBUG_SERVOS
#define ServosEnabled   (g_fEnableServos)
#else
#define ServosEnabled  (true)      // always true compiler should remove if...
#endif

//=============================================================================
// Global - Local to this file only...
//=============================================================================
static const char *lss_status_text[] PROGMEM  = {
	"Unknown",	"Limp",	"FreeMoving",	"Accelerating",	"Travelling",	"Decelerating",
	"Holding",	"OutsideLimits",	"Stuck",  "Blocked", 
	"SafeMode", "Last" };

static const byte cPinTable[] = {
  cRRCoxaPin,  cRMCoxaPin,  cRFCoxaPin,  cLRCoxaPin,  cLMCoxaPin,  cLFCoxaPin,
  cRRFemurPin, cRMFemurPin, cRFFemurPin, cLRFemurPin, cLMFemurPin, cLFFemurPin,
  cRRTibiaPin, cRMTibiaPin, cRFTibiaPin, cLRTibiaPin, cLMTibiaPin, cLFTibiaPin
#ifdef c4DOF
  , cRRTarsPin, cRMTarsPin, cRFTarsPin, cLRTarsPin, cLMTarsPin, cLFTarsPin
#endif
#ifdef cTurretRotPin
  , cTurretRotPin, cTurretTiltPin
#endif  
};

#define FIRSTCOXAPIN     0
#define FIRSTFEMURPIN    (CNT_LEGS)
#define FIRSTTIBIAPIN    (CNT_LEGS*2)
#ifdef c4DOF
#define FIRSTTARSPIN     (CNT_LEGS*3)
#define FIRSTTURRETPIN   (CNT_LEGS*4)
#else
#define FIRSTTURRETPIN   (CNT_LEGS*3)
#endif
// Not sure yet if I will use the controller class or not, but...
LSS myLSS = LSS(0);
boolean g_fServosFree;    // Are the servos in a free state?


//============================================================================================
// Lets try rolling our own GPSequence code here...
#define GPSEQ_EEPROM_START 0x40       // Reserve the first 64 bytes of EEPROM for other stuff...
#define GPSEQ_EEPROM_START_DATA  0x50 // Reserved room for up to 8 in header...
#define GPSEQ_EEPROM_SIZE 0x800       // I think we have 2K
#define GPSEQ_EEPROM_MAX_SEQ 5        // For now this is probably the the max we can probably hold...


// Not sure if pragma needed or not...
//#pragma pack(1)
typedef struct {
	byte  bSeqNum;       // the sequence number, used to verify
	byte  bCntServos;    // count of servos
	byte  bCntSteps;     // How many steps there are
	byte  bCntPoses;     // How many poses
}
EEPromPoseHeader;

typedef struct {
	byte bPoseNum;        // which pose to use
	word wTime;        // Time to do pose
}
EEPROMPoseSeq;      // This is a sequence entry

// Pose is just an array of words...


// A sequence is stored as:
//<header> <sequences><poses>



// Some forward references
extern void MakeSureServosAreOn(void);
extern void DoPyPose(byte* psz);
extern void EEPROMReadData(word wStart, uint8_t* pv, byte cnt);
extern void EEPROMWriteData(word wStart, uint8_t* pv, byte cnt);
extern void TCServoPositions();
extern void FindServoOffsets();

extern void TCTrackServos();
extern void SetRegOnAllServos(uint8_t bReg, uint8_t bVal);

//====================================
//set MJS RF config Gait Test Values
// and Mucked up by KJE ;)
//====================================
typedef struct {
  uint8_t         id;
  LSS_ConfigGyre  gyre;
  int16_t         offset;
  int16_t         max_speed;
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
  {"Left Front",  {cLFCoxaPin, LSS_GyreCounterClockwise, 0, 600}, {cLFFemurPin, LSS_GyreClockwise, -104, 600}, {cLFTibiaPin, LSS_GyreClockwise, -137, 600}},
  {"Left Middle", {cLMCoxaPin, LSS_GyreCounterClockwise, 0, 600}, {cLMFemurPin, LSS_GyreClockwise, -104, 600}, {cLMTibiaPin, LSS_GyreClockwise, -137, 600}},
  {"Left Rear",   {cLRCoxaPin, LSS_GyreCounterClockwise, 0, 600}, {cLRFemurPin, LSS_GyreClockwise, -104, 600}, {cLRTibiaPin, LSS_GyreClockwise, -137, 600}},

  {"Right Front",  {cRFCoxaPin, LSS_GyreCounterClockwise, 0, 600}, {cRFFemurPin, LSS_GyreCounterClockwise, -104, 600}, {cRFTibiaPin, LSS_GyreCounterClockwise, -137, 600}},
  {"Right Middle", {cRMCoxaPin, LSS_GyreCounterClockwise, 0, 600}, {cRMFemurPin, LSS_GyreCounterClockwise, -104, 600}, {cRMTibiaPin, LSS_GyreCounterClockwise, -137, 600}},
  {"Right Rear",   {cRRCoxaPin, LSS_GyreCounterClockwise, 0, 600}, {cRRFemurPin, LSS_GyreCounterClockwise, -104, 600}, {cRRTibiaPin, LSS_GyreCounterClockwise, -137, 600}}
};
#define COUNT_LEGS (sizeof(legs)/sizeof(legs[0]))

void ServoDriver::setGaitConfig()
{
  for (uint8_t leg = 0; leg < COUNT_LEGS; leg++) {
    legs[leg].leg_found = true;
    myLSS.setServoID(legs[leg].coxa.id);
    if (myLSS.getStatus() == LSS_StatusUnknown) legs[leg].leg_found = false;
    myLSS.setMaxSpeed(legs[leg].coxa.max_speed, LSS_SetSession);
    myLSS.setGyre(legs[leg].coxa.gyre, LSS_SetSession);
    myLSS.setOriginOffset(legs[leg].coxa.offset, LSS_SetSession);

    myLSS.setServoID(legs[leg].femur.id);
    if (myLSS.getStatus() == LSS_StatusUnknown) legs[leg].leg_found = false;
    myLSS.setMaxSpeed(legs[leg].femur.max_speed, LSS_SetSession);
    myLSS.setGyre(legs[leg].femur.gyre, LSS_SetSession);
    myLSS.setOriginOffset(legs[leg].femur.offset, LSS_SetSession);

    myLSS.setServoID(legs[leg].tibia.id);
    if (myLSS.getStatus() == LSS_StatusUnknown) legs[leg].leg_found = false;
    myLSS.setMaxSpeed(legs[leg].tibia.max_speed, LSS_SetSession);
    myLSS.setGyre(legs[leg].tibia.gyre, LSS_SetSession);
    myLSS.setOriginOffset(legs[leg].tibia.offset, LSS_SetSession);

    if (legs[leg].leg_found) Serial.printf("Servos for Leg %s **found**\n", legs[leg].leg_name);
    else Serial.printf("Servos for Leg %s **NOT found**\n", legs[leg].leg_name);
  }
}

//--------------------------------------------------------------------
//Init
//--------------------------------------------------------------------
void ServoDriver::Init(void) {
	// First lets get the actual servo positions for all of our servos...
	//  pinMode(0, OUTPUT);
	LSS::initBus(LSS_SERIAL_PORT, LSS_BAUD);

	g_fServosFree = true;
#ifdef DBGSerial
	int32_t pos;
	int     count_missing = 0;

	for (int i = 0; i < NUMSERVOS; i++) {
		// Set the id
		int servo_id = pgm_read_byte(&cPinTable[i]);
		myLSS.setServoID(servo_id);

		// Now try to get it's position
		DBGSerial.print("Servo(");
		DBGSerial.print(i, DEC);
		DBGSerial.print("): ");
		DBGSerial.print(servo_id, DEC);

		pos = myLSS.getPosition();
		if (myLSS.getLastCommStatus() == LSS_CommStatus_ReadSuccess) {
			DBGSerial.printf(" %d\n", pos);
		}
		else {
			DBGSerial.println(" not found");
			++count_missing;
		}
		delay(25);
	}

	// Now see if we should try to recover from a potential servo that renumbered itself back to 1.
	if (count_missing) {
		DBGSerial.print("ERROR: Servo driver init: ");
		DBGSerial.print(count_missing, DEC);
		DBGSerial.println(" servos missing");
	}
#endif


#ifdef cVoltagePin
	for (byte i = 0; i < 8; i++)
		GetBatteryVoltage();  // init the voltage pin
#endif


}


//--------------------------------------------------------------------
//GetBatteryVoltage - Maybe should try to minimize when this is called
// as it uses the serial port... Maybe only when we are not interpolating
// or if maybe some minimum time has elapsed...
//--------------------------------------------------------------------

#ifdef cVoltagePin
word  g_awVoltages[8] = {
  0, 0, 0, 0, 0, 0, 0, 0
};
word  g_wVoltageSum = 0;
byte  g_iVoltages = 0;

word ServoDriver::GetBatteryVoltage(void) {
	g_iVoltages = (g_iVoltages + 1) & 0x7; // setup index to our array...
	g_wVoltageSum -= g_awVoltages[g_iVoltages];
	g_awVoltages[g_iVoltages] = analogRead(cVoltagePin);
	g_wVoltageSum += g_awVoltages[g_iVoltages];

#ifdef CVREF
	return ((long)((long)g_wVoltageSum * CVREF * (CVADR1 + CVADR2)) / (long)(8192 * (long)CVADR2));
#else
	return ((long)((long)g_wVoltageSum * 125 * (CVADR1 + CVADR2)) / (long)(2048 * (long)CVADR2));
#endif
}

#else
word g_wLastVoltage = 0xffff;    // save the last voltage we retrieved...
byte g_bLegVoltage = 0;   // what leg did we last check?
unsigned long g_ulTimeLastBatteryVoltage;

word ServoDriver::GetBatteryVoltage(void) {
	// In this case, we have to ask a servo for it's current voltage level, which is a lot more overhead than simply doing
	// one AtoD operation.  So we will limit when we actually do this to maybe a few times per second.
	// Also if interpolating, the code will try to only call us when it thinks it won't interfer with timing of interpolation.
#ifdef LATER
	unsigned long ulDeltaTime = millis() - g_ulTimeLastBatteryVoltage;
	if (g_wLastVoltage != 0xffff) {
		if ((ulDeltaTime < VOLTAGE_MIN_TIME_BETWEEN_CALLS)
			|| (bioloid.interpolating && (ulDeltaTime < VOLTAGE_MAX_TIME_BETWEEN_CALLS)))
			return g_wLastVoltage;
	}

	// Lets cycle through the Tibia servos asking for voltages as they may be the ones doing the most work...
	register word wVoltage = ax12GetRegister(pgm_read_byte(&cPinTable[FIRSTTIBIAPIN + g_bLegVoltage]), AX_PRESENT_VOLTAGE, 1);
	if (++g_bLegVoltage >= CNT_LEGS)
		g_bLegVoltage = 0;
	if (wVoltage != 0xffff) {
		g_ulTimeLastBatteryVoltage = millis();
		g_wLastVoltage = wVoltage * 10;
		return g_wLastVoltage;
	}

	// Allow it to error our a few times, but if the time until we get a valid response exceeds some time limit then error out.
	if (ulDeltaTime < VOLTAGE_TIME_TO_ERROR)
		return g_wLastVoltage;
#endif
	return 0;

}
#endif


//------------------------------------------------------------------------------------------
//[BeginServoUpdate] Does whatever preperation that is needed to starrt a move of our servos
//------------------------------------------------------------------------------------------
void ServoDriver::BeginServoUpdate(void)    // Start the update
{
	MakeSureServosAreOn();
#if 0
	if (ServosEnabled) {
		// If we are trying our own Servo control need to save away the new positions...
		for (byte i = 0; i < NUMSERVOS; i++) {
			g_cur_servo_pos[i] = g_goal_servo_pos[i];
		}
	}
#endif

}

//------------------------------------------------------------------------------------------
//[OutputServoInfoForLeg] Do the output to the SSC-32 for the servos associated with
//         the Leg number passed in.
//------------------------------------------------------------------------------------------
#ifdef c4DOF
void ServoDriver::OutputServoInfoForLeg(byte LegIndex, short sCoxaAngle1, short sFemurAngle1, short sTibiaAngle1, short sTarsAngle1)
#else
void ServoDriver::OutputServoInfoForLeg(byte LegIndex, short sCoxaAngle1, short sFemurAngle1, short sTibiaAngle1)
#endif
{
	// Save away the new positions...
	g_goal_servo_pos[FIRSTCOXAPIN + LegIndex] = sCoxaAngle1;  // What order should we store these values?
	g_goal_servo_pos[FIRSTFEMURPIN + LegIndex] = sFemurAngle1;
	g_goal_servo_pos[FIRSTTIBIAPIN + LegIndex] = sTibiaAngle1;
#ifdef c4DOF
	g_awGoalAXTarsPos[FIRSTTARSPIN + LegIndex] = sTarsAngle1;
#endif
#ifdef DEBUG_SERVOS
	if (g_fDebugOutput) {
		DBGSerial.print(LegIndex, DEC);
		DBGSerial.print("(");
		DBGSerial.print(sCoxaAngle1, DEC);
		DBGSerial.print("),(");
		DBGSerial.print(sFemurAngle1, DEC);
		DBGSerial.print("),(");
		DBGSerial.print("(");
		DBGSerial.print(sTibiaAngle1, DEC);
		DBGSerial.print(") :");
	}
#endif
	g_InputController.AllowControllerInterrupts(true);    // Ok for hserial again...
}


//------------------------------------------------------------------------------------------
//[OutputServoInfoForTurret] Set up the outputse servos associated with an optional turret
//         the Leg number passed in.  FIRSTTURRETPIN
//------------------------------------------------------------------------------------------
#ifdef cTurretRotPin
void ServoDriver::OutputServoInfoForTurret(short sRotateAngle1, short sTiltAngle1)
{
#ifdef LATER
	word    wRotateSDV;
	word    wTiltSDV;        //

	// The Main code now takes care of the inversion before calling.
	wRotateSDV = (((long)(sRotateAngle1)) * cPwmMult) / cPwmDiv + cPFConst;
	wTiltSDV = (((long)((long)(sTiltAngle1)) * cPwmMult) / cPwmDiv + cPFConst);

	if (ServosEnabled) {
		if (g_fAXSpeedControl) {
#ifdef USE_AX12_SPEED_CONTROL
			// Save away the new positions...
			g_goal_servo_pos[FIRSTTURRETPIN] = wRotateSDV;    // What order should we store these values?
			g_goal_servo_pos[FIRSTTURRETPIN + 1] = wTiltSDV;
#endif
		}
		else {
			bioloid.setNextPose(pgm_read_byte(&cPinTable[FIRSTTURRETPIN]), wRotateSDV);
			bioloid.setNextPose(pgm_read_byte(&cPinTable[FIRSTTURRETPIN + 1]), wTiltSDV);
		}
	}
#ifdef DEBUG_SERVOS
	if (g_fDebugOutput) {
		DBGSerial.print("(");
		DBGSerial.print(sRotateAngle1, DEC);
		DBGSerial.print("=");
		DBGSerial.print(wRotateSDV, DEC);
		DBGSerial.print("),(");
		DBGSerial.print(sTiltAngle1, DEC);
		DBGSerial.print("=");
		DBGSerial.print(wTiltSDV, DEC);
		DBGSerial.print(") :");
	}
#endif
#endif
}
#endif
//--------------------------------------------------------------------
//[CommitServoDriver Updates the positions of the servos - This outputs
//         as much of the command as we can without committing it.  This
//         allows us to once the previous update was completed to quickly
//        get the next command to start
//--------------------------------------------------------------------
void ServoDriver::CommitServoDriver(word wMoveTime)
{
	g_InputController.AllowControllerInterrupts(false);    // If on xbee on hserial tell hserial to not processess...
	if (ServosEnabled) {
		for (int i = 0; i < NUMSERVOS; i++) {
			if (g_cur_servo_pos[i] != g_goal_servo_pos[i]) {
				g_cur_servo_pos[i] = g_goal_servo_pos[i];
				// Set the id
				int servo_id = pgm_read_byte(&cPinTable[i]);
				myLSS.setServoID(servo_id);
				myLSS.moveT(g_goal_servo_pos[i], wMoveTime);
			}
		}
	}
	else {
		// Rear middle front
		//DBGSerial.println("Servo positions shown by leg joints\n(Rear)");
		//DBGSerial.println("    T     F     C |     C     F     T");
		for (int legs = 0; legs < 3; legs++) {
			DBGSerial.printf("%5d %5d %5d | %5d %5d %5d || ",
				g_goal_servo_pos[FIRSTTIBIAPIN + legs], g_goal_servo_pos[FIRSTFEMURPIN + legs], g_goal_servo_pos[FIRSTCOXAPIN + legs],
				g_goal_servo_pos[FIRSTCOXAPIN + legs + 3], g_goal_servo_pos[FIRSTFEMURPIN + legs + 3], g_goal_servo_pos[FIRSTTIBIAPIN + legs + 3]);
		}
		Serial.printf("%u\n", wMoveTime);
	}
#ifdef DEBUG_SERVOS
	if (g_fDebugOutput)
		DBGSerial.println(wMoveTime, DEC);
#endif
	g_InputController.AllowControllerInterrupts(true);
}

//--------------------------------------------------------------------
//[FREE SERVOS] Frees all the servos
//--------------------------------------------------------------------
void ServoDriver::FreeServos(void)
{
	if (!g_fServosFree) {
		g_InputController.AllowControllerInterrupts(false);    // If on xbee on hserial tell hserial to not processess...

		// See if we can do by broadcast
		LSS::genericWrite(LSS_BroadcastID, LSS_ActionLimp); // Tell all of the servos to go limp
		g_InputController.AllowControllerInterrupts(true);
		g_fServosFree = true;
	}
}

//--------------------------------------------------------------------
//Function that gets called from the main loop if the robot is not logically
//     on.  Gives us a chance to play some...
//--------------------------------------------------------------------
static uint8_t g_iIdleServoNum = (uint8_t)-1;
static uint8_t g_iIdleLedState = 1;  // what state to we wish to set...
void ServoDriver::IdleTime(void)
{
	// Each time we call this set servos LED on or off...
	// Lets just have one on at a time.
	if (g_iIdleServoNum < NUMSERVOS) {
		myLSS.setServoID(cPinTable[g_iIdleServoNum]);
		myLSS.setColorLED(LSS_LED_Black);
	}

	g_iIdleServoNum++;
	if (g_iIdleServoNum >= NUMSERVOS) {
		g_iIdleServoNum = 0;
		g_iIdleLedState++;
		if (g_iIdleLedState > 7) g_iIdleLedState = 0;
	}
	myLSS.setServoID(cPinTable[g_iIdleServoNum]);
	myLSS.setColorLED((LSS_LED_Color)g_iIdleLedState);

}

//--------------------------------------------------------------------
//Function that gets called from the main loop if the robot is not logically
//     on.  Gives us a chance to play some...
//--------------------------------------------------------------------
void ServoDriver::showUserFeedback(int feedback_state) {
	switch (feedback_state) {
		case 0:
			// turn off all leds... 
			LSS::genericWrite(LSS_BroadcastID, LSS_ActionColorLED, LSS_LED_Black);
			break;
	}
}

//--------------------------------------------------------------------
//[MakeSureServosAreOn] Function that is called to handle when you are
//  transistioning from servos all off to being on.  May need to read
//  in the current pose...
//--------------------------------------------------------------------
void MakeSureServosAreOn(void)
{
	if (ServosEnabled) {
		if (!g_fServosFree)
			return;    // we are not free

		g_InputController.AllowControllerInterrupts(false);    // If on xbee on hserial tell hserial to not processess...

		LSS::genericWrite(LSS_BroadcastID, LSS_ActionHold); // Tell all of the servos to hold a position.
		boolean servos_reset = false;
		for (int i = 0; i < NUMSERVOS; i++) {
			g_cur_servo_pos[i] = 32768; // set to a value that is not valid to force next output
			// lets make sure that servos are not in an error state.   
			myLSS.setServoID(cPinTable[i]);
			LSS_Status servo_status = myLSS.getStatus();
			switch (servo_status) {
			case LSS_StatusUnknown:
			case LSS_StatusHolding:
				break;	// don't need to do anything
			case LSS_StatusLimp:
			case LSS_StatusFreeMoving:
			case LSS_StatusAccelerating:
			case LSS_StatusTravelling:
			case LSS_StatusDecelerating:
			case LSS_StatusOutsideLimits:
			case LSS_StatusStuck:
			case LSS_StatusBlocked:
			case LSS_StatusSafeMode:
			default:
				DBGSerial.printf("EnableServos: Servo %d reset due to status: %s(%d)\n", cPinTable[i], 
						(servo_status < (sizeof(lss_status_text)/sizeof(lss_status_text[0]))) ? lss_status_text[servo_status] : "?",
						servo_status); 
				myLSS.reset();
				servos_reset = true;
				break;
			}
		}

		if (servos_reset) {
			delay(3000);  // give servos some time to reset.
			// try again to hold servos. 
			LSS::genericWrite(LSS_BroadcastID, LSS_ActionHold); // Tell all of the servos to hold a position
		}

		g_InputController.AllowControllerInterrupts(true);
		g_fServosFree = false;
	}
}

//==============================================================================
// BackgroundProcess - Allows us to have some background processing for those
//    servo drivers that need us to do things like polling...
//==============================================================================
void  ServoDriver::BackgroundProcess(void)
{
#ifdef cTurnOffVol          // only do if we a turn off voltage is defined
#ifndef cVoltagePin         // and we are not doing AtoD type of conversion...
	if (iTimeToNextInterpolate > VOLTAGE_MIN_TIME_UNTIL_NEXT_INTERPOLATE)      // At least 4ms until next interpolation.  See how this works...
		GetBatteryVoltage();
#endif
#endif
}


#ifdef OPT_TERMINAL_MONITOR
//==============================================================================
// ShowTerminalCommandList: Allow the Terminal monitor to call the servo driver
//      to allow it to display any additional commands it may have.
//==============================================================================
void ServoDriver::ShowTerminalCommandList(void)
{
	DBGSerial.println(F("V - Voltage"));
	DBGSerial.println(F("M - Toggle Motors on or off"));
	DBGSerial.println(F("T - Test Servos"));
	DBGSerial.println(F("P - Servo Positions"));
	DBGSerial.println(F("S - Track Servos"));
#ifdef OPT_PYPOSE
	DBGSerial.println(F("P<DL PC> - Pypose"));
#endif
#ifdef OPT_FIND_SERVO_OFFSETS
	DBGSerial.println(F("O - Enter Servo offset mode"));
#endif
}

//==============================================================================
// ProcessTerminalCommand: The terminal monitor will call this to see if the
//     command the user entered was one added by the servo driver.
//==============================================================================
boolean ServoDriver::ProcessTerminalCommand(byte* psz, byte bLen)
{
	if ((bLen == 1) && ((*psz == 'm') || (*psz == 'M'))) {
		g_fEnableServos = !g_fEnableServos;
		if (g_fEnableServos) {

			DBGSerial.println(F("Motors are on"));
		}
		else {
			DBGSerial.println(F("Motors are off"));
			FreeServos();	// make sure we turn off servos. 
		}

		return true;
	}
	if ((bLen == 1) && ((*psz == 'v') || (*psz == 'V'))) {
		DBGSerial.print(F("Voltage: "));
		DBGSerial.println(GetBatteryVoltage(), DEC);
#ifdef cVoltagePin
		DBGSerial.print("Raw Analog: ");
		DBGSerial.println(analogRead(cVoltagePin));
#endif

		DBGSerial.print(F("From Servo 2: "));
		myLSS.setServoID(2);
		DBGSerial.println(myLSS.getVoltage(), DEC);
	}

	if ((bLen == 1) && ((*psz == 't') || (*psz == 'T'))) {
		// Test to see if any servos are responding
		for (int i = 1; i <= 32; i++) {
			int iPos;
			myLSS.setServoID(i);
			iPos = myLSS.getPosition();
			DBGSerial.print(i, DEC);
			DBGSerial.print(F("="));
			if (myLSS.getLastCommStatus() == LSS_CommStatus_ReadSuccess) {
				DBGSerial.println(iPos, DEC);
			}
			else {
				DBGSerial.println(F("** failed **"));
			}
			delay(25);
		}
	}
	if (((*psz == 'p') || (*psz == 'P'))) {
		TCServoPositions();
	}
	if ((*psz == 's') || (*psz == 'S')) {
		TCTrackServos();
	}


#ifdef OPT_FIND_SERVO_OFFSETS
	else if ((bLen == 1) && ((*psz == 'o') || (*psz == 'O'))) {
		FindServoOffsets();
	}
#endif
	return false;

}
//==============================================================================
// TCServoPositions -
//==============================================================================
void TCServoPositions() {
	int16_t servo_pos[NUMSERVOS];
	LSS_Status servo_status[NUMSERVOS];
	int i;

	for (i = 0; i < NUMSERVOS; i++) {
		myLSS.setServoID(cPinTable[i]);
		servo_pos[i] = myLSS.getPosition();
		LSS_LastCommStatus lss_status = myLSS.getLastCommStatus();
		if (lss_status != LSS_CommStatus_ReadSuccess) {
			servo_pos[i] = 0x7fff; // out of valid range
			DBGSerial.printf("%u fail: %x\n", cPinTable[i], (uint32_t)lss_status);
		}
		servo_status[i] = myLSS.getStatus();
	}

	// Not very clean
	// Rear middle front
	DBGSerial.println("Servo positions shown by leg joints\n(Rear)");
	DBGSerial.println("    T     F     C |     C     F     T");
	for (int legs = 0; legs < 3; legs++) {
		DBGSerial.printf("%5d(%u) %5d(%u) %5d(%u) | %5d(%u) %5d(%u) %5d(%u)\n",
			servo_pos[FIRSTTIBIAPIN + legs], 	 servo_status[FIRSTTIBIAPIN + legs], 
			servo_pos[FIRSTFEMURPIN + legs], 	 servo_status[FIRSTFEMURPIN + legs], 
			servo_pos[FIRSTCOXAPIN + legs], 	 servo_status[FIRSTCOXAPIN + legs],
			servo_pos[FIRSTCOXAPIN + legs + 3],  servo_status[FIRSTCOXAPIN + legs + 3], 
			servo_pos[FIRSTFEMURPIN + legs + 3], servo_status[FIRSTFEMURPIN + legs + 3], 
			servo_pos[FIRSTTIBIAPIN + legs + 3], servo_status[FIRSTTIBIAPIN + legs + 3]);
	}
}

//==============================================================================
// TCTrackServos - Lets set a mode to track servos.  Can use to help figure out
// proper initial positions and min/max values...
//==============================================================================
void TCTrackServos()
{
	// First read through all of the servos to get their position.
	int16_t auPos[NUMSERVOS];
	int16_t  uPos;
	int16_t servo_mins[NUMSERVOS];
	int16_t servo_maxs[NUMSERVOS];
	int i;
	boolean fChange;

	// Clear out any pending input characters
	while (DBGSerial.read() != -1)
		;
	DBGSerial.println("\nTrack servos - enter any key to exit");
	for (i = 0; i < NUMSERVOS; i++) {
		myLSS.setServoID(cPinTable[i]);
		servo_mins[i] = auPos[i] = servo_maxs[i] = myLSS.getPosition();
	}

	// Now loop until we get some input on the serial
	while (!DBGSerial.available()) {
		fChange = false;
		for (int i = 0; i < NUMSERVOS; i++) {
			myLSS.setServoID(cPinTable[i]);
			uPos = myLSS.getPosition();
			if (myLSS.getLastCommStatus() == LSS_CommStatus_ReadSuccess) {
				if (uPos > servo_maxs[i]) servo_maxs[i] = uPos;
				if (uPos < servo_mins[i]) servo_mins[i] = uPos;
				// Lets put in a littl delta or shows lots
				if (abs(auPos[i] - uPos) > 2) {
					auPos[i] = uPos;
					if (fChange)
						DBGSerial.print(", ");
					else
						fChange = true;
					DBGSerial.print(pgm_read_byte(&cPinTable[i]), DEC);
					DBGSerial.print(": ");
					DBGSerial.print(uPos, DEC);
				}
			}
		}
		if (fChange)
			DBGSerial.println();
		delay(25);
	}
	// Print out Mins and Max.
	//DBGSerial.println("    T     F     C |     C     F     T");
	static const char* apszLegs[] = {
	  "RR", "RM", "RF", "LR", "LM", "LF"
	};      // Leg Order

	for (int legs = 0; legs < 6; legs++) {
		DBGSerial.printf("#define c%sCoxaMin1\t%d\n", apszLegs[legs], servo_mins[FIRSTCOXAPIN + legs]);
		DBGSerial.printf("#define c%sCoxaMax1\t%d\n", apszLegs[legs], servo_maxs[FIRSTCOXAPIN + legs]);
		DBGSerial.printf("#define c%sFemurMin1\t%d\n", apszLegs[legs], servo_mins[FIRSTFEMURPIN + legs]);
		DBGSerial.printf("#define c%sFemurMax1\t%d\n", apszLegs[legs], servo_maxs[FIRSTFEMURPIN + legs]);
		DBGSerial.printf("#define c%sTibiaMin1\t%d\n", apszLegs[legs], servo_mins[FIRSTTIBIAPIN + legs]);
		DBGSerial.printf("#define c%sTibiaMax1\t%d\n", apszLegs[legs], servo_maxs[FIRSTTIBIAPIN + legs]);
	}
#if 0
	DBGSerial.println("\nExit Track servos - Min/Max values");
	for (int i = 0; i < NUMSERVOS; i++) {
		if (servo_maxs[i] != servo_mins[i]) {
			DBGSerial.print(cPinTable[i], DEC);
			DBGSerial.print(" Min: ");
			DBGSerial.print(servo_mins[i], DEC);
			DBGSerial.print(" Max: ");
			DBGSerial.println(servo_maxs[i], DEC);
		}
	}
#endif
}


#endif

#ifdef OPT_FIND_SERVO_OFFSETS
//==============================================================================
//  FindServoOffsets - Find the zero points for each of our servos...
//==============================================================================
#ifndef NUMSERVOSPERLEG
#define NUMSERVOSPERLEG 3
#endif

void FindServoOffsets()
{
	// not clean but...
	signed short asOffsets[NUMSERVOSPERLEG * CNT_LEGS];      // we have 18 servos to find/set offsets for...

	static const char* apszLegs[] = {
	  "RR", "RM", "RF", "LR", "LM", "LF"
	};      // Leg Order
	static const char* apszLJoints[] = {
	  " Coxa", " Femur", " Tibia", " tArs"
	};   // which joint on the leg...


	int data;
	short sSN;       // which servo number
	boolean fNew = true;  // is this a new servo to work with?
	boolean fExit = false;  // when to exit

	for (uint8_t i = 0; i < NUMSERVOS; i++) {
		asOffsets[i] = 0;
	}

	if (CheckVoltage()) {
		// Voltage is low...
		Serial.println("Low Voltage: fix or hit $ to abort");
		while (CheckVoltage()) {
			if (Serial.read() == '$')  return;
		}
	}
	// Now lets enable all servos and set them to zero point
	MakeSureServosAreOn();
	LSS::genericWrite(LSS_BroadcastID, LSS_ActionMove, 0,
		LSS_ActionParameterTime, 500);  // move in half second

// OK lets move all of the servos to their zero point.
	Serial.println("Find Servo Zeros.\n$-Exit, +- changes, *-change servo");
	Serial.println("    0-n Chooses a leg, C-Coxa, F-Femur, T-Tibia");
	//#define NUMSERVOS (NUMSERVOSPERLEG*CNT_LEGS)

	// Lets show some information about each of the servos. 
	for (sSN = 0; sSN < NUMSERVOS; sSN++) {
		asOffsets[sSN] = 0;
		myLSS.setServoID(cPinTable[sSN]);
		Serial.print("Servo: ");
		Serial.print(apszLegs[sSN % CNT_LEGS]);
		Serial.print(apszLJoints[sSN / CNT_LEGS]);
		Serial.print("(");
		Serial.print(cPinTable[sSN], DEC);
		Serial.print(") Pos:");
		Serial.print(myLSS.getPosition(), DEC);
		Serial.print(" Origin Offset: ");
		Serial.print(myLSS.getOriginOffset(), DEC);
		Serial.print(" Angular Range: ");
		Serial.println(myLSS.getAngularRange(), DEC);

	}


	sSN = 0;
	bool data_received = false;
	while (!fExit) {
		if (fNew) {
			uint8_t servo_id = cPinTable[sSN];
			Serial.print("Servo: ");
			Serial.print(apszLegs[sSN % CNT_LEGS]);
			Serial.print(apszLJoints[sSN / CNT_LEGS]);
			Serial.print("(");
			Serial.print(servo_id, DEC);
			Serial.println(")");

			myLSS.setServoID(servo_id);
			myLSS.moveT(asOffsets[sSN], 250);
			delay(250);
			myLSS.moveT(asOffsets[sSN] + 100, 250);
			delay(250);
			myLSS.moveT(asOffsets[sSN] - 100, 250);
			delay(250);
			myLSS.moveT(asOffsets[sSN], 250);
			delay(250);
			fNew = false;
		}

		//get user entered data
		data = Serial.read();
		//if data received
		if (data >= '\r') {
			if (data == '\r') {
				if (!data_received) {
					// direct enter of which servo to change
					fNew = true;
					sSN++;
					if (sSN == CNT_LEGS * NUMSERVOSPERLEG)
						sSN = 0;
				}
				data_received = false;
			}
			else {
				data_received = true;
				if (data == '$')
					fExit = true; // not sure how the keypad will map so give NL, CR, LF... all implies exit

				else if ((data == '+') || (data == '-')) {
					if (data == '+')
						asOffsets[sSN] += 5;    // increment by 5us
					else
						asOffsets[sSN] -= 5;    // increment by 5us

					Serial.print("    ");
					Serial.println(asOffsets[sSN], DEC);

					myLSS.moveT(asOffsets[sSN], 100);
				}
				else if ((data >= '0') && (data <= '5')) {
					// direct enter of which servo to change
					fNew = true;
					//sSN = (sSN % NUMSERVOSPERLEG) + (data - '0') * NUMSERVOSPERLEG;
					sSN = (sSN / CNT_LEGS) * CNT_LEGS + (data - '0');
				}
				else if ((data == 'c') || (data == 'C')) {
					fNew = true;
					//sSN = (sSN / NUMSERVOSPERLEG) * NUMSERVOSPERLEG + 0;
					sSN = sSN % CNT_LEGS;
				}
				else if ((data == 'f') || (data == 'F')) {
					fNew = true;
					//sSN = (sSN / NUMSERVOSPERLEG) * NUMSERVOSPERLEG + 1;
					sSN = (sSN / NUMSERVOSPERLEG) * NUMSERVOSPERLEG + 1;
					sSN = CNT_LEGS + (sSN % CNT_LEGS);
				}
				else if ((data == 't') || (data == 'T')) {
					// direct enter of which servo to change
					fNew = true;
					sSN = (sSN / NUMSERVOSPERLEG) * NUMSERVOSPERLEG + 2;
					sSN = 2 * CNT_LEGS + (sSN % CNT_LEGS);
				}
				else if (data == '*') {
					// direct enter of which servo to change
					fNew = true;
					sSN++;
					if (sSN == CNT_LEGS * NUMSERVOSPERLEG)
						sSN = 0;
				}
			}
		}
	}
	Serial.print("Find Servo exit ");
	for (sSN = 0; sSN < NUMSERVOS; sSN++) {
		Serial.print("Servo: ");
		Serial.print(apszLegs[sSN / NUMSERVOSPERLEG]);
		Serial.print(apszLJoints[sSN % NUMSERVOSPERLEG]);
		Serial.println();
	}

#if 0
	Serial.print("\nSave Changes? Y/N: ");

	//get user entered data
	while (((data = Serial.read()) == -1) || ((data >= 10) && (data <= 15)))
		;

	if ((data == 'Y') || (data == 'y')) {
		// Ok they asked for the data to be saved.  We will store the data with a
		// number of servos (byte)at the start, followed by a byte for a checksum...followed by our offsets array...
		// Currently we store these values starting at EEPROM address 0. May later change...
		//

		for (sSN = 0; sSN < CNT_LEGS * NUMSERVOSPERLEG; sSN++) {
			SSCSerial.print("R");
			SSCSerial.print(32 + abSSCServoNum[sSN], DEC);
			SSCSerial.print("=");
			SSCSerial.println(asOffsetsRead[sSN] + asOffsets[sSN], DEC);
			delay(10);
		}

		// Then I need to have the SSC-32 reboot in order to use the new values.
		delay(10);    // give it some time to write stuff out.
		SSCSerial.println("GOBOOT");
		delay(5);        // Give it a little time
		SSCSerial.println("g0000");    // tell it that we are done in the boot section so go run the normall SSC stuff...
		delay(500);                // Give it some time to boot up...
	}
	else {
		void LoadServosConfig();
	}
#endif
	g_ServoDriver.FreeServos();

}
#endif  // OPT_FIND_SERVO_OFFSETS

//==============================================================================
// EEPromReadData - Quick and dirty function to read multiple bytes in from
//  eeprom...
//==============================================================================
void EEPROMReadData(word wStart, uint8_t* pv, byte cnt) {
	while (cnt--) {
		*pv++ = EEPROM.read(wStart++);
	}
}

//==============================================================================
// DoPyPose - This is based off of the Pypose sketch...
// ArbotiX Test Program for use with PyPose 0013
// Copyright (c) 2008-2010 Michael E. Ferguson.  All right reserved.
//
// The code was put onto a memory diet and extended by me...
//==============================================================================
#ifdef OPT_PYPOSE

// Some defines for different modes and commands
#define ARB_SIZE_POSE   7  // also initializes
#define ARB_LOAD_POSE   8
#define ARB_LOAD_SEQ    9
#define ARB_PLAY_SEQ    10
#define ARB_LOOP_SEQ    11
#define ARB_SAVE_EEPROM_SEQ 12
#define ARB_TEST        25

// Global to this function...
byte   g_bPoseSize = NUMSERVOS;

short g_poses[540];              // enough for 30 steps...
typedef struct {
	byte  pose;    // index of pose to transition to
	word time;              // time for transition
}
sp_trans_t;

sp_trans_t g_sequence[30];   // sequence
byte g_bParams[90];  // parameters

// Some forward referencs
extern boolean PyPoseSaveToEEPROM(byte);



void DoPyPose(byte* psz)
{
	int mode = 0;              // where we are in the frame

	byte id = 0;      // id of this frame
	byte length = 0;  // length of this frame
	byte ins = 0;     // instruction of this frame

	byte index = 0;   // index in param buffer
	word wPoseIndex;

	int checksum = 0;              // checksum


	//  pose and sequence storage
	// Put on diet - convert int to short plus convert poses from 2 dimensions to 1 dimension...
	//
	byte seqPos;                // step in current sequence

	// See if the user gave us a string to process.  If so it should be the XBEE DL to talk to.
	// BUGBUG:: if using our XBEE stuff could default to debug terminal...
#ifdef USEXBEE

#ifdef DBGSerial
	word wMY;
	wMY = GetXBeeHVal('M', 'Y');

	DBGSerial.print(F("My: "));
	DBGSerial.println(wMY, HEX);
	wMY = GetXBeeHVal('D', 'L');  // I know reused variable...
	DBGSerial.print(F("PC DL: "));
	DBGSerial.println(wMY, HEX);
	DBGSerial.println(F("Exit Terminal and Start Pypose"));
	DBGSerial.println(F("$TEXTM$"));  // add some support in VB side to convert to text mode...
#endif
  // Now lets reset our xbee...
	APISendXBeeGetCmd('F', 'R'); // Send a Forced reset...
	delay(2000);
	while (Serial.read() != -1)
		;

	if (psz) {
		word wDL;
		for (wDL = 0; *psz; psz++) {
			if ((*psz >= '0') && (*psz <= '9'))
				wDL = (wDL << 4) + *psz - '0';
			if ((*psz >= 'a') && (*psz <= 'f'))
				wDL = (wDL << 4) + *psz - 'a' + 10;
			if ((*psz >= 'A') && (*psz <= 'F'))
				wDL = (wDL << 4) + *psz - 'A' + 10;
		}
		if (wDL) {

			Serial.print("+++");
			delay(1000);
			while ((checksum = Serial.read()) != -1) {
				//Serial.write((byte)checksum);
			}
			// Now lets set the DL and exit command mode...
			Serial.print("ATDL ");
			Serial.print(wDL, HEX);
			Serial.println(",ATCN");
		}
	}
	delay(10);
	while ((checksum = Serial.read()) != -1) {
		//Serial.write((byte)checksum);
	}

#endif

	// process messages
	for (;;) {
		while (Serial.available() > 0) {
			// We need to 0xFF at start of packet
			if (mode == 0) {       // start of new packet
				if (Serial.read() == 0xff) {
					mode = 2;
					//          digitalWrite(0,HIGH-digitalRead(0));
				}
				//}else if(mode == 1){   // another start byte
				//    if(Serial.read() == 0xff)
				//        mode = 2;
				//    else
				//        mode = 0;
			}
			else if (mode == 2) { // next byte is index of servo
				id = Serial.read();
				if (id != 0xff)
					mode = 3;
			}
			else if (mode == 3) { // next byte is length
				length = Serial.read();
				checksum = id + length;
				mode = 4;
			}
			else if (mode == 4) { // next byte is instruction
				ins = Serial.read();
				checksum += ins;
				index = 0;
				mode = 5;
			}
			else if (mode == 5) { // read data in
				g_bParams[index] = Serial.read();
				checksum += (int)g_bParams[index];
				index++;
				if (index + 1 == length) { // we've read params & checksum
					mode = 0;
					if ((checksum % 256) != 255) {
						// return a packet: FF FF id Len Err params=None check
						Serial.write((byte)0xff);
						Serial.write((byte)0xff);
						Serial.write((byte)id);
						Serial.write((byte)2);
						Serial.write((byte)64);
						Serial.write((byte)(255 - ((66 + id) % 256)));
					}
					else {
						if (id == 253) {
							// return a packet: FF FF id Len Err params=None check
							Serial.write((byte)0xff);
							Serial.write((byte)0xff);
							Serial.write((byte)id);
							Serial.write((byte)2);
							Serial.write((byte)0);
							Serial.write((byte)(255 - ((2 + id) % 256)));
							// special ArbotiX instructions
							// Pose Size = 7, followed by single param: size of pose
							// Load Pose = 8, followed by index, then pose positions (# of param = 2*pose_size)
							// Load Seq = 9, followed by index/times (# of parameters = 3*seq_size)
							// Play Seq = A, no params
							if (ins == ARB_SIZE_POSE) {
								g_bPoseSize = bioloid.poseSize = g_bParams[0];
								bioloid.readPose();
								//Serial.println(bioloid.poseSize);
							}
							else if (ins == ARB_LOAD_POSE) {
								int i;
								//Serial.print("New Pose:");
								wPoseIndex = g_bParams[0] * g_bPoseSize;
								for (i = 0; i < bioloid.poseSize; i++) {
									g_poses[wPoseIndex + i] = g_bParams[(2 * i) + 1] + (g_bParams[(2 * i) + 2] << 8);
									//Serial.print(g_poses[g_bParams[0]][i]);
									//Serial.print(",");
								}
								//Serial.println("");
							}
							else if (ins == ARB_LOAD_SEQ) {
								int i;
								for (i = 0; i < (length - 2) / 3; i++) {
									g_sequence[i].pose = g_bParams[(i * 3)];
									g_sequence[i].time = g_bParams[(i * 3) + 1] + (g_bParams[(i * 3) + 2] << 8);
									//Serial.print("New Transition:");
									//Serial.print((int)g_sequence[i].pose);
									//Serial.print(" in ");
									//Serial.println(g_sequence[i].time);
								}
							}
							else if (ins == ARB_PLAY_SEQ) {
								seqPos = 0;
								while (g_sequence[seqPos].pose != 0xff) {
									int i;
									int p = g_sequence[seqPos].pose;
									wPoseIndex = p * g_bPoseSize;
									// are we HALT?
									if (Serial.read() == 'H') return;
									// load pose
									for (i = 0; i < bioloid.poseSize; i++) {
										bioloid.setNextPose(i + 1, g_poses[wPoseIndex + i]);
									}
									// interpolate
									bioloid.interpolateSetup(g_sequence[seqPos].time);
									while (bioloid.interpolating)
										bioloid.interpolateStep();
									// next transition
									seqPos++;
								}
							}
							else if (ins == ARB_LOOP_SEQ) {
								while (1) {
									seqPos = 0;
									while (g_sequence[seqPos].pose != 0xff) {
										int i;
										int p = g_sequence[seqPos].pose;
										wPoseIndex = p * g_bPoseSize;
										// are we HALT?
										if (Serial.read() == 'H') return;
										// load pose
										for (i = 0; i < bioloid.poseSize; i++) {
											bioloid.setNextPose(i + 1, g_poses[wPoseIndex + i]);
										}
										// interpolate
										bioloid.interpolateSetup(g_sequence[seqPos].time);
										while (bioloid.interpolating)
											bioloid.interpolateStep();
										// next transition
										seqPos++;
									}
								}
							}
							else if (ins == ARB_SAVE_EEPROM_SEQ) {
								// g_bParams[0] = which location to save to...
								PyPoseSaveToEEPROM(g_bParams[0]);
							}
							else if (ins == ARB_TEST) {
								int i;
								// Test Digital I/O
								for (i = 0; i < 8; i++) {
									// test digital
									pinMode(i, OUTPUT);
									digitalWrite(i, HIGH);
									// test analog
									pinMode(31 - i, OUTPUT);
									digitalWrite(31 - i, HIGH);

									delay(500);
									digitalWrite(i, LOW);
									digitalWrite(31 - i, LOW);
								}
								// Test Ax-12
								for (i = 452; i < 552; i += 20) {
									SetPosition(1, i);
									delay(200);
								}
								delay(1500);
								// Test Analog I/O
								for (i = 0; i < 8; i++) {
									// test digital
									pinMode(i, OUTPUT);
									digitalWrite(i, HIGH);
									// test analog
									pinMode(31 - i, OUTPUT);
									digitalWrite(31 - i, HIGH);

									delay(500);
									digitalWrite(i, LOW);
									digitalWrite(31 - i, LOW);
								}
							}
						}
						else {
							int i;
							// pass thru
							if (ins == AX_READ_DATA) {
								ax12GetRegister(id, g_bParams[0], g_bParams[1]);
								// return a packet: FF FF id Len Err params check
								if (ax_rx_buffer[3] > 0) {
									for (i = 0; i < ax_rx_buffer[3] + 4; i++)
										Serial.write(ax_rx_buffer[i]);
								}
								ax_rx_buffer[3] = 0;
							}
							else if (ins == AX_WRITE_DATA) {
								if (length == 4) {
									ax12SetRegister(id, g_bParams[0], g_bParams[1]);
								}
								else {
									int x = g_bParams[1] + (g_bParams[2] << 8);
									ax12SetRegister2(id, g_bParams[0], x);
								}
								// return a packet: FF FF id Len Err params check
								Serial.write((byte)0xff);
								Serial.write((byte)0xff);
								Serial.write((byte)id);
								Serial.write((byte)2);
								Serial.write((byte)0);
								Serial.write((byte)(255 - ((2 + id) % 256)));
							}
						}
					}
				}
			}
		}

		// update joints
		bioloid.interpolateStep();
	}
}



void EEPROMWriteData(word wStart, uint8_t* pv, byte cnt) {
	while (cnt--) {
		EEPROM.write(wStart++, *pv++);
	}
}

// BUGBUG:: keeping it simple to start.  Will clear any sequence number > than the one we are storing...
boolean PyPoseSaveToEEPROM(byte bSeqNum) {
	//first verify it is in range...
	byte iSeq;
	word w;
	word wHeaderStart;
	EEPromPoseHeader eepph;
	EEPROMPoseSeq eepps;
	boolean fValidHeaders;

	if (bSeqNum >= GPSEQ_EEPROM_MAX_SEQ)
		return false;

	// Lets first see where the data should start and do some validation...
	wHeaderStart = GPSEQ_EEPROM_START_DATA;    // here is where I expect the next one to be...
	fValidHeaders = true;
	for (iSeq = 0; iSeq < bSeqNum; iSeq++) {
		EEPROMReadData(GPSEQ_EEPROM_START + iSeq * sizeof(word), (uint8_t*)&w, sizeof(w));
		if (w != wHeaderStart) {
			fValidHeaders = false;
			break;
		}
		EEPROMReadData(w, (uint8_t*)&eepph, sizeof(eepph));
		if ((eepph.bSeqNum != iSeq) || (eepph.bCntServos != NUMSERVOS)) {
			fValidHeaders = false;
			break;
		}
		w = wHeaderStart + sizeof(eepph) + (eepph.bCntSteps * sizeof(EEPROMPoseSeq)) + (eepph.bCntPoses * sizeof(word) * NUMSERVOS);
		if (w >= GPSEQ_EEPROM_SIZE) {
			fValidHeaders = false;
			break;
		}
		wHeaderStart = w;  // save away the last generated one...
	}

	// Note: if previous data invalid will try to store data after last valid one...
	eepph.bSeqNum = iSeq;    // save the sequence number here.
	eepph.bCntServos = NUMSERVOS;  // say that it has NUMSERVOS steps.
	eepph.bCntPoses = 0;
	for (eepph.bCntSteps = 0; g_sequence[eepph.bCntSteps].pose != 0xff; eepph.bCntSteps++) {
		if (g_sequence[eepph.bCntSteps].pose > eepph.bCntPoses)
			eepph.bCntPoses = g_sequence[eepph.bCntSteps].pose;
	}
	eepph.bCntPoses++;  // Cnt not index...

	// Make sure it will fit!
	if ((wHeaderStart + sizeof(eepph) + (eepph.bCntSteps * sizeof(EEPROMPoseSeq)) + (eepph.bCntPoses * sizeof(word) * NUMSERVOS)) >= GPSEQ_EEPROM_SIZE)
		return false;

	// Now lets start writing out the data...
	EEPROMWriteData(GPSEQ_EEPROM_START + iSeq * sizeof(word), (uint8_t*)&wHeaderStart, sizeof(wHeaderStart));

	// Clear out any other headers...
	w = 0;
	while (++iSeq < GPSEQ_EEPROM_MAX_SEQ)
		EEPROMWriteData(GPSEQ_EEPROM_START + iSeq * sizeof(word), (uint8_t*)&w, sizeof(w));


	// Now write out the sequence header
	EEPROMWriteData(wHeaderStart, (uint8_t*)&eepph, sizeof(eepph));
	wHeaderStart += sizeof(eepph);  // next location to write to.

	// Now lets write out sequence data
	for (eepph.bCntSteps = 0; g_sequence[eepph.bCntSteps].pose != 0xff; eepph.bCntSteps++) {
		eepps.bPoseNum = g_sequence[eepph.bCntSteps].pose;        // which pose to use
		eepps.wTime = g_sequence[eepph.bCntSteps].time;          // Time to do pose
		EEPROMWriteData(wHeaderStart, (uint8_t*)&eepps, sizeof(eepps));
		wHeaderStart += sizeof(eepps);
	}

	// Last lets write out the Pose data...
	EEPROMWriteData(wHeaderStart, (uint8_t*)g_poses, eepph.bCntPoses * NUMSERVOS * sizeof(g_poses[0]));

	return true;

}



#endif  //DOPypose
