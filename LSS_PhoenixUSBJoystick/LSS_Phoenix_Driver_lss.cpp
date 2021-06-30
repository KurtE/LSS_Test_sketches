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

// Some options for how we do interpolation
#define OUTPUT_ONLY_CHANGED_SERVOS 0
#define DYNAMIC_FPS 1



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
	"SafeMode", "Last"
};

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
// Some forward references
extern void TCServoPositions();

extern void TCTrackServos();
extern void ClearServoOffsets();

//====================================
//set MJS RF config Gait Test Values
// and Mucked up by KJE ;)
//====================================
static const LSS_ConfigGyre cGyreTable[] = {
	cRRCoxaGyre,  cRMCoxaGyre,  cRFCoxaGyre,  cLRCoxaGyre,  cLMCoxaGyre,  cLFCoxaGyre,
	cRRFemurGyre, cRMFemurGyre, cRFFemurGyre, cLRFemurGyre, cLMFemurGyre, cLFFemurGyre,
	cRRTibiaGyre, cRMTibiaGyre, cRFTibiaGyre, cLRTibiaGyre, cLMTibiaGyre, cLFTibiaGyre
#ifdef c4DOF
	, cRRTarsGyre, cRMTarsGyre, cRFTarsGyre, cLRTarsGyre, cLMTarsGyre, cLFTarsGyre
#endif
#ifdef cTurretRotGyre
	, cTurretRotGyre, cTurretTiltGyre
#endif
};

static const int16_t cDefaultServoOffsets[] = {
	cRRCoxaOff,  cRMCoxaOff,  cRFCoxaOff,  cLRCoxaOff,  cLMCoxaOff,  cLFCoxaOff,
	cRRFemurOff, cRMFemurOff, cRFFemurOff, cLRFemurOff, cLMFemurOff, cLFFemurOff,
	cRRTibiaOff, cRMTibiaOff, cRFTibiaOff, cLRTibiaOff, cLMTibiaOff, cLFTibiaOff
#ifdef c4DOF
	, cRRTarsOff, cRMTarsOff, cRFTarsOff, cLRTarsOff, cLMTarsOff, cLFTarsOff
#endif
#ifdef cTurretRotOff
	, cTurretRotOff, cTurretTiltOff
#endif
};

/*typedef struct {
	uint8_t         id;
	LSS_ConfigGyre  gyre;
	int16_t         offset;
	int16_t         max_speed;
} servo_info_t; */
typedef struct {
	const char    *leg_name;
//	servo_info_t  coxa;
//	servo_info_t  femur;
//	servo_info_t  tibia;
	bool          leg_found;
} leg_info_t;

leg_info_t legs[] = {
	{"Right Rear"},	{"Right Middle"}, {"Right Front"},
	{"Left Rear"}, {"Left Middle"}, 	{"Left Front"}
};

/*leg_info_t legs[] = {
	{"Right Rear", {cRRCoxaPin, cRRCoxaGyre, cRRCoxaOff, cServoSpeed}, {cRRFemurPin, cRRFemurGyre, cRRFemurOff, cServoSpeed}, {cRRTibiaPin, cRRTibiaGyre, cRRTibiaOff, cServoSpeed}}
	{"Right Middle", {cRMCoxaPin, cRMCoxaGyre, cRMCoxaOff, cServoSpeed}, {cRMFemurPin, cRMFemurGyre, cRMFemurOff, cServoSpeed}, {cRMTibiaPin, cRMTibiaGyre, cRMTibiaOff, cServoSpeed}},
	{"Right Front", {cRFCoxaPin, cRFCoxaGyre, cRFCoxaOff, cServoSpeed}, {cRFFemurPin, cRFFemurGyre, cRFFemurOff, cServoSpeed}, {cRFTibiaPin, cRFTibiaGyre, cRFTibiaOff, cServoSpeed}},

	{"Left Rear", {cLRCoxaPin, cLRCoxaGyre, cLRCoxaOff, cServoSpeed}, {cLRFemurPin, cLRFemurGyre, cLRFemurOff, cServoSpeed}, {cLRTibiaPin, cLRTibiaGyre, cLRTibiaOff, cServoSpeed}},
	{"Left Middle", {cLMCoxaPin, cLMCoxaGyre, cLMCoxaOff, cServoSpeed}, {cLMFemurPin, cLMFemurGyre, cLMFemurOff, cServoSpeed}, {cLMTibiaPin, cLMTibiaGyre, cLMTibiaOff, cServoSpeed}},
	{"Left Front", {cLFCoxaPin, cLFCoxaGyre, cLFCoxaOff, cServoSpeed}, {cLFFemurPin, cLFFemurGyre, cLFFemurOff, cServoSpeed}, {cLFTibiaPin, cLFTibiaGyre, cLFTibiaOff, cServoSpeed}},

};
*/
#define COUNT_LEGS (sizeof(legs)/sizeof(legs[0]))

//============================================================================================
// Check to see if the servos have been configured properly
//============================================================================================
void ServoDriver::checkAndInitServosConfig(bool force_defaults)
{

	use_servos_moveT = false;

	// lets see if we need to set the Origin and GYRE...
	// start off if the GYRE does not match what we believe we need... Set everything
	if (!force_defaults) {
		Serial.println(">>> Check Servo Settings <<<");
		for (uint8_t leg = 0; leg < COUNT_LEGS; leg++) {
			myLSS.setServoID(cPinTable[FIRSTCOXAPIN + leg]);
			// BUGBUG we did reset but session did not equal config...
			LSS_ConfigGyre cgyre_session =  myLSS.getGyre(LSS_QuerySession);
			LSS_ConfigGyre cgyre_config =  myLSS.getGyre(LSS_QueryConfig);
			if (myLSS.getLastCommStatus() == LSS_CommStatus_ReadSuccess) {
				Serial.printf("%d:%d:%d:%d ", myLSS.getServoID(), cGyreTable[FIRSTCOXAPIN + leg], cgyre_session, cgyre_config);
				if (cGyreTable[FIRSTCOXAPIN + leg] != cgyre_config) {
					Serial.println("\n *** checkAndInitServosConfig: Need to configure servos ***");
					force_defaults = true;  // reuse variable
					break;
				}
			}
		}

	}

	// Either the caller to setup defaults or quick check above said to...
	for (uint8_t leg = 0; leg < COUNT_LEGS; leg++) {
		legs[leg].leg_found = true;
		myLSS.setServoID(cPinTable[FIRSTCOXAPIN + leg]);
		if (myLSS.getStatus() == LSS_StatusUnknown) legs[leg].leg_found = false;
		else {
			myLSS.setMotionControlEnabled(0);
			myLSS.setAngularHoldingStiffness(4, LSS_SetSession);
			myLSS.setAngularStiffness(-4, LSS_SetSession);
			myLSS.setFilterPositionCount(3, LSS_SetSession);
			myLSS.setGyre(cGyreTable[FIRSTCOXAPIN + leg], LSS_SetSession);

			if (force_defaults) {
				Serial.print("@");
				myLSS.setOriginOffset(cDefaultServoOffsets[FIRSTCOXAPIN + leg], LSS_SetSession);
			}
		}

		myLSS.setServoID(cPinTable[FIRSTFEMURPIN + leg]);
		if (myLSS.getStatus() == LSS_StatusUnknown) legs[leg].leg_found = false;
		else {
			myLSS.setMotionControlEnabled(0);
			myLSS.setAngularHoldingStiffness(4, LSS_SetSession);
			myLSS.setAngularStiffness(-4, LSS_SetSession);
			myLSS.setFilterPositionCount(3, LSS_SetSession);
			myLSS.setGyre(cGyreTable[FIRSTFEMURPIN + leg], LSS_SetSession);

			if (force_defaults) {
				myLSS.setOriginOffset(cDefaultServoOffsets[FIRSTFEMURPIN + leg], LSS_SetSession);
			}
		}
		myLSS.setServoID(cPinTable[FIRSTTIBIAPIN + leg]);
		if (myLSS.getStatus() == LSS_StatusUnknown) legs[leg].leg_found = false;
		else {
			myLSS.setMotionControlEnabled(0);
			myLSS.setAngularHoldingStiffness(4, LSS_SetSession);
			myLSS.setAngularStiffness(-4, LSS_SetSession);
			myLSS.setFilterPositionCount(3, LSS_SetSession);
			myLSS.setGyre(cGyreTable[FIRSTTIBIAPIN + leg], LSS_SetSession);

			if (force_defaults) {
				myLSS.setOriginOffset(cDefaultServoOffsets[FIRSTTIBIAPIN + leg], LSS_SetSession);
			}
		}
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

	TMReset(); // reset our servo list

	// Reset all servos in case any are in error state
	Serial.println("ServoDriver::Init - reset all servos");
	myLSS.setServoID(LSS_BroadcastID);
	myLSS.reset();
	delay(1500);  // make sure all servos reset.

	for (int i = 0; i < NUMSERVOS; i++) {
		// Set the id
		int servo_id = pgm_read_byte(&cPinTable[i]);
		uint8_t index = TMAddID(servo_id); // bugbug assuming some position stuff...
		myLSS.setServoID(servo_id);

		// Now try to get it's position
		DBGSerial.print("Servo(");
		DBGSerial.print(i, DEC);
		DBGSerial.print("): ");
		DBGSerial.print(servo_id, DEC);

		pos = myLSS.getPosition();
		if (myLSS.getLastCommStatus() == LSS_CommStatus_ReadSuccess) {
			DBGSerial.printf(" %d\n", pos);
			TMSetTargetByIndex(index, pos);
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
	// Check and set the servo register values
#ifdef RESET_LSS_SERVO_SETTINGS // turn this on if you want to reset servo settings...
	checkAndInitServosConfig(true);  //sets servo config to MJS version
#else
	checkAndInitServosConfig();  //sets servo config to MJS version
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
	// Sort of duplicate stuff now... May clean up.
	TMSetTargetByIndex(FIRSTCOXAPIN + LegIndex, sCoxaAngle1);  // What order should we store these values?
	TMSetTargetByIndex(FIRSTFEMURPIN + LegIndex, sFemurAngle1);
	TMSetTargetByIndex(FIRSTTIBIAPIN + LegIndex, sTibiaAngle1);
#ifdef c4DOF
	TMSetTargetByIndex(FIRSTTARSPIN + LegIndex, sTarsAngle);
#endif

	g_goal_servo_pos[FIRSTCOXAPIN + LegIndex] = sCoxaAngle1;  // What order should we store these values?
	g_goal_servo_pos[FIRSTFEMURPIN + LegIndex] = sFemurAngle1;
	g_goal_servo_pos[FIRSTTIBIAPIN + LegIndex] = sTibiaAngle1;
#ifdef c4DOF
	g_awGoalAXTarsPos[FIRSTTARSPIN + LegIndex] = sTarsAngle1;
#endif
#ifdef DEBUG_SERVOS
	if (g_fDebugOutput) {
		DBGSerial.print(LegIndex, DEC);
		DBGSerial.print("C");
		DBGSerial.print(sCoxaAngle1, DEC);
		DBGSerial.print("),F");
		DBGSerial.print(sFemurAngle1, DEC);
		DBGSerial.print("),(");
		DBGSerial.print("T");
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
		if (use_servos_moveT) {
			for (int i = 0; i < NUMSERVOS; i++) {
				if (g_cur_servo_pos[i] != g_goal_servo_pos[i]) {
					g_cur_servo_pos[i] = g_goal_servo_pos[i];
					// Set the id
					int servo_id = pgm_read_byte(&cPinTable[i]);
					myLSS.setServoID(servo_id);
					myLSS.moveT(g_goal_servo_pos[i], wMoveTime);
				}
			}
		} else {
			TMSetupMove(wMoveTime);
			if (g_fDebugOutput || servo_debug) TMPrintDebugInfo();
			TMStep(true); // force the first step..
		}
	} 	else {
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
static uint8_t g_iIdleServoNum = (uint8_t) - 1;
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
bool ServoDriver::MakeSureServosAreOn(void)
{
	boolean servos_reset = false;
	if (ServosEnabled) {
		if (!g_fServosFree)
			return false;    // we are not free

		g_InputController.AllowControllerInterrupts(false);    // If on xbee on hserial tell hserial to not processess...

		LSS::genericWrite(LSS_BroadcastID, LSS_ActionHold); // Tell all of the servos to hold a position.
		delay(50);
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
				                 (servo_status < (sizeof(lss_status_text) / sizeof(lss_status_text[0]))) ? lss_status_text[servo_status] : "?",
				                 servo_status);
				myLSS.reset();
				servos_reset = true;
				break;
			}
		}

		if (servos_reset) {
			delay(3000);  // give servos some time to reset.

			// Make sure the servos values are reset as well
			checkAndInitServosConfig();
			// try again to hold servos.
			LSS::genericWrite(LSS_BroadcastID, LSS_ActionHold); // Tell all of the servos to hold a position
			delay(50);
		}

		g_InputController.AllowControllerInterrupts(true);
		g_fServosFree = false;
	}

	return (servos_reset);
}

//==============================================================================
// BackgroundProcess - Allows us to have some background processing for those
//    servo drivers that need us to do things like polling...
//==============================================================================
void  ServoDriver::BackgroundProcess(void)
{
	if (!use_servos_moveT)
		TMStep(false); // force the first step..
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
	DBGSerial.println(F("A - Toggle LSS speed control"));
	DBGSerial.println(F("L - Toggle LSS Servo Debug output"));
	DBGSerial.println(F("F <FPS> - Set FPS for Interpolation mode"));
	DBGSerial.println(F("B <FPC> - Set Filter Position Count for Interpolation mode"));
	DBGSerial.println(F("S - Track Servos"));
#ifdef OPT_FIND_SERVO_OFFSETS
	DBGSerial.println(F("O[m] - Enter Servo offset mode (m - manual mode)"));
	DBGSerial.println(F("C - clear Servo Offsets"));
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
		return true;
	}

	else if ((bLen == 1) && ((*psz == 't') || (*psz == 'T'))) {
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
	else if (((*psz == 'p') || (*psz == 'P'))) {
		TCServoPositions();
	}
	else if ((*psz == 's') || (*psz == 'S')) {
		TCTrackServos();
	}

	else if ((bLen == 1) && ((*psz == 'a') || (*psz == 'A'))) {
		use_servos_moveT = !use_servos_moveT;
		if (use_servos_moveT) {
			DBGSerial.println(F("Use Servo moveT"));
		}
		else {
			DBGSerial.println(F("Use Software Interplation"));
		}
		TMConfigureServos();
		return true;
	}
	else if ((bLen == 1) && ((*psz == 'l') || (*psz == 'L'))) {
		servo_debug = !servo_debug;
		if (servo_debug) {
			DBGSerial.println(F("LSS Debug output enabled"));
		}
		else {
			DBGSerial.println(F("LSS Debug output disabled"));
		}
		return true;
	}
	else if ((*psz == 'f') || (*psz == 'F')) {
		uint16_t fps = 0;
		psz++;
		while (*psz == ' ') psz++; // ignore any blanks.
		while ((*psz >= '0') && (*psz <= '9')) {
			fps = fps * 10 + *psz++ - '0';
		}
		if (fps == 0) fps = DEFAULT_FRAMES_PER_SECOND;
		tmCycleTime = 1000000 / fps;
		DBGSerial.printf("Set FPS to: %u Cycle time\n", fps, tmCycleTime);
	}
	else if ((*psz == 'b') || (*psz == 'B')) {
		uint16_t fpc = 0;
		psz++;
		while (*psz == ' ') psz++; // ignore any blanks.
		while ((*psz >= '0') && (*psz <= '9')) {
			fpc = fpc * 10 + *psz++ - '0';
		}
		DBGSerial.printf("Set FPC to: %u\n", fpc);
		for (uint8_t servo = 0; servo < tmServoCount; servo++) {
			myLSS.setServoID(tmServos[servo].id);
			myLSS.setFilterPositionCount(fpc, LSS_SetSession);
		}

	}


#ifdef OPT_FIND_SERVO_OFFSETS
	else if ((*psz == 'o') || (*psz == 'O')) {
		psz++;
		// Did the user want to force manual mode?
		FindServoOffsets((*psz == 'm') || (*psz == 'M'));
		return true;
	}
	else if ((bLen == 1) && ((*psz == 'c') || (*psz == 'C'))) {
		ClearServoOffsets();
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
		DBGSerial.printf("%5d(%u:%u) %5d(%u:%u) %5d(%u:%u) | %5d(%u:%u) %5d(%u:%u) %5d(%u:%u)\n",
		                 servo_pos[FIRSTTIBIAPIN + legs],	cPinTable[FIRSTTIBIAPIN + legs], 	  	servo_status[FIRSTTIBIAPIN + legs],
		                 servo_pos[FIRSTFEMURPIN + legs],	cPinTable[FIRSTFEMURPIN + legs], 	  	servo_status[FIRSTFEMURPIN + legs],
		                 servo_pos[FIRSTCOXAPIN + legs],	cPinTable[FIRSTCOXAPIN + legs], 	  	servo_status[FIRSTCOXAPIN + legs],
		                 servo_pos[FIRSTCOXAPIN + legs + 3], cPinTable[FIRSTCOXAPIN + legs + 3],   	servo_status[FIRSTCOXAPIN + legs + 3],
		                 servo_pos[FIRSTFEMURPIN + legs + 3], cPinTable[FIRSTFEMURPIN + legs + 3], 	servo_status[FIRSTFEMURPIN + legs + 3],
		                 servo_pos[FIRSTTIBIAPIN + legs + 3], cPinTable[FIRSTTIBIAPIN + legs + 3], 	servo_status[FIRSTTIBIAPIN + legs + 3]);
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

void ServoDriver::FindServoOffsets(bool force_manual_mode)
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
	short servo_index;       // which servo number
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

	// LSS_ActionMove is DOA so have to roll our own!
	for (uint8_t i = 0; i < tmServoCount; i++) TMSetTargetByIndex(i, 0); // set all to 0

	// Don't go to zero if told to go into manual mode... If not told to be manual
	// next loop will see if any servos are not in valid state and again reset...
	if (!force_manual_mode) {
		TMTimedMove(500);
		// hack to make sure this did not error any of the servos out...
		force_manual_mode = MakeSureServosAreOn(); // returns true if any servo was in error
	}
	//LSS::genericWrite(LSS_BroadcastID, LSS_ActionMove, 0,
	//                  LSS_ActionParameterTime, 500);  // move in half second

	Serial.println("\nUpdate Servos Offsets and their rotation direction(Gyre)");
	Serial.println("Current Servo Information");
	// Lets show some information about each of the servos.
	for (servo_index = 0; servo_index < NUMSERVOS; servo_index++) {
		asOffsets[servo_index] = 0;
		myLSS.setServoID(cPinTable[servo_index]);
		Serial.print("\tServo: ");
		Serial.print(apszLegs[servo_index % CNT_LEGS]);
		Serial.print(apszLJoints[servo_index / CNT_LEGS]);
		Serial.print("(");
		Serial.print(cPinTable[servo_index], DEC);
		Serial.print(") Pos:");
		Serial.print(myLSS.getPosition(), DEC);
		Serial.print("\tO:");
		Serial.print(myLSS.getOriginOffset(), DEC);
		Serial.print(":");
		Serial.print(myLSS.getOriginOffset(LSS_QueryConfig), DEC);
		Serial.print("\tG:");
		Serial.print(myLSS.getGyre(), DEC);
		Serial.print(":");
		Serial.print(myLSS.getGyre(LSS_QueryConfig), DEC);
		Serial.print("\tEMC:");
		Serial.print(myLSS.getIsMotionControlEnabled(), DEC);
		//Serial.print(":");
		//Serial.print(myLSS.getIsMotionControlEnabled(LSS_QueryConfig), DEC);
		Serial.print("\tFPC:");
		Serial.print(myLSS.getFilterPositionCount(), DEC);
		Serial.print(":");
		Serial.print(myLSS.getFilterPositionCount(LSS_QueryConfig), DEC);
		Serial.print("\tAS:");
		Serial.print(myLSS.getAngularStiffness(), DEC);
		Serial.print(":");
		Serial.print(myLSS.getAngularStiffness(LSS_QueryConfig), DEC);
		Serial.print("\tAH:");
		Serial.print(myLSS.getAngularHoldingStiffness(), DEC);
		Serial.print(":");
		Serial.print(myLSS.getAngularHoldingStiffness(LSS_QueryConfig), DEC);
		Serial.print("\tAR:");
		Serial.print(myLSS.getAngularRange(), DEC);
		Serial.print(":");
		Serial.println(myLSS.getAngularRange(LSS_QueryConfig), DEC);
	}

	myLSS.setServoID(LSS_BroadcastID);
	myLSS.setColorLED(LSS_LED_Black);
// OK lets move all of the servos to their zero point.
	Serial.println("\nThe Goal is to align the top two servo pivots (Coxa and Femur) to be parallel to ground");
	Serial.println("And the Tibia should be at a right angle to the ground\n");
	Serial.println("Enter $-Exit, +- changes, *-change servo");
	Serial.println("    0-n Chooses a leg, C-Coxa, F-Femur, T-Tibia");
	Serial.println("    m - manually move mode to get close");

	if (force_manual_mode) Serial.println("*** starting off in manual mode ***");

	servo_index = 0;
	bool data_received = false;
	while (!fExit) {
		if (force_manual_mode ) {  // warning we reused this parameter...
			force_manual_mode = false;
			Serial.println("*** Entered Manual mode, press any key to exit ***");
			while (Serial.read() != -1);
			// Tell all servos to go limp...
			LSS::genericWrite(LSS_BroadcastID, LSS_ActionLimp); // Tell all of the servos to go limp
			while (Serial.read() == -1);  // wait for some new data
			while (Serial.read() != -1);
			Serial.println("*** Manual Mode Exited ***");
			LSS::genericWrite(LSS_BroadcastID, LSS_ActionHold); // Tell all of the servos to hold again.
			// Now we need to read in the current positions to work with.
			for (uint8_t i = 0; i < tmServoCount; i++) {
				myLSS.setServoID(cPinTable[i]);
				asOffsets[i] = myLSS.getPosition();	// get the position
				TMSetTargetByIndex(i, asOffsets[i]); // called twice to make sure source and dest are set
				TMSetTargetByIndex(i, asOffsets[i]); //
				Serial.printf("%u:%d ", cPinTable[servo_index], asOffsets[i]);
				// set leds to get an idea of which ones may have moved...
				myLSS.setColorLED((abs(asOffsets[i]) < 10) ? LSS_LED_Black : LSS_LED_Red);
			}
			Serial.println();
		}
		if (fNew) {
			uint8_t servo_id = cPinTable[servo_index];
			myLSS.setServoID(servo_id);
			myLSS.setColorLED(LSS_LED_Green);
			Serial.print("Servo: ");
			Serial.print(apszLegs[servo_index % CNT_LEGS]);
			Serial.print(apszLJoints[servo_index / CNT_LEGS]);
			Serial.print("(");
			Serial.print(servo_id, DEC);
			Serial.println(")");
			// Again avoid MoveT
			TMSetTargetByID(servo_id, asOffsets[servo_index]);
			TMTimedMove(250);
			TMSetTargetByID(servo_id, asOffsets[servo_index] + 100);
			TMTimedMove(250);
			TMSetTargetByID(servo_id, asOffsets[servo_index] - 100);
			TMTimedMove(250);
			TMSetTargetByID(servo_id, asOffsets[servo_index]);
			TMTimedMove(250);
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
					servo_index++;
					if (servo_index == CNT_LEGS * NUMSERVOSPERLEG)
						servo_index = 0;
				}
				data_received = false;
			}
			else {
				data_received = true;
				if (data == '$')
					fExit = true; // not sure how the keypad will map so give NL, CR, LF... all implies exit
				else if ((data == 'm') || (data == 'M')) {
					force_manual_mode = true; // reused parameter...
				}
				else if ((data == '+') || (data == '-')) {
					if (data == '+')
						asOffsets[servo_index] += 5;    // increment by 5us
					else
						asOffsets[servo_index] -= 5;    // increment by 5us

					Serial.print("    ");
					Serial.println(asOffsets[servo_index], DEC);

					TMSetTargetByID(cPinTable[servo_index], asOffsets[servo_index]);
					TMTimedMove(100);
					myLSS.setColorLED(LSS_LED_Red);
				}
				else if ((data >= '0') && (data <= '5')) {
					// direct enter of which servo to change
					fNew = true;
					//servo_index = (servo_index % NUMSERVOSPERLEG) + (data - '0') * NUMSERVOSPERLEG;
					servo_index = (servo_index / CNT_LEGS) * CNT_LEGS + (data - '0');
				}
				else if ((data == 'c') || (data == 'C')) {
					fNew = true;
					//servo_index = (servo_index / NUMSERVOSPERLEG) * NUMSERVOSPERLEG + 0;
					servo_index = servo_index % CNT_LEGS;
				}
				else if ((data == 'f') || (data == 'F')) {
					fNew = true;
					servo_index = FIRSTFEMURPIN + servo_index % CNT_LEGS;
				}
				else if ((data == 't') || (data == 'T')) {
					// direct enter of which servo to change
					fNew = true;
					servo_index = FIRSTTIBIAPIN + servo_index % CNT_LEGS;
				}
				else if (data == '*') {
					// direct enter of which servo to change
					fNew = true;
					servo_index++;
					if (servo_index == CNT_LEGS * NUMSERVOSPERLEG)
						servo_index = 0;
				}
			}
		}
	}
	Serial.print("Find Servo exit ");
	for (servo_index = 0; servo_index < NUMSERVOS; servo_index++) {
		myLSS.setServoID(cPinTable[servo_index]);
		Serial.print("Servo: ");
		Serial.print(apszLegs[servo_index / NUMSERVOSPERLEG]);
		Serial.print(apszLJoints[servo_index % NUMSERVOSPERLEG]);
		Serial.print("Session Offset: ");
		Serial.print(myLSS.getOriginOffset(), DEC);
		Serial.print(" Delta: ");
		Serial.println(asOffsets[servo_index]);
	}

	Serial.print("\nSave Changes? Y/N/C(choose): ");

	//get user entered data
	while (((data = Serial.read()) == -1) || ((data >= 10) && (data <= 15)))
		;

	if ((data == 'Y') || (data == 'y') || (data == 'c') || (data == 'C')) {
		// Ok they asked for the data to be saved.  So for each servo we will update their Gyre and Offset
		// settings.
		//
		while (Serial.read() != -1) ;
		bool manually_choose = (data == 'c') || (data == 'C');
		for (servo_index = 0; servo_index < NUMSERVOS; servo_index++) {
			myLSS.setServoID(cPinTable[servo_index]);
			Serial.print("Servo: ");
			Serial.print(apszLegs[servo_index % CNT_LEGS]);
			Serial.print(apszLJoints[servo_index / CNT_LEGS]);
			Serial.print("(");
			Serial.print(cPinTable[servo_index], DEC);
			Serial.print(")");

			Serial.print(" Gyre: ");
			Serial.println(cGyreTable[servo_index], DEC);
			myLSS.setGyre(cGyreTable[servo_index], LSS_SetConfig);
			myLSS.setGyre(cGyreTable[servo_index], LSS_SetSession);
			Serial.print(" Config Servo Offset: From: ");
			int16_t origin_offset = myLSS.getOriginOffset(); // should use the working set...
			Serial.print(origin_offset, DEC);
			Serial.print(" to: ");
			origin_offset += asOffsets[servo_index];
			Serial.print(origin_offset, DEC);
			if (manually_choose) {
				Serial.print(" Update ?");
				int ch;
				while ((ch = Serial.read()) == -1);
				if ((ch == 'Y') || (ch == 'y')) {
					myLSS.setOriginOffset(origin_offset, LSS_SetConfig);
					myLSS.setOriginOffset(origin_offset, LSS_SetSession);
					Serial.print("*Updated*");
				}

			} else {
				myLSS.setOriginOffset(origin_offset, LSS_SetConfig);
				myLSS.setOriginOffset(origin_offset, LSS_SetSession);
			}
		}

		Serial.println("Find Offsets complete");
		// Not sure if we need to reset or not???
		/* myLSS.setServoID(LSS_BroadcastID);
		myLSS.reset();
		delay(1500);  // make sure all servos reset. */
	}
	else {
		//void LoadServosConfig();
	}
	g_ServoDriver.FreeServos();

}

void ClearServoOffsets() {
	Serial.println("This will clear out all of the servo offsets and Gyre, do you wish to continue (Y/N):");
	while (Serial.read() != -1);
	int ch;
	while ((ch = Serial.read()) == -1) ;
	while (Serial.read() != -1);

	if ((ch == 'y') || (ch == 'Y')) {
		for (int servo_index = 0; servo_index < NUMSERVOS; servo_index++) {
			Serial.printf("%u ", cPinTable[servo_index]);
			myLSS.setServoID(cPinTable[servo_index]);
			delay(10);
			myLSS.setOriginOffset(0, LSS_SetConfig);
			delay(10);
			myLSS.setOriginOffset(0, LSS_SetSession);
			delay(10);
			myLSS.setGyre(LSS_GyreClockwise, LSS_SetConfig);
			delay(10);
			myLSS.setGyre(LSS_GyreClockwise, LSS_SetSession);
		}
		Serial.println();
		for (int servo_index = 0; servo_index < NUMSERVOS; servo_index++) {
			myLSS.setServoID(cPinTable[servo_index]);
			Serial.printf("%d:%x(%x):%x(%x) ", myLSS.getServoID(),
			              myLSS.getGyre(LSS_QueryConfig), myLSS.getGyre(LSS_QuerySession),
			              myLSS.getOriginOffset(LSS_QueryConfig), myLSS.getOriginOffset(LSS_QuerySession));
		}
		Serial.println("\nClear complete you should probably restart the program");
	}
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


//=============================================================================
// Do our own timed moves support functions.
//=============================================================================
// Add in experiments to see if they improve servo...

void ServoDriver::TMReset() {
	tmServoCount = 0;
}

// Add a servo to the list.
uint8_t ServoDriver::TMAddID(uint8_t id) {
	tmServos[tmServoCount].id = id;
	tmServos[tmServoCount].pos_repeated_count = 0;
	tmServos[tmServoCount].target_pos = 0;
	tmServos[tmServoCount].starting_pos = 0;
	tmServoCount++;
	return tmServoCount - 1;
}

void ServoDriver::TMInitWithCurrentservoPositions() {
	for (uint8_t servo = 0; servo < tmServoCount; servo++) {
		myLSS.setServoID(tmServos[servo].id);
		myLSS.setMotionControlEnabled(0);
		tmServos[servo].starting_pos = myLSS.getPosition();
		tmServos[servo].target_pos = tmServos[servo].starting_pos;
	}
}
void ServoDriver::TMConfigureServos() {
	int em_mode = use_servos_moveT ? 1 : 0;
	DBGSerial.printf("Set Servo EM=%u\n", em_mode);
	for (uint8_t servo = 0; servo < tmServoCount; servo++) {
		myLSS.setServoID(tmServos[servo].id);
		myLSS.setMotionControlEnabled(em_mode);
	}
}

bool ServoDriver::TMSetTargetByID(uint8_t id, int16_t target) {
	for (uint8_t servo = 0; servo < tmServoCount; servo++) {
		if (id == tmServos[servo].id) {
			TMSetTargetByIndex(servo, target);
			return true;
		}
	}
	return false;
}

void ServoDriver::TMSetTargetByIndex(uint8_t index, int16_t target) {
	tmServos[index].starting_pos = tmServos[index].target_pos; // set source as last target
	tmServos[index].target_pos = target;
}
void ServoDriver::TMSetupMove(uint32_t move_time) {
	tmMovetime = move_time * 1000; // convert to us

	// setup to maybe do dynmic cycle times... First maybe compute max delta...
#if DYNAMIC_FPS
	int max_delta = 0;
	int second_max_delta = 0;
	for (uint8_t servo = 0; servo < tmServoCount; servo++) {
		myLSS.setServoID(tmServos[servo].id);
		if (tmSetupServos) myLSS.setMotionControlEnabled(0);
		if (tmServos[servo].starting_pos == -1) tmServos[servo].starting_pos = myLSS.getPosition();
		int servo_delta = abs(tmServos[servo].target_pos - tmServos[servo].starting_pos);
		if (servo_delta > max_delta) { second_max_delta = max_delta;  max_delta = servo_delta;}
		else if (servo_delta > second_max_delta) second_max_delta = servo_delta;
	}
	// lets take some guesses on good frame time...

	uint32_t max_frames_for_move_time = (MAX_FPS * move_time) / 1000;
	tmCyclesLeft = (second_max_delta) ? max_delta * second_max_delta : max_delta;
	if (tmCyclesLeft > max_frames_for_move_time) tmCyclesLeft = max_frames_for_move_time;
	if (!tmCyclesLeft) tmCyclesLeft = 1;
	tmCycleTime = tmMovetime / tmCyclesLeft;

	for (uint8_t servo = 0; servo < tmServoCount; servo++) {
		tmServos[servo].pos = tmServos[servo].starting_pos;
		tmServos[servo].cycle_delta = ((tmServos[servo].target_pos - tmServos[servo].starting_pos)); // set it first to get into floating point
		tmServos[servo].cycle_delta /= tmCyclesLeft;
	}


#else
	tmCyclesLeft = (tmMovetime + tmCycleTime / 2) / tmCycleTime;
	for (uint8_t servo = 0; servo < tmServoCount; servo++) {
		myLSS.setServoID(tmServos[servo].id);
		if (tmSetupServos) myLSS.setMotionControlEnabled(0);
		tmServos[servo].pos = tmServos[servo].starting_pos;
		tmServos[servo].cycle_delta = ((tmServos[servo].target_pos - tmServos[servo].starting_pos)); // set it first to get into floating point
		tmServos[servo].cycle_delta /= tmCyclesLeft;
	}
#endif

	tmSetupServos = false;
	tmTimer = 0;

}

int  ServoDriver::TMStep(bool wait) {
	if (!tmCyclesLeft) return 0;
	// BUGBUG not processing wait yet... but normally
	// can set false so can return between steps to do other stuff.
	int time_left_in_cycle = (int)(tmCycleTime - tmTimer);
	if (!wait && (time_left_in_cycle > (int)tmMinNotwaitTime)) return time_left_in_cycle; //

	//static uint8_t pos_count = 0;
	//Serial.print("!"); pos_count++; if (pos_count == 80) {pos_count=0; Serial.println();}

	while (tmTimer < tmCycleTime) ;
	// how many cycles.
	for (uint8_t servo = 0; servo < tmServoCount; servo++) {
		if (tmServos[servo].cycle_delta) {
			int cur_pos = tmServos[servo].pos;
			tmServos[servo].pos += tmServos[servo].cycle_delta;
			int next_pos = tmServos[servo].pos;
			if (tmCyclesLeft == 1) {
				next_pos = tmServos[servo].target_pos;
				tmServos[servo].starting_pos = tmServos[servo].target_pos; // set source as last target
			} else {
				if (tmServos[servo].cycle_delta < 0) {
					if (next_pos < tmServos[servo].target_pos) next_pos = tmServos[servo].target_pos;
				} else if (next_pos > tmServos[servo].target_pos) next_pos = tmServos[servo].target_pos;
			}
			if (next_pos != cur_pos) {
				tmServos[tmServoCount].pos_repeated_count = 0;
#if (OUTPUT_ONLY_CHANGED_SERVOS == 1)
				myLSS.setServoID(tmServos[servo].id);
				myLSS.move(next_pos);
#endif
				if (next_pos == tmServos[servo].target_pos) {
					tmServos[servo].cycle_delta = 0; // servo done
					tmServos[servo].starting_pos = tmServos[servo].target_pos; // set source as last target
				}
			} else if (tmServos[tmServoCount].pos_repeated_count < OUTPUT_SAME_POS_COUNT) {
				tmServos[tmServoCount].pos_repeated_count++;
#if (OUTPUT_ONLY_CHANGED_SERVOS == 1)
				myLSS.setServoID(tmServos[servo].id);
				myLSS.move(next_pos);
#endif
			}

		}
#if (OUTPUT_ONLY_CHANGED_SERVOS == 0)  // output every servo on every step.
		if (tmServos[tmServoCount].pos_repeated_count < OUTPUT_SAME_POS_COUNT) {
			myLSS.setServoID(tmServos[servo].id);
			myLSS.move(tmServos[servo].pos);
		}
#endif
	}
	tmCyclesLeft--;
	tmTimer -= tmCycleTime;
#if DYNAMIC_FPS
	tmMovetime -= tmCycleTime;
	if (tmCyclesLeft == 1) tmCycleTime = tmMovetime; // last frame setup to get to the right timing
#endif
	return 0;
}

void ServoDriver::TMTimedMove(uint32_t move_time) {
	TMSetupMove(move_time);
	//TMPrintDebugInfo();
	elapsedMillis em;
	while (em < move_time) TMStep();
}

void ServoDriver::TMPrintDebugInfo() {
#ifdef DBGSerial
	DBGSerial.println("*** TM debug info");
	DBGSerial.printf("Move Time:%u Cyle time:%u cycles:%u\n", tmMovetime, tmCycleTime, tmCyclesLeft);
	DBGSerial.println("ID\t Start\t End\tCyle Delta");
	for (uint8_t servo = 0; servo < tmServoCount; servo++) {
		DBGSerial.printf("  %u\t%d\t%d\t%f\n", tmServos[servo].id, tmServos[servo].starting_pos, tmServos[servo].target_pos, tmServos[servo].cycle_delta);
	}
#endif
}
