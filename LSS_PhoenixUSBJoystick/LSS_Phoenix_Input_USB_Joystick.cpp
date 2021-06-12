
//====================================================================
//Project Lynxmotion Phoenix
//Description: Phoenix, control file.
//The control input subroutine for the phoenix software is placed in this file.
//Can be used with V2.0 and above
//Configuration version: V1.0
//Date: 25-10-2009
//Programmer: Jeroen Janssen (aka Xan)
//             Kurt Eckhardt (aka KurtE) - converted to c ported to Arduino...
//
//Hardware setup: Arbotix Commander version - Try to emulate most of PS2, but PS2 has 16 buttons and Commander 
// has 10. so some things may not be there, others may be doubled up.
// 
//NEW IN V1.0
//- First Release
//
//Walk method 1:
//- Left StickWalk/Strafe
//- Right StickRotate
//
//Walk method 2:
//- Left StickDisable
//- Right StickWalk/Rotate
//
//
// Quick and Dirty description of controls... WIP
// In most cases I try to mention what button on the PS2 things coorespond to..
// On/OFF - Turning the commander 2 on and off (PS2 start button)
// R1 - options (Change walk gait, Change Leg in Single Leg, Change GP sequence) (Select on PS2)
// R2 - Toggle walk method...  Run Sequence in GP mode
// R3 - Walk method (Not done yet) - (PS2 R3)
// L4 - Ballance mode on and off
// L5 - Stand/Sit (Triangle on PS2)
// L6+Right Joy UP/DOWN - Body up/down - (PS2 Dpad Up/Down)
// L6+Right Joy Left/Right - Speed higher/lower - (PS2 DPad left/right)
// Right Top(S7) - Cycle through options of Normal walk/Double Height/Double Travel) - (PS2 R1, R2)
// Left Top(S8) - Cycle through modes (Walk, Translate, Rotate, Single Leg) (PS2: Circle, X, L1, L2)

// Note: Left some descriptions of PS2 stuff, especially parts still left to Map/Implement.
//
//[Walk Controls]
//- selectSwitch gaits
//- Left Stick(Walk mode 1) Walk/Strafe
// (Walk mode 2) Disable
//- Right Stick(Walk mode 1) Rotate, 
//(Walk mode 2) Walk/Rotate
//- R1Toggle Double gait travel speed
//- R2Toggle Double gait travel length
//
//[Shift Controls]
//- Left StickShift body X/Z
//- Right StickShift body Y and rotate body Y
//
//[Rotate Controls]
//- Left StickRotate body X/Z
//- Right StickRotate body Y
//
//[Single leg Controls] - Need to check...
//- selectSwitch legs
//- Left StickMove Leg X/Z (relative)
//- Right StickMove Leg Y (absolute)
//- R2Hold/release leg position
//
//[GP Player Controls] - How to do sequences???
//- selectSwitch Sequences
//- R2Start Sequence
//
//====================================================================
// [Include files]

#include <Arduino.h>
#include <TeensyDebug.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>

#include "LSS_Phoenix.h"
#include <USBHost_t36.h>

#define DBGSerial Serial

#if __has_include(<TeensyDebug.h>)
#include <TeensyDebug.h>
#endif

#ifdef USBHOST_DEGUG_MEMORY_STREAM
#ifdef ARDUINO_TEENSY41
uint8_t debug_stream_buffer[2048 * 1024] EXTMEM;
extern "C"
{
  extern uint8_t external_psram_size;
}
#else
uint8_t debug_stream_buffer[256 * 1024] DMAMEM;
#endif
DebugMemoryStream USBHostDebugStream(debug_stream_buffer, sizeof(debug_stream_buffer));
#endif


//[CONSTANTS]
enum {
	WALKMODE = 0, TRANSLATEMODE, ROTATEMODE,
#ifdef OPT_SINGLELEG      
	SINGLELEGMODE,
#endif
	MODECNT
};
enum {
	NORM_NORM = 0, NORM_LONG, HIGH_NORM, HIGH_LONG
};
#ifdef USE_ST7789
const char* mode_names[] PROGMEM = { "Walk", "Translate", "Rotate", "Single" };

#endif

#define cTravelDeadZone 6      //The deadzone for the analog input from the remote

#define ARBOTIX_TO  1250        // if we don't get a valid message in this number of mills turn off

#ifndef USER
#define USER 13
#endif

#ifndef MAX_BODY_Y
#define MAX_BODY_Y 100
#endif


// lets define Buttons and Axis mapping for different joysticks.
enum {
	BUT_L1 = 0, BUT_L2, BUT_L3, BUT_R1, BUT_R2, BUT_R3,
	BUT_TRI, BUT_SQ, BUT_X, BUT_CIRC,
	BUT_PS3, BUT_SELECT, BUT_START,
	BUT_HAT_UP, BUT_HAT_DOWN, BUT_HAT_LEFT, BUT_HAT_RIGHT
};
enum { AXIS_LX, AXIS_LY, AXIS_RX, AXIS_LT, AXIS_RT, AXIS_RY };  // Order of PS3

const static uint32_t PS3_BTNS[] = { 0x400, 0x100, 0x2, 0x800, 0x200, 0x4,
	0x1000, 0x8000, 0x4000, 0x2000,
	0x10000, 0x1, 0x8,
	// UP  DN    LFT   RHT
	0x10, 0x40, 0x80, 0x20
};

const static uint32_t PS4_BTNS[] = {
	0x10, 0x40, 0x400, 0x20, 0x80, 0x800,    //L1,L2,??? , R1, R2 ?????
	0x8, 0x1, 0x2, 0x4,         //tri, square, cross, circle
	0x1000, 0x200, 0x100,       // PS, options, Share
	0x10000, 0x40000, 0x80000, 0x20000       // HAT - up, down, left, right
};
const static uint32_t PS4_MAP_HAT_MAP[] = {
	//0x10, 0x30, 0x20, 0x60, 0x40, 0xc0, 0x80, 0x90, 0x00 };
	0x10000, 0x30000, 0x20000, 0x60000, 0x40000, 0xC0000, 0x80000, 0x90000, 0x0 };
  //up,    NE,      right,   SE,      down,    SW,      left,    NW,   ???
 uint32_t const * BTN_MASKS = PS3_BTNS;

//=============================================================================
// Global - Local to this file only...
//=============================================================================
USBHost myusb;
USBHub hub1(myusb);
USBHub hub2(myusb);
USBHIDParser hid1(myusb);
USBHIDParser hid2(myusb);
USBHIDParser hid3(myusb);
JoystickController joystick1(myusb);
#if defined(BLUETOOTH)
	//BluetoothController bluet(myusb, true, "0000");   // Version does pairing to device
	BluetoothController bluet(myusb);   // version assumes it already was paired

  USBDriver* drivers[] = { &hub1, &hub2, &joystick1, &bluet, &hid1, &hid2, &hid3 };
  
  #define CNT_DEVICES (sizeof(drivers)/sizeof(drivers[0]))
  const char* driver_names[CNT_DEVICES] = { "Hub1", "Hub2", "JOY1D", "Bluet", "HID1" , "HID2", "HID3" };
  
  bool driver_active[CNT_DEVICES] = { false, false, false, false };
#else
  BluetoothController bluet(myusb);   // version assumes it already was paired

  USBDriver* drivers[] = { &hub1, &hub2, &joystick1, &hid1, &hid2, &hid3 };
  
  #define CNT_DEVICES (sizeof(drivers)/sizeof(drivers[0]))
  const char* driver_names[CNT_DEVICES] = { "Hub1", "Hub2", "JOY1D", "HID1" , "HID2", "HID3" };
  
  bool driver_active[CNT_DEVICES] = { false, false, false };
#endif

int user_axis[64];
uint32_t g_buttons_prev = 0;
uint32_t g_buttons;
bool first_joystick_message = true;
uint8_t last_bdaddr[6] = { 0, 0, 0, 0, 0, 0 };




// Lets also look at HID Input devices
USBHIDInput* hiddrivers[] = { &joystick1 };

#define CNT_HIDDEVICES (sizeof(hiddrivers)/sizeof(hiddrivers[0]))
const char* hid_driver_names[CNT_DEVICES] = { "Joystick1" };

bool hid_driver_active[CNT_DEVICES] = { false };
#if defined(BLUETOOTH)
	BTHIDInput* bthiddrivers[] = { &joystick1 };
	#define CNT_BTHIDDEVICES (sizeof(bthiddrivers)/sizeof(bthiddrivers[0]))
	const char* bthid_driver_names[CNT_HIDDEVICES] = { "joystick" };
	bool bthid_driver_active[CNT_HIDDEVICES] = { false };
#endif

unsigned long g_ulLastMsgTime;
short  g_sGPSMController;    // What GPSM value have we calculated. 0xff - Not used yet
boolean g_fDynamicLegXZLength = false;  // Has the user dynamically adjusted the Leg XZ init pos (width)
boolean g_fDebugJoystick = false;
#define USBJoystickInputController InputController
// Define an instance of the Input Controller...
InputController  g_InputController;       // Our Input controller 



static short   g_BodyYOffset;
static short   g_BodyYShift;
static byte    ControlMode;
static byte    HeightSpeedMode;
//static bool  DoubleHeightOn;
static bool    DoubleTravelOn;
static byte    bJoystickWalkMode;
byte           GPSeq;             //Number of the sequence


// some external or forward function references.
extern void JoystickTurnRobotOff(void);
extern void UpdateActiveDeviceInfo();

//==============================================================================
// This is The function that is called by the Main program to initialize
//the input controller, which in this case is the PS2 controller
//process any commands.
//==============================================================================

// If both PS2 and XBee are defined then we will become secondary to the xbee
void USBJoystickInputController::Init(void)
{
	g_BodyYOffset = 0;
	g_BodyYShift = 0;
#ifdef DBGSerial  
	DBGSerial.print("USB Joystick Init: ");
#endif  
#ifdef USBHOST_DEGUG_MEMORY_STREAM
#ifdef ARDUINO_TEENSY41
	if (external_psram_size == 0) {
		uint8_t* debug_buffer = (uint8_t*)malloc(256 * 1024);
		if (debug_buffer) {
			Serial.println("\n*** USBHost Debug data - No external PSRAM using DMAMEM ***");
			USBHostDebugStream.setBuffer(debug_buffer, 256 * 1024);
		}
		else {
			Serial.println("\n*** USBHost Debug data - No external PSRAM malloc failed ***");
			USBHostDebugStream.setBuffer(debug_buffer, 0);
			USBHostDebugStream.enable(false);

		}
	}
#endif
#endif

	myusb.begin();
	GPSeq = 0;  // init to something...

	ControlMode = WALKMODE;
	HeightSpeedMode = NORM_NORM;
	//    DoubleHeightOn = false;
	DoubleTravelOn = false;
	bJoystickWalkMode = 0;
}

//==============================================================================
// This function is called by the main code to tell us when it is about to
// do a lot of bit-bang outputs and it would like us to minimize any interrupts
// that we do while it is active...
//==============================================================================
void USBJoystickInputController::AllowControllerInterrupts(boolean fAllow __attribute__((unused)))
{
	// We don't need to do anything...
}

//==============================================================================
// This is The main code to input function to read inputs from the Commander and then
//process any commands.
//==============================================================================
void USBJoystickInputController::ControlInput(void)
{
	// Make sure USB gets chance to process stuff. 
	myusb.Task();
	// check to see if the device list has changed:
	UpdateActiveDeviceInfo();
	//  processPS3MotionTimer();  - not sure yet support this yet. 

	if (joystick1.available()) {
		if (first_joystick_message) {
			Serial.printf("*** First Joystick message %x:%x ***\n",
				joystick1.idVendor(), joystick1.idProduct());
			first_joystick_message = false;

			const uint8_t* psz = joystick1.manufacturer();
			if (psz && *psz) Serial.printf("  manufacturer: %s\n", psz);
			psz = joystick1.product();
			if (psz && *psz) Serial.printf("  product: %s\n", psz);
			psz = joystick1.serialNumber();
			if (psz && *psz) Serial.printf("  Serial: %s\n", psz);

			// lets try to reduce number of fields that update
			joystick1.axisChangeNotifyMask(0xFFFFFl);
		}

		// If we receive a valid message than turn robot on...
		boolean fAdjustLegPositions = false;
		short sLegInitXZAdjust = 0;
		short sLegInitAngleAdjust = 0;

#if 0
		if (!g_InControlState.fRobotOn) {
			g_InControlState.fRobotOn = true;
			fAdjustLegPositions = true;
		}
#endif
		// [SWITCH MODES]
		g_buttons = joystick1.getButtons();
		if (joystick1.joystickType() == JoystickController::PS4) {
#if defined(BLUETOOTH)
			int hat = joystick1.getAxis(10);  // get hat - up/dwn buttons
#else
			int hat = joystick1.getAxis(9);  // get hat - up/dwn buttons
#endif
			if ((hat >= 0) && (hat < 8)) g_buttons |= PS4_MAP_HAT_MAP[hat];
			BTN_MASKS = PS4_BTNS;	// should have been set earlier, but just in case...
		}
		else {
			BTN_MASKS = PS3_BTNS;
		}

#if defined(BLUETOOTH)
		if ((g_buttons & BTN_MASKS[BUT_PS3]) && !(g_buttons_prev & BTN_MASKS[BUT_PS3])) {
			if ((joystick1.joystickType() == JoystickController::PS3) &&
				(g_buttons & (BTN_MASKS[BUT_L1] | BTN_MASKS[BUT_R1]))) {
				// PS button just pressed and select button pressed act like PS4 share like...
				// Note: you can use either R1 or L1 with the PS button, to work with Sony Move Navigation...
				Serial.print("\nPS3 Pairing Request");
				if (!last_bdaddr[0] && !last_bdaddr[1] && !last_bdaddr[2] && !last_bdaddr[3] && !last_bdaddr[4] && !last_bdaddr[5]) {
					Serial.println(" - failed - no Bluetooth adapter has been plugged in");
				}
				else if (!hiddrivers[0]) {  // Kludge see if we are connected as HID?
					Serial.println(" - failed - PS3 device not plugged into USB");
				}
				else {
					Serial.printf(" - Attempt pair to: %x:%x:%x:%x:%x:%x\n", last_bdaddr[0], last_bdaddr[1], last_bdaddr[2], last_bdaddr[3], last_bdaddr[4], last_bdaddr[5]);

					if (!joystick1.PS3Pair(last_bdaddr)) {
						Serial.println("  Pairing call Failed");
					}
					else {
						Serial.println("  Pairing complete (I hope), make sure Bluetooth adapter is plugged in and try PS3 without USB");
					}
				}
			} else {
				// Maybe lets toggle the 
				if (!g_InControlState.fRobotOn) {
					g_InControlState.fRobotOn = true;
					fAdjustLegPositions = true;
					Serial.println("Robot Power On.....");
				}
				else {
					JoystickTurnRobotOff();
					Serial.println("Robot Power Off.....");
				}
			}
		}
#else
		if ((g_buttons & BTN_MASKS[BUT_PS3]) && !(g_buttons_prev & BTN_MASKS[BUT_PS3])) {
				if (!g_InControlState.fRobotOn) {
					g_InControlState.fRobotOn = true;
					fAdjustLegPositions = true;
					Serial.println("Robot Power On.....");
				}
				else {
					JoystickTurnRobotOff();
					Serial.println("Robot Power Off.....");
				}
		}
#endif

		// Cycle through modes...
		if ((g_buttons & BTN_MASKS[BUT_TRI]) && !(g_buttons_prev & BTN_MASKS[BUT_TRI])) {
			if (++ControlMode >= MODECNT) {
				ControlMode = WALKMODE;    // cycled back around...
				MSound(2, 50, 2000, 50, 3000);
			}
			else {
				MSound(1, 50, 2000);
			}
#ifdef OPT_SINGLELEG      
			if (ControlMode != SINGLELEGMODE)
				g_InControlState.SelectedLeg = 255;
			else {
				g_InControlState.SelectedLeg = 0;   // Select leg 0 when we go into this mode. 
				g_InControlState.PrevSelectedLeg = 255;
#ifdef DEBUG_SINGLELEG
				Serial.println("Single Leg Mode .....");
#endif
			}
#endif
			// If we have display display new mode
#ifdef USE_ST7789
			tft.setCursor(0, TFT_Y_MODE);
			tft.setTextSize(2);
			tft.setTextColor(ST77XX_WHITE, ST77XX_RED);
			tft.print(mode_names[ControlMode]);
			tft.fillRect(tft.getCursorX(), tft.getCursorY(), tft.width(), 15, ST77XX_RED);
#endif
			Serial.println(mode_names[ControlMode]);

		}

		//[Common functions]
		//Switch Balance mode on/off 
		if ((g_buttons & BTN_MASKS[BUT_SQ]) && !(g_buttons_prev & BTN_MASKS[BUT_SQ])) {
			g_InControlState.BalanceMode = !g_InControlState.BalanceMode;
			if (g_InControlState.BalanceMode) {
				MSound(1, 250, 1500);
				Serial.println("Balance Mode: On ........");
			}
			else {
				MSound(2, 100, 2000, 50, 4000);
				Serial.println("Balance Mode: Off ........");
			}

		}

		//Stand up, sit down  
		if ((g_buttons & BTN_MASKS[BUT_HAT_DOWN]) && !(g_buttons_prev & BTN_MASKS[BUT_HAT_DOWN])) {
			g_BodyYOffset = 0;
			Serial.println("Sitting ........");
		}
		if ((g_buttons & BTN_MASKS[BUT_HAT_UP]) && !(g_buttons_prev & BTN_MASKS[BUT_HAT_UP])) {
			if (g_BodyYOffset < 35) g_BodyYOffset = 35;
			else g_BodyYOffset += 10;
			if (g_BodyYOffset > MAX_BODY_Y) g_BodyYOffset = MAX_BODY_Y;
			fAdjustLegPositions = true;
			g_fDynamicLegXZLength = false;
			Serial.println("Ready for Action ........");
		}

		// We will use L1 with the Right joystick to control both body offset as well as Speed...
		// We move each pass through this by a percentage of how far we are from center in each direction
		// We get feedback with height by seeing the robot move up and down.  For Speed, I put in sounds
		// which give an idea, but only for those whoes robot has a speaker
		int lx = joystick1.getAxis(AXIS_LX) - 127;
		int ly = joystick1.getAxis(AXIS_LY) - 127;
		int rx = joystick1.getAxis(AXIS_RX) - 127;
		int ry = joystick1.getAxis(AXIS_RY) - 127;
		if (g_fDebugJoystick) {
			Serial.printf("(%d)BTNS: %x LX: %d, LY: %d, RX: %d, RY: %d LT: %d RT: %d\r\n", joystick1.joystickType(), g_buttons,
				lx, ly, rx, ry, joystick1.getAxis(AXIS_LT), joystick1.getAxis(AXIS_RT));
		}

		if (g_buttons & BTN_MASKS[BUT_L1] && !(g_buttons_prev & BTN_MASKS[BUT_L1])) {
			Serial.println("Adjusting up/down (Ry) /speed (Rx) ........");
			// raise or lower the robot on the joystick up /down
			// Maybe should have Min/Max
			int delta = ry / 25;
			if (delta) {
				g_BodyYOffset = max(min(g_BodyYOffset + delta, MAX_BODY_Y), 0);
				fAdjustLegPositions = true;
			}

			// Also use right Horizontal to manually adjust the initial leg positions.
			sLegInitXZAdjust = lx / 10;        // play with this.
			sLegInitAngleAdjust = ly / 8;
			lx = 0;
			ly = 0;

			// Likewise for Speed control
			delta = rx / 16;   // 
			if ((delta < 0) && g_InControlState.SpeedControl) {
				if ((word)(-delta) < g_InControlState.SpeedControl)
					g_InControlState.SpeedControl += delta;
				else
					g_InControlState.SpeedControl = 0;
				MSound(1, 50, 1000 + g_InControlState.SpeedControl);
			}
			if ((delta > 0) && (g_InControlState.SpeedControl < 2000)) {
				g_InControlState.SpeedControl += delta;
				if (g_InControlState.SpeedControl > 2000)
					g_InControlState.SpeedControl = 2000;
				MSound(1, 50, 1000 + g_InControlState.SpeedControl);
			}

			rx = 0; // don't walk when adjusting the speed here...
		}

#ifdef DBGSerial
		if ((g_buttons & BTN_MASKS[BUT_X]) && !(g_buttons_prev & BTN_MASKS[BUT_X])) {
			MSound(1, 50, 2000);
			g_fDebugJoystick = !g_fDebugJoystick;
		}
		if ((g_buttons & BTN_MASKS[BUT_L3]) && !(g_buttons_prev & BTN_MASKS[BUT_L3])) {
			MSound(1, 50, 2000);
#if defined(TEENSY_DEBUG_H)
			if (debug.isGDBConnected()) {
				DBGSerial.println(F("Trying to break in to GDB"));
				// lets try it through setting breakpoint at loop
				debug.setBreakpoint((void*)&loop);
				//halt();
			}
			else
#endif
			{
				g_fDebugOutput = !g_fDebugOutput;
				if (g_fDebugOutput) DBGSerial.println(F("Debug is on"));
				else DBGSerial.println(F("Debug is off"));
			}

		}
#endif    
		//[Walk functions]
		if (ControlMode == WALKMODE) {
			//Switch Gaits
			if (((g_buttons & BTN_MASKS[BUT_SELECT]) && !(g_buttons_prev & BTN_MASKS[BUT_SELECT]))
				&& abs(g_InControlState.TravelLength.x) < cTravelDeadZone //No movement
				&& abs(g_InControlState.TravelLength.z) < cTravelDeadZone
				&& abs(g_InControlState.TravelLength.y * 2) < cTravelDeadZone) {
				g_InControlState.GaitType = g_InControlState.GaitType + 1;                    // Go to the next gait...
				if (g_InControlState.GaitType < NUM_GAITS) {                 // Make sure we did not exceed number of gaits...
					MSound(1, 50, 2000);
				}
				else {
					MSound(2, 50, 2000, 50, 2250);
					g_InControlState.GaitType = 0;
				}
				GaitSelect();
			}

			//Double leg lift height
			if ((g_buttons & BTN_MASKS[BUT_R2]) && !(g_buttons_prev & BTN_MASKS[BUT_R2])) {
				MSound(1, 50, 2000);
				HeightSpeedMode = (HeightSpeedMode + 1) & 0x3; // wrap around mode
				DoubleTravelOn = HeightSpeedMode & 0x1;
				if (HeightSpeedMode & 0x2) {
					g_InControlState.LegLiftHeight = 80;
					Serial.println("Double Leg Height Selected .....");
				} else {
					g_InControlState.LegLiftHeight = 50;
					Serial.println("Normal Leg Height Selected .....");
				}
			}

			// Switch between Walk method 1 && Walk method 2
			if ((g_buttons & BTN_MASKS[BUT_CIRC]) && !(g_buttons_prev & BTN_MASKS[BUT_CIRC])) {
#ifdef cTurretRotPin
				if ((++bJoystickWalkMode) > 2)
#else
				if ((++bJoystickWalkMode) > 1)
#endif 
					bJoystickWalkMode = 0;
				MSound(1, 50, 2000 + bJoystickWalkMode * 250);
				Serial.printf("Walkmethod %d Selected ........\n", bJoystickWalkMode );
			}

			//Walking
			switch (bJoystickWalkMode) {
			case 0:
				g_InControlState.TravelLength.x = -lx;
				g_InControlState.TravelLength.z = -ly;
				g_InControlState.TravelLength.y = -(rx) / 4; //Right Stick Left/Right 
				break;
			case 1:
				g_InControlState.TravelLength.z = (joystick1.getAxis(AXIS_RY)); //Right Stick Up/Down  
				g_InControlState.TravelLength.y = -(rx) / 4; //Right Stick Left/Right 
				break;
#ifdef cTurretRotPin
			case 2:
				g_InControlState.TravelLength.x = -lx;
				g_InControlState.TravelLength.z = -ly;

				// Will use Right now stick to control turret.
				g_InControlState.TurretRotAngle1 = max(min(g_InControlState.TurretRotAngle1 + rx / 5, cTurretRotMax1), cTurretRotMin1);      // Rotation of turret in 10ths of degree
				g_InControlState.TurretTiltAngle1 = max(min(g_InControlState.TurretTiltAngle1 + joystick1.getAxis(AXIS_RY) / 5, cTurretTiltMax1), cTurretTiltMin1);  // tilt of turret in 10ths of degree
#endif
			}

			if (!DoubleTravelOn) {  //(Double travel length)
				g_InControlState.TravelLength.x = g_InControlState.TravelLength.x / 2;
				g_InControlState.TravelLength.z = g_InControlState.TravelLength.z / 2;
			}

		}

		//[Translate functions]
		g_BodyYShift = 0;
		if (ControlMode == TRANSLATEMODE) {
			g_InControlState.BodyPos.x = SmoothControl(((lx) * 2 / 3), g_InControlState.BodyPos.x, SmDiv);
			g_InControlState.BodyPos.z = SmoothControl(((ly) * 2 / 3), g_InControlState.BodyPos.z, SmDiv);
			g_InControlState.BodyRot1.y = SmoothControl(((rx) * 2), g_InControlState.BodyRot1.y, SmDiv);

			//      g_InControlState.BodyPos.x = (lx)/2;
			//      g_InControlState.BodyPos.z = -(ly)/3;
			//      g_InControlState.BodyRot1.y = (rx)*2;
			g_BodyYShift = (-(joystick1.getAxis(AXIS_RY)) / 2);
		}

		//[Rotate functions]
		if (ControlMode == ROTATEMODE) {
			g_InControlState.BodyRot1.x = (ly);
			g_InControlState.BodyRot1.y = (rx) * 2;
			g_InControlState.BodyRot1.z = (lx);
			g_BodyYShift = (-(joystick1.getAxis(AXIS_RY)) / 2);
		}
		//[Single leg functions]
#ifdef OPT_SINGLELEG      
		if (ControlMode == SINGLELEGMODE) {
			//Switch leg for single leg control
			if ((g_buttons & BTN_MASKS[BUT_SELECT]) && !(g_buttons_prev & BTN_MASKS[BUT_SELECT])) {
				MSound(1, 50, 2000);
				if (g_InControlState.SelectedLeg < (CNT_LEGS - 1)) {
					g_InControlState.SelectedLeg = g_InControlState.SelectedLeg + 1;
					Serial.printf("Leg #%d Selected .....\n", g_InControlState.SelectedLeg);
				} else {
					g_InControlState.SelectedLeg = 0;
				}
			}

#if 0
			g_InControlState.SLLeg.x = (signed char)((int)((int)lx + 128) / 2); //Left Stick Right/Left
			g_InControlState.SLLeg.y = (signed char)((int)((int)joystick1.getAxis(AXIS_RY) + 128) / 10); //Right Stick Up/Down
			g_InControlState.SLLeg.z = (signed char)((int)((int)ly + 128) / 2); //Left Stick Up/Down
#else
			// BUGBUG:: Need to figure out a decent range for these values... 
			g_InControlState.SLLeg.x = lx; //Left Stick Right/Left
			g_InControlState.SLLeg.y = joystick1.getAxis(AXIS_RY) / 3 - 20; //Right Stick Up/Down
			g_InControlState.SLLeg.z = ly; //Left Stick Up/Down
#endif
#ifdef DEBUG_SINGLELEG
			Serial.print(g_InControlState.SLLeg.x, DEC);
			Serial.print(",");
			Serial.print(g_InControlState.SLLeg.y, DEC);
			Serial.print(",");
			Serial.println(g_InControlState.SLLeg.z, DEC);
#endif
			// Hold single leg in place
			if ((g_buttons & BTN_MASKS[BUT_R1]) && !(g_buttons_prev & BTN_MASKS[BUT_R1])) {
				MSound(1, 50, 2000);
				g_InControlState.fSLHold = !g_InControlState.fSLHold;
				Serial.println("Holding selected leg in position ...........");
			}
		}
#endif

		//Calculate walking time delay
		g_InControlState.InputTimeDelay = 128 - max(max(abs(lx), abs(ly)), abs(rx));

		//Calculate g_InControlState.BodyPos.y
		g_InControlState.BodyPos.y = max(g_BodyYOffset + g_BodyYShift, 0);

		if (sLegInitXZAdjust || sLegInitAngleAdjust) {
			// User asked for manual leg adjustment - only do when we have finished any previous adjustment

			if (!g_InControlState.ForceGaitStepCnt) {
				if (sLegInitXZAdjust)
					g_fDynamicLegXZLength = true;

				sLegInitXZAdjust += GetLegsXZLength();  // Add on current length to our adjustment...
				// Handle maybe change angles...
				if (sLegInitAngleAdjust)
					RotateLegInitAngles(sLegInitAngleAdjust);

				// Give system time to process previous calls
				AdjustLegPositions(sLegInitXZAdjust);
			}
		}

		if (fAdjustLegPositions && !g_fDynamicLegXZLength)
			AdjustLegPositionsToBodyHeight();    // Put main workings into main program file

		  // Save away the buttons state as to not process the same press twice.
		g_buttons_prev = g_buttons;
		g_ulLastMsgTime = millis();
	}
	else {
		// We did not receive a valid packet.  check for a timeout to see if we should turn robot off...
		if (g_InControlState.fRobotOn) {
			if (!joystick1)
				JoystickTurnRobotOff();
		}
	}
}

//==============================================================================
// JoystickTurnRobotOff - code used couple of places so save a little room...
//==============================================================================
void JoystickTurnRobotOff(void)
{
	//Turn off
	g_InControlState.BodyPos.x = 0;
	g_InControlState.BodyPos.y = 0;
	g_InControlState.BodyPos.z = 0;
	g_InControlState.BodyRot1.x = 0;
	g_InControlState.BodyRot1.y = 0;
	g_InControlState.BodyRot1.z = 0;
	g_InControlState.TravelLength.x = 0;
	g_InControlState.TravelLength.z = 0;
	g_InControlState.TravelLength.y = 0;
	g_BodyYOffset = 0;
	g_BodyYShift = 0;
#ifdef OPT_SINGLELEG      
	g_InControlState.SelectedLeg = 255;
#endif
	g_InControlState.fRobotOn = 0;

#ifdef cTurretRotPin
	g_InControlState.TurretRotAngle1 = cTurretRotInit;      // Rotation of turrent in 10ths of degree
	g_InControlState.TurretTiltAngle1 = cTurretTiltInit;    // the tile for the turret
#endif

	g_fDynamicLegXZLength = false; // also make sure the robot is back in normal leg init mode...
}
//================================================================================
#ifdef OPT_TERMINAL_MONITOR_IC
// Optional stuff to allow me to have Input device debug support
//==============================================================================
// ShowTerminalCommandList: Allow the Terminal monitor to call the servo driver
//      to allow it to display any additional commands it may have.
//==============================================================================
void USBJoystickInputController::ShowTerminalCommandList(void)
{
	DBGSerial.println(F("J - Show Joystick data"));
#ifdef USBHOST_DEGUG_MEMORY_STREAM
	DBGSerial.println(F("U <DCFL> USB Host Dump, Clear, First, Last"));
#endif
}

boolean USBJoystickInputController::ProcessTerminalCommand(byte* psz, byte bLen)
{
	if ((bLen == 1) && ((*psz == 'j') || (*psz == 'J'))) {
		g_fDebugJoystick = !g_fDebugJoystick;
		if (g_fDebugJoystick) DBGSerial.println("\n*** Joystick Debug ON ***");
		else DBGSerial.println("\n*** Joystick Debug OFF ***");
		return true;
	}
#ifdef USBHOST_DEGUG_MEMORY_STREAM
	else if ((*psz == 'u') || (*psz == 'U')) {
		psz++;

		if ((*psz == 'c') || (*psz == 'C')) {
			DBGSerial.println("\n*** Clear USB Host Debug data ***");
				USBHostDebugStream.clear();
		}
		else if ((*psz == 'f') || (*psz == 'F')) {
			DBGSerial.println("\n*** Save first received USBHost debug bytes ***");
			USBHostDebugStream.stopWritesOnOverflow(true);
		}
		else if ((*psz == 'l') || (*psz == 'L')) {
			DBGSerial.println("\n*** Save last received USBHost debug bytes ***");
			USBHostDebugStream.stopWritesOnOverflow(false);
		}
		else {
			DBGSerial.println("\n*** dump the USBHost Debug data ***");
			int ch;
			while ((ch = USBHostDebugStream.read()) != -1) DBGSerial.write(ch);
		}
		return true;
	}
#endif

	return false;

}
#endif

//=============================================================================
// UpdateActiveDeviceInfo
//=============================================================================
void UpdateActiveDeviceInfo() {
	for (uint8_t i = 0; i < CNT_DEVICES; i++) {
		if (*drivers[i] != driver_active[i]) {
			if (driver_active[i]) {
				Serial.printf("*** Device %s - disconnected ***\n", driver_names[i]);
				driver_active[i] = false;
			}
			else {
				Serial.printf("*** Device %s %x:%x - connected ***\n", driver_names[i], drivers[i]->idVendor(), drivers[i]->idProduct());
				driver_active[i] = true;

				const uint8_t* psz = drivers[i]->manufacturer();
				if (psz && *psz) DBGSerial.printf("  manufacturer: %s\n", psz);
				psz = drivers[i]->product();
				if (psz && *psz) DBGSerial.printf("  product: %s\n", psz);
				psz = drivers[i]->serialNumber();
				if (psz && *psz) DBGSerial.printf("  Serial: %s\n", psz);

				if (drivers[i] == &bluet) {
					const uint8_t* bdaddr = bluet.myBDAddr();
					// remember it...
					DBGSerial.printf("  BDADDR: %x:%x:%x:%x:%x:%x\n", bdaddr[0], bdaddr[1], bdaddr[2], bdaddr[3], bdaddr[4], bdaddr[5]);
					for (uint8_t i = 0; i < 6; i++) last_bdaddr[i] = bdaddr[i];
				}
			}
		}
	}

	for (uint8_t i = 0; i < CNT_HIDDEVICES; i++) {
		if (*hiddrivers[i] != hid_driver_active[i]) {
			if (hid_driver_active[i]) {
				DBGSerial.printf("*** HID Device %s - disconnected ***\n", hid_driver_names[i]);
				hid_driver_active[i] = false;
			}
			else {
				DBGSerial.printf("*** HID Device %s %x:%x - connected ***\n", hid_driver_names[i], hiddrivers[i]->idVendor(), hiddrivers[i]->idProduct());
				hid_driver_active[i] = true;

				const uint8_t* psz = hiddrivers[i]->manufacturer();
				if (psz && *psz) DBGSerial.printf("  manufacturer: %s\n", psz);
				psz = hiddrivers[i]->product();
				if (psz && *psz) DBGSerial.printf("  product: %s\n", psz);
				psz = hiddrivers[i]->serialNumber();
				if (psz && *psz) DBGSerial.printf("  Serial: %s\n", psz);

				// See if this is our joystick object...
				if (hiddrivers[i] == &joystick1) {
					DBGSerial.printf("  Joystick type: %d\n", joystick1.joystickType());
					switch (joystick1.joystickType()) {
					case JoystickController::PS4:
						BTN_MASKS = PS4_BTNS;
						break;
					default:
					case JoystickController::PS3:
						BTN_MASKS = PS3_BTNS;
						break;
					}

#ifdef LATER
					if (joystick1.joystickType() == JoystickController::PS3_MOTION) {
						DBGSerial.println("  PS3 Motion detected");
						PS3_MOTION_timer = millis();  // set time for last event
						PS3_MOTION_tried_to_pair_state = 0;
					}
#endif
				}

			}
		}
	}
	
#if defined(BLUETOOTH)
	// Then Bluetooth devices
	for (uint8_t i = 0; i < CNT_BTHIDDEVICES; i++) {
		if (*bthiddrivers[i] != bthid_driver_active[i]) {
			if (bthid_driver_active[i]) {
				DBGSerial.printf("*** BTHID Device %s - disconnected ***\n", hid_driver_names[i]);
				bthid_driver_active[i] = false;
			}
			else {
				DBGSerial.printf("*** BTHID Device %s %x:%x - connected ***\n", hid_driver_names[i], hiddrivers[i]->idVendor(), hiddrivers[i]->idProduct());
				bthid_driver_active[i] = true;

				const uint8_t* psz = bthiddrivers[i]->manufacturer();
				if (psz && *psz) Serial.printf("  manufacturer: %s\n", psz);
				psz = bthiddrivers[i]->product();
				if (psz && *psz) Serial.printf("  product: %s\n", psz);
				psz = bthiddrivers[i]->serialNumber();
				if (psz && *psz) Serial.printf("  Serial: %s\n", psz);
				// See if this is our joystick object...
				if (hiddrivers[i] == &joystick1) {
					DBGSerial.printf("  Joystick type: %d\n", joystick1.joystickType());
					switch (joystick1.joystickType()) {
					case JoystickController::PS4:
						BTN_MASKS = PS4_BTNS;
						break;
					default:
					case JoystickController::PS3:
						BTN_MASKS = PS3_BTNS;
						break;
					}
				}
			}
		}
	}
#endif
	
}
