//====================================================================
//Project Lynxmotion Phoenix
//Description: 
//    This is the hardware configuration file for the 
//    Will first define to use their commander unit.
//
//Date: June 19, 2013
//Programmer: Kurt (aka KurtE)
//
//NEW IN V1.0
//   - First Release - Changes from Trossen member KingPin
//
//====================================================================
#ifndef HEX_CFG_H
#define HEX_CFG_H
//==================================================================================================================================
// Define which input classes we will use. If we wish to use more than one we need to define USEMULTI - This will define a forwarder
//    type implementation, that the Inputcontroller will need to call.  There will be some negotion for which one is in contol.
//
//  If this is not defined, The included Controller should simply implement the InputController Class...
//==================================================================================================================================
//#define USECOMMANDER
#define USEJOYSTICK		// Use T3.6 or T4.x USB Host control

#define LSS_SERIAL_PORT     Serial1
#define LSS_BAUD            250000
// Global defines to control which configuration we are using.  Note: Only define one of these...
// 
// Which type of control(s) do you want to compile in
#if defined(KINETISK) || defined(__IMXRT1062__)
#define DBGSerial         Serial
#else
#if defined(UBRR2H)
#define DBGSerial         Serial
#endif
#endif 

// Lets put in option to use ST7789
#define USE_ST7789
#define TFT_CS		10
#define TFT_DC		9
#define TFT_RST		28		// Reset
#define TFT_BL		29		// backlight
#define TFT_WIDTH	240
#define TFT_HEIGHT	240

#define TFT_Y_MODE	120
#define TFT_Y_GAIT	140

#ifdef USE_ST7789
#include <ST7735_t3.h>
#include <ST7789_t3.h>
extern ST7789_t3 tft;
#endif

#define DISPLAY_GAIT_NAMES

// Define other optional compnents to be included or not...
//#define cFemurHornOffset1 -35
//#define cTibiaHornOffset1 463
// Coxa may be reversed from default
#define cRRCoxaInv 0
#define cRRFemurInv 0
#define cRRTibiaInv 0

#define cRMCoxaInv 0
#define cRMFemurInv 0
#define cRMTibiaInv 0

#define cRFCoxaInv 0 
#define cRFFemurInv 0 
#define cRFTibiaInv 0

/* I think femur directions are same as default
#define cRRFemurInv 1 
#define cRFFemurInv 1 
#define cLRFemurInv 0 
#define cLMFemurInv 0 
#define cLFFemurInv 0 
*/

/* Likewise Tibia unlike PhantomX
#define cRRTibiaInv 1 
#define cRMTibiaInv 1 
#define cRFTibiaInv 1 
#define cLRTibiaInv 0 
#define cLMTibiaInv 0 
#define cLFTibiaInv 0 
*/

// These were from PhantomX
/*#define cRRTibiaInv 0 
#define cRMTibiaInv 0 
#define cRFTibiaInv 0 
#define cLRTibiaInv 1 
#define cLMTibiaInv 1 
#define cLFTibiaInv 1 
*/

//===================================================================
// Debug Options
#ifdef DBGSerial
#define OPT_TERMINAL_MONITOR  
#define OPT_TERMINAL_MONITOR_IC    // Allow the input controller to define stuff as well
#define OPT_FIND_SERVO_OFFSETS    // Only useful if terminal monitor is enabled
#endif

//#define DEBUG_IOPINS
#ifdef DEBUG_IOPINS
#define DebugToggle(pin)  {digitalWrite(pin, !digitalRead(pin));}
#define DebugWrite(pin, state) {digitalWrite(pin, state);}
#else
#define DebugToggle(pin)  {;}
#define DebugWrite(pin, state) {;}
#endif


// Also define that we are using the AX12 driver
#define USE_AX12_DRIVER
#define OPT_BACKGROUND_PROCESS    // The AX12 has a background process
#define OPT_CHECK_SERVO_RESET     // Try to find single servo that reset it's ID...
#define OPT_GPPLAYER

#define OPT_SINGLELEG      

#define DEFINE_HEX_GLOBALS
//==================================================================================================================================
//==================================================================================================================================
//==================================================================================================================================
//  PhantomX
//==================================================================================================================================
//[SERIAL CONNECTIONS]

//====================================================================
// XBEE on non mega???
#if defined(KINETISK) || defined(__IMXRT1062__)
#define XBeeSerial Serial2
#else
#if defined(UBRR2H)
#define XBeeSerial Serial2
#endif
#define XBeeSerial Serial
#endif
#define XBEE_BAUD        38400
//--------------------------------------------------------------------
//[Arbotix Pin Numbers]
#if defined(KINETISK) || defined(__IMXRT1062__)
#define SOUND_PIN    5
#else
#define SOUND_PIN    1 //0xff        // Tell system we have no IO pin...
#define USER 0                        // defaults to 13 but Arbotix on 0...
#endif

// Define Analog pin and minimum voltage that we will allow the servos to run
#if defined(KINETISK) ||  defined(__IMXRT1062__)
// Our Teensy board
#define cVoltagePin  0
#define CVADR1      402  // VD Resistor 1 - reduced as only need ratio... 40.2K and 10K
#define CVADR2      100    // VD Resistor 2
#define CVREF       330    // 3.3v
#endif
//#define cVoltagePin  7      // Use our Analog pin jumper here...
//#define CVADR1      1000  // VD Resistor 1 - reduced as only need ratio... 20K and 4.66K
//#define CVADR2      233   // VD Resistor 2
//#define cTurnOffVol  1000     // 10v
//#define cTurnOnVol   1100     // 11V - optional part to say if voltage goes back up, turn it back on...

//====================================================================
#define  DEFAULT_GAIT_SPEED 35  // Default gait speed  - Will depend on what Servos you are using...
#define  DEFAULT_SLOW_GAIT  50  // Had a couple different speeds...

//====================================================================
// Defines for Optional XBee Init and configuration code.
//====================================================================
#define CHECK_AND_CONFIG_XBEE
#define DEFAULT_MY 0x101  // Swap My/DL on 2nd unit
#define DEFAULT_DL 0x102
#define DEFAULT_ID 0x3332

//--------------------------------------------------------------------
// Define which pins(sevo IDS go with which joint

#define cRRCoxaPin      8   //Rear Right leg Hip Horizontal
#define cRRFemurPin     10   //Rear Right leg Hip Vertical
#define cRRTibiaPin     12   //Rear Right leg Knee

#define cRMCoxaPin      14  //Middle Right leg Hip Horizontal
#define cRMFemurPin     16  //Middle Right leg Hip Vertical
#define cRMTibiaPin     18  //Middle Right leg Knee

#define cRFCoxaPin      2  //Front Right leg Hip Horizontal
#define cRFFemurPin     4  //Front Right leg Hip Vertical
#define cRFTibiaPin     6   //Front Right leg Knee

#define cLRCoxaPin      7   //Rear Left leg Hip Horizontal
#define cLRFemurPin     9   //Rear Left leg Hip Vertical
#define cLRTibiaPin     11   //Rear Left leg Knee

#define cLMCoxaPin      13   //Middle Left leg Hip Horizontal
#define cLMFemurPin     15   //Middle Left leg Hip Vertical
#define cLMTibiaPin     17  //Middle Left leg Knee

#define cLFCoxaPin      1   //Front Left leg Hip Horizontal
#define cLFFemurPin     3   //Front Left leg Hip Vertical
#define cLFTibiaPin     5   //Front Left leg Knee

//#define cTurretRotPin  20   // Turret Rotate Pin
//#define cTurretTiltPin 21  // Turret Tilt pin


//--------------------------------------------------------------------
//[MIN/MAX ANGLES] - Start off assume same as Phoenix...
#define cXXTibiaMin1    -600
#define cXXTibiaMax1     750
#define cXXFemurMin		-900
#define cXXFemurMax		 900
#define cXXCoxaMin		-750
#define cXXCoxaMax		 750

#define cRRCoxaMin1			cXXCoxaMin
#define cRRCoxaMax1			cXXCoxaMax
#define cRRFemurMin1			cXXFemurMin
#define cRRFemurMax1		cXXFemurMax
#define cRRTibiaMin1			cXXTibiaMin1
#define cRRTibiaMax1			cXXTibiaMax1

#define cRMCoxaMin1			cXXCoxaMin    //Mechanical limits of the Right Middle Leg
#define cRMCoxaMax1			cXXCoxaMax
#define cRMFemurMin1			cXXFemurMin
#define cRMFemurMax1			cXXFemurMax
#define cRMTibiaMin1			cXXTibiaMin1
#define cRMTibiaMax1			cXXTibiaMax1

#define cRFCoxaMin1			cXXCoxaMin    //Mechanical limits of the Right Front Leg
#define cRFCoxaMax1			cXXCoxaMax
#define cRFFemurMin1			cXXFemurMin
#define cRFFemurMax1			cXXFemurMax
#define cRFTibiaMin1			cXXTibiaMin1
#define cRFTibiaMax1			cXXTibiaMax1

#define cLRCoxaMin1			cXXCoxaMin    //Mechanical limits of the Left Rear Leg
#define cLRCoxaMax1			cXXCoxaMax
#define cLRFemurMin1			cXXFemurMin
#define cLRFemurMax1			cXXFemurMax
#define cLRTibiaMin1			cXXTibiaMin1
#define cLRTibiaMax1			cXXTibiaMax1

#define cLMCoxaMin1			cXXCoxaMin    //Mechanical limits of the Left Middle Leg
#define cLMCoxaMax1			cXXCoxaMax
#define cLMFemurMin1			cXXFemurMin
#define cLMFemurMax1			cXXFemurMax
#define cLMTibiaMin1			cXXTibiaMin1
#define cLMTibiaMax1			cXXTibiaMax1

#define cLFCoxaMin1			cXXCoxaMin    //Mechanical limits of the Left Front Leg
#define cLFCoxaMax1			cXXCoxaMax
#define cLFFemurMin1			cXXFemurMin
#define cLFFemurMax1			cXXFemurMax
#define cLFTibiaMin1			cXXTibiaMin1
#define cLFTibiaMax1			cXXTibiaMax1

//--------------------------------------------------------------------
//[Joint offsets]
// This allows us to calibrate servos to some fixed position, and then adjust them by moving theim
// one or more servo horn clicks.  This requires us to adjust the value for those servos by 15 degrees
// per click.  This is used with the T-Hex 4DOF legs
//First calibrate the servos in the 0 deg position using the SSC-32 reg offsets, then:
//--------------------------------------------------------------------
//[LEG DIMENSIONS]
//Universal dimensions for each leg in mm
#define cXXCoxaLength     51    // 50.8 to be exact
#define cXXFemurLength    80    // 80.32mm to be exact
#define cXXTibiaLength    116  //116.24 from drawing


#define cRRCoxaLength     cXXCoxaLength	    //Right Rear leg
#define cRRFemurLength    cXXFemurLength
#define cRRTibiaLength    cXXTibiaLength


#define cRMCoxaLength     cXXCoxaLength	    //Right middle leg
#define cRMFemurLength    cXXFemurLength
#define cRMTibiaLength    cXXTibiaLength


#define cRFCoxaLength     cXXCoxaLength	    //Rigth front leg
#define cRFFemurLength    cXXFemurLength
#define cRFTibiaLength    cXXTibiaLength


#define cLRCoxaLength     cXXCoxaLength	    //Left Rear leg
#define cLRFemurLength    cXXFemurLength
#define cLRTibiaLength    cXXTibiaLength


#define cLMCoxaLength     cXXCoxaLength	    //Left middle leg
#define cLMFemurLength    cXXFemurLength
#define cLMTibiaLength    cXXTibiaLength


#define cLFCoxaLength     cXXCoxaLength	    //Left front leg
#define cLFFemurLength    cXXFemurLength
#define cLFTibiaLength    cXXTibiaLength



//--------------------------------------------------------------------
//[BODY DIMENSIONS]
#define cRRCoxaAngle1   -560   //Default Coxa setup angle, 34.280877 degs from center line? atan-1(60.6/88.9)
#define cRMCoxaAngle1    0      //Default Coxa setup angle
#define cRFCoxaAngle1    560      //Default Coxa setup angle
#define cLRCoxaAngle1    -560   //Default Coxa setup angle
#define cLMCoxaAngle1    0      //Default Coxa setup angle
#define cLFCoxaAngle1    560     //Default Coxa setup angle

#define X_COXA      89  // MM between front and back legs /2  ???????????
#define Y_COXA      89  // MM between front/back legs /2   ????????????????????????????
#define M_COXA      61  // MM between two middle legs /2   ????????????????????????????

#define cRROffsetX      -61    //Distance X from center of the body to the Right Rear coxa
#define cRROffsetZ       89     //Distance Z from center of the body to the Right Rear coxa

#define cRMOffsetX      -81     //Distance X from center of the body to the Right Middle coxa
#define cRMOffsetZ      0       //Distance Z from center of the body to the Right Middle coxa

#define cRFOffsetX      -61     //Distance X from center of the body to the Right Front coxa
#define cRFOffsetZ      -89    //Distance Z from center of the body to the Right Front coxa

#define cLROffsetX       61      //Distance X from center of the body to the Left Rear coxa
#define cLROffsetZ       89     //Distance Z from center of the body to the Left Rear coxa

#define cLMOffsetX       81    //Distance X from center of the body to the Left Middle coxa
#define cLMOffsetZ       0       //Distance Z from center of the body to the Left Middle coxa

#define cLFOffsetX       61      //Distance X from center of the body to the Left Front coxa
#define cLFOffsetZ      -89    //Distance Z from center of the body to the Left Front coxa
//--------------------------------------------------------------------
//[START POSITIONS FEET]
#define cHexInitXZ   131
#define CHexInitXZCos60  73    // COS(56) = .707
#define CHexInitXZSin60  109    // sin(56) = .707
#define CHexInitY  116 //30


// Lets try some multi leg positions depending on height settings.
#define CNT_HEX_INITS 2
#define MAX_BODY_Y  120
#ifdef DEFINE_HEX_GLOBALS
const byte g_abHexIntXZ[] PROGMEM = {cHexInitXZ, 144};
const byte g_abHexMaxBodyY[] PROGMEM = { 20, MAX_BODY_Y};
#else
extern const byte g_abHexIntXZ[] PROGMEM;
extern const byte g_abHexMaxBodyY[] PROGMEM;
#endif

#define cRRInitPosX     CHexInitXZCos60      //Start positions of the Right Rear leg
#define cRRInitPosY     CHexInitY
#define cRRInitPosZ     CHexInitXZSin60

#define cRMInitPosX     cHexInitXZ			//Start positions of the Right Middle leg
#define cRMInitPosY     CHexInitY
#define cRMInitPosZ     0

#define cRFInitPosX     CHexInitXZCos60      //Start positions of the Right Front leg
#define cRFInitPosY     CHexInitY
#define cRFInitPosZ     -CHexInitXZSin60

#define cLRInitPosX     CHexInitXZCos60      //Start positions of the Left Rear leg
#define cLRInitPosY     CHexInitY
#define cLRInitPosZ     CHexInitXZSin60

#define cLMInitPosX     cHexInitXZ      //Start positions of the Left Middle leg
#define cLMInitPosY     CHexInitY
#define cLMInitPosZ     0

#define cLFInitPosX     CHexInitXZCos60      //Start positions of the Left Front leg
#define cLFInitPosY     CHexInitY
#define cLFInitPosZ     -CHexInitXZSin60

// Turret initial position
#define cTurretRotInit  0
#define cTurretTiltInit 0


//--------------------------------------------------------------------
//[Tars factors used in formula to calc Tarsus angle relative to the ground]
#define cTarsst	720	//4DOF ONLY
#define cTarsMulti	2	//4DOF ONLY
#define cTarsFactorA	70	//4DOF ONLY
#define cTarsFactorB	60	//4DOF ONLY
#define cTarsFactorC	50	//4DOF ONLY

#endif // HEX_CFG_H
