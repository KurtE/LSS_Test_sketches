# LSS_Phoenix_BT

## Menu Commands

Command | Description
------------ | -------------
D | Toggle debug on or off
J | Show Joystick data
V | Voltage
M | Toggle Motors on or off
T | Test Servos
P | Servo Positions
A | Toggle LSS speed control
L | Toggle LSS Servo Debug output
F <FPS> | Set FPS for Interpolation mode
S | Track Servos
O | Enter Servo offset mode
OM| Enter Servo offset mode in manual mode
C | clear Servo Offsets
  
### Servo Offset Mode
  
The servo offset mode, allows you to properly setup the 
servo Offsets as well as the Gyre (rotation direction)

It will initially print out current settings as well as
several other servo settings.  It currently then outputs:
```
The Goal is to align the top two servo pivots (Coxa and Femur) to be parallel to ground
And the Tibia should be at a right angle to the ground

  Enter $-Exit, +- changes, *-change servo
    0-n Chooses a leg, C-Coxa, F-Femur, T-Tibia
    m - manually move mode to get close
```
  
Manual mode was added to hopefully make it easier for you to setup the offsets 
regardless of how you aligned your servos during assembly.  You should set the
hexapod up on some form of stand, and enter this mode, and rougly try to set the
legs like mentioned above (Parallel and right angles).  Once you are close, enter
any keys to get back to normal align mode. 

In normal mode, when a new servo is selected, it will wiggle that servo and turn
the led to green, you can use the + and - keyboard keys to adjust a little. If
you adjust a servo I turn the LED red.  

You can select different Servos, by cycle through them with either the * key or
just <LF> without anything else on line.  You can also select a leg to move to
by typing in 0-5.  You can also jump to a specific servo on a leg with the C,F or T
keys.

Once you are done making changes, type a $ to exit.  At this point you have the option
to save or not save... I also added an ability to choose servo by servo...
If you save anything, I would probably suggest that you completely power it off and
back on, including rebooting... And hopefully the new updated offsets will work.
  
#### Joystick Commands

Command | Description
------------ | -------------
PS | Power On/Off
Cross | Joystick Debug
Up Arrow | Stand/Start Position
Dwn Arrow | Sit Position
Left Arrow | Slow Down Gait
Right Arrow | Speed Up Gait
Squate | Balance Mode On/Off
Triangle | Motion Select
... | Translate
... | Rotate
... | Single
... | Walk
L1 | Leg Controls
... | Ry - Up/Dwn
... | Rx - Speed Up/Dwn
--- | Lx/Ly  manually adjust the initial leg positions
R1 | Hold Single leg in place when Single leg selected

### Available Commands in Certain Motions Selected
  ### Single Leg Motion 
Command | Description
------------ | -------------
Select | Left or right front leg
  
  ### Walk Motion
Command | Description
------------ | -------------
Options | Gait Select
... | Tripod 8
... | Tripple 12
... | Tripple 16
... | Wave 24
... | Tripod 6
... |  Ripple 12
R2 | Double Leg Lift
Circle | Walkmode 1 (Auto)
... | Circle | Walkmode 0 (Manual Control)

  ### Rotate Motion
Command | Description
------------ | -------------
X axis | Use Ly stick
Y axis | Use Rx stick
Z axis | Use Lx stick
  
  ### Translate Motion
Command | Description
------------ | -------------
X axis | Use Lx stick
Y axis | Use Rx stick
Z axis | Use Ly stick

  
### Joystick Bluetooth pairing (PS3 and PS4)

For information on which Bluetooth dongles and joysticks that we have tried and 
hopefully working, see the PJRC Forum thread: https://forum.pjrc.com/threads/49358-T3-6-USB-Host-Bluetooth

To Pair a PS4 to a bluetooth dongle in a relatively easier manor. 

    Edit the LSS_Phoenix_Input_USB_Joystick.cpp to uncomment the line that says pairing and comment out the next line - that line is used when you are already paired.

```
  #if defined(BLUETOOTH)
    //BluetoothController bluet(myusb, true, "0000");   // Version does pairing to device
    BluetoothController bluet(myusb);   // version assumes it already was paired
```

Once you boot up with this version of the code:
Hold down the Share button and PS button at the same time, until the PS4, starts a sort of double blink… Hopefully then it pairs. Then assuming it paired, you change the two lines back and rebuild again.

With the BLUETOOTH option, it was needed before on PS4 as a little of the data is different when connected by plugged in versus by BT. In particular the HAT buttons. Some of the current code with BT enabled, tries to deduce this. It assumes the PS4 is BT if the BT object is active otherwise it assumes wired.

Note: I have also cheated in the past and simply paired a PS4 with a BT dongle on my Linux boot… Could probably do as well with windows, but my windows machine has a BT built in…

With PS3 it is even easier… You can use the normal BT build… Then you need to plug in the BT dongle and make sure it starts up, and either remove it or if using hub, plug in the PS3 wired… Hopefully code will have remembered the BT address. Once the joystick has started up wired, I believe you hold down either the L1 or R1 buttons and the PS3 button and it will tell the PS3 to update itself for that BT object… After that simply unplug it, plug back in the BT dongle and press the PS3 button to connect up…
  
For list of known working dongles post #1 (https://forum.pjrc.com/threads/49358-T3-6-USB-Host-Bluetooth) in USBHost_t36 PJRC Forum thread: 

