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
C | clear Servo Offsets
  
### Servo Offset Mode
  
The servo offset mode, allows you to properly setup the 
servo Offsets as well as the Gyre (rotation direction)

It will initially print out current settings as well as
several other servo settings.  It currently then outputs:
The Goal is to align the top two servo pivots (Coxa and Femur) to be parallel to ground
And the Tibia should be at a right angle to the ground

Enter $-Exit, +- changes, *-change servo
    0-n Chooses a leg, C-Coxa, F-Femur, T-Tibia
    m - manually move mode to get close

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

  
