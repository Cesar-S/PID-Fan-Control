## Control software for PID Test Rig
Sketch to control the PID test/learning rig. The hardware system for this code is described in the [Arduino++ blog](https://arduinoplusplus.wordpress.com/2017/06/10/pid-control-experiment-making-the-testing-rig/)

## Hardware requirements
* Arduino Uno/Nano/Mini/etc
* 2 line LCD module
* Rotary encoder with built in selection switch
* SR-04 Ultrasonic sensor

## Function
This sketch controls a fan blowing into a vertical acrylic tube designed to 
contain a floating ping pong ball. The control algorithm is supposed to 
levitate the ball at a set height from the bottom of the tube.
* Distance using a SR-04 ultrasonic sensor located at the open end of the tube.
* The fan at the other end of the tube is controlled with a PWM signal into a 
L293D motor controller. 
* A 2 line LCD display module and a rotary encoder are used to change the PID constants 
while tuning the loop. The LCD is connecteed with a Shift Register one-wire backpack;
other types of LCD will requires changes to the class initialisation parameters
to suit that hardware type.
* PID parameters can be saved to EEPROM and are reloaded at startup.
* Data can be logged to the Serial Monitor, Serial Plotter or PLX-DAQ for 
Microsoft Excel, by menu selection.
