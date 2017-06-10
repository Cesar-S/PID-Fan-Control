## Control software for PID Test Rig
Sketch to control the PID test/learning rig. The hardware system for this code is described in the [Arduino++ blog](https://arduinoplusplus.wordpress.com/2017/)

## Hardware requirements
* Arduino Uno/Nano/Mini/etc
* 2 line LCD module
* Rotary encoder with built in selection switch
* SR-04 Ultrasonic sensor

## Function
This sketch controls a fan blowing into a vertical acrylic tube designed to 
contain a floating a ping pong ball. The control algorithm is supposed to 
maintain the ball at a set height from the bottom of the tube.
* Distance measurement is with a SR-04 ultrasonic sensor located at the open end of 
the tube.
* The fan is controlled with a PWM signal fed into a L293N morot controller with built
in diode protection. 
* A 2 line LCD display module and a rotary encoder are used to change the PID constants 
while tuning the loop. The LCD is conencteed with a Shift Register one-wire backpack;
other types of LCD will requires changes to the class initialisation parameters
to suit that hardware type.
* PID parameters can be saved to EEPROM and are relaoded at startup.
* Data is logged to the Serial Monitor to allow it to be graphed externally. Arduino IDE
Serial Plot can also be used, but the timestamp needs to be removed from the data to 
make the graph legible.
