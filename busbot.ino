
/* Parts of this code from:
 * Encoder Library, for measuring quadrature encoded signals
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 * Copyright (c) 2011,2013 PJRC.COM, LLC - Paul Stoffregen <paul@pjrc.com>
 * See full license at end of file
 */
 
#include <advancedSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include "TimerOne.h"

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

#include "motors.h"

Motors *motors;

void setup()
{
  
  Serial.begin(115200);
  aSerial.setPrinter(Serial);
  Serial.println("-----------");
  //motors = new Motors();

  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }

}

int getHeading(){
    /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  
  float declinationAngle = 0.22;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 

  return headingDegrees;
  
  
}

float setpoint = 200;

#include "motor.h"
DRV8835MotorShield motorShield;
Motor motor1(1,&motorShield,2,4);

void loop() {

  motor1.runPulses(2000,1000, true);

  motor1.setSpeed(0);
 
  delay(100000);

}

void xloop() {

  //Serial.println());

  float heading = getHeading();

  // Error in units of rotations
  float error = (setpoint - heading) / 360.0 ;

  float t = error * .05;
  float speed = error * 400 + 500;

  Serial.print(heading, 6); Serial.print(" ");Serial.print(error, 6); Serial.print(" ");Serial.print(t, 6); Serial.print(" "); Serial.println(speed);

  motors->turn(t, speed);

  delay(1000);

}



