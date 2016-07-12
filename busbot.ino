
/* Parts of this code from:
 * Encoder Library, for measuring quadrature encoded signals
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 * Copyright (c) 2011,2013 PJRC.COM, LLC - Paul Stoffregen <paul@pjrc.com>
 * See full license at end of file
 */

// See http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/ for 
// details about how the PID algorithm works. 


#include "motors.h"

#define LED_PIN 13

Motors *motors;

void setup()
{
  
  Serial.begin(115200);
  motors = new Motors();

}

void loop() {

  motors->move(5000,1000);
  motors->movingTurn(5000, .5, 2000);
  motors->move(5000,1000);
  motors->movingTurn(5000, .5, 2000);
  motors->turn(1, 1000);
  motors->turn(-1, 1000);
  

}



