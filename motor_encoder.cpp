/*
 * Encoder library for Pololu micormotors with encoders. 
 * Based on code from PJRC.COM; see license at end of file. 
 */

#include "motor_encoder.h"

Encoder_internal_state_t * Encoder::states[CORE_NUM_INTERRUPT] = {};

Encoder::Encoder(int pin1, int pin2) {
  
  encoderPin = pin1;

  switch (pin1) {
      case CORE_INT0_PIN:
        interrupt =  0;
        attachInterrupt(0, isr0, CHANGE);
        break;
      case CORE_INT1_PIN:
        interrupt = 1;
        attachInterrupt(1, isr1, CHANGE);
        break;

  }

  Encoder_internal_state_t *encoder = states[interrupt] = new Encoder_internal_state_t();

  encoder->pin1 = pin1;
  encoder->pin1_register = PIN_TO_BASEREG(pin1);
  encoder->pin1_bitmask = PIN_TO_BITMASK(pin1);

  encoder->pin2_register = PIN_TO_BASEREG(pin2);
  encoder->pin2_bitmask = PIN_TO_BITMASK(pin2);
    
  uint8_t s = 0;
  
  if (DIRECT_PIN_READ(encoder->pin1_register, encoder->pin1_bitmask)) {
    s |= 1;
  }
  
  if (DIRECT_PIN_READ(encoder->pin2_register, encoder->pin2_bitmask)) {
    s |= 2;
  }
  
  encoder->state = s;

  encoder->last_velocity_calc = millis();


}



void Encoder::update(Encoder_internal_state_t *encoder) {


  /// The important parts of this code originally from: Paul Stoffregen <paul@pjrc.com>

  uint8_t p1val = DIRECT_PIN_READ(encoder->pin1_register, encoder->pin1_bitmask);
  uint8_t p2val = DIRECT_PIN_READ(encoder->pin2_register, encoder->pin2_bitmask);

  // Keep only the bottom two bits, the last values for A and B
  uint8_t state = encoder->state & 3;
  
  // Set the current tw values for A and B above the old ones.
  if (p1val){
    state |= 4;
  }
  if (p2val){
    state |= 8;
  }

  // For the next round, shift the old values off.
  encoder->state = (state >> 2);

  switch (state) {
    case 1:  // 1  = B0001
    case 7:  // 7  = B0111
    case 8:  // 8  = B1000
    case 14: // 14 = B1110
      encoder->position++;
      encoder->direction = 1;
      decPulsesToEvent(encoder, 1);
      break;
    case 2:  // 2  = B0010
    case 4:  // 4  = B0100
    case 11: // 11 = B1011
    case 13: // 13 = B1101
      encoder->position--;
      encoder->direction = -1;
      decPulsesToEvent(encoder, 1);
      break;
    // These are cases when only one pin has an interrupt
    case 3:  // 3  = B0011
    case 12: // 12 = B1100
      encoder->position += 2;
      encoder->direction = 1;
      decPulsesToEvent(encoder, 2);
      break;
    case 6: // 6 = B0110
    case 9: // 9 = B1001
      encoder->position -= 2;
      encoder->direction = -1;
      decPulsesToEvent(encoder, 2);
      break;
      // Unused, impossible states.
      // 0
      // 5  = B0101
      // 10 = B1010
      // 15 = B1111
  }

  unsigned long ms = millis();

  if (ms >  encoder->last_velocity_calc + VELOCITY_CALC_DELAY){


    // FIXME! Should probably force a velocity cal when the direction changes. 
    if (ms > encoder->last_velocity_calc){ // Wrap-around protection
            
      encoder->velocity = float( encoder->position - encoder->last_position ) / 
                          float(ms - encoder->last_velocity_calc) * 1000.0;

      float acceleration = float( encoder->velocity - encoder->last_velocity ) / 
                          float(ms - encoder->last_velocity_calc) * 1000.0;

      // Rolling average of acceleration
      encoder->acceleration *= .9;
      encoder->acceleration += acceleration/10.0;


      
      encoder->last_velocity_calc = ms;
      encoder->last_position = encoder->position;
      encoder->last_velocity = encoder->velocity; 
      
      encoder->mean_velocity -= encoder->mean_velocity / 5.0; // 5 sample moving average
      encoder->mean_velocity += encoder->velocity / 5.0;
      
      encoder->last_direction = encoder->direction;

      if (encoder->updateCallback != 0){
        Motor *motor = encoder->motor;
        MotorUpdateCallback f = encoder->updateCallback;
        (motor->*f)(encoder->pin1);
      }
    }
  }
}


void Encoder::decPulsesToEvent(Encoder_internal_state_t *encoder, int dec) {


  if (encoder->pulsesToEvent <= 0){
    return;
  } 

  encoder->pulsesToEvent -= dec;

  if (encoder->pulsesToEvent <= 0){
    

    encoder->pulsesToEvent = 0;
    encoder->eventTriggered = true;
    
    Motor *motor = encoder->motor;
    MotorUpdateCallback f = encoder->eventCallback;
    
    if (f){
      (motor->*f)(encoder->pin1);
    }
    
  } 

}

/* Encoder Library, for measuring quadrature encoded signals
   http://www.pjrc.com/teensy/td_libs_Encoder.html
   Copyright (c) 2011,2013 PJRC.COM, LLC - Paul Stoffregen <paul@pjrc.com>

   Version 1.2 - fix -2 bug in C-only code
   Version 1.1 - expand to support boards with up to 60 interrupts
   Version 1.0 - initial release

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
   THE SOFTWARE.
*/