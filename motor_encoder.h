/*
 * Encoder library for Pololu micormotors with encoders. 
 * Based on code from PJRC.COM; see license at end of file. 
 */
#ifndef motor_encoder_h
#define motor_encoder_h

#include <Arduino.h>
#include "digitalWriteFast.h"
#include "direct_pin_read.h"
#include "interrupt_pins.h"

#define VELOCITY_CALC_DELAY 50 // How often to calc velocity, in miliseconds

class Motor;

typedef void (Motor::*MotorUpdateCallback)(int pin1);  

typedef struct {

  volatile int pin1;
  volatile IO_REG_TYPE *pin1_register;
  volatile IO_REG_TYPE *pin2_register;
  IO_REG_TYPE pin1_bitmask;
  IO_REG_TYPE pin2_bitmask;
  
  volatile uint8_t state; // Works like a shift register to hold bits for current and last encoder states. 

  volatile int8_t direction = 0;
  volatile int32_t position = 0; 
  volatile float velocity = 0; 
  volatile float mean_velocity = 0; 
  volatile float acceleration = 0; 
 
  volatile float last_direction = 0;
  volatile float last_position = 0;
  volatile float last_velocity = 0;

  volatile int pulsesToEvent = 0; // Number of pulses remaining to call positionCallback
  
  volatile int eventTriggered = false; // Number of pulses remaining to call positionCallback
  
  volatile unsigned long last_velocity_calc = 0;

  Motor *motor;
  
  volatile MotorUpdateCallback eventCallback = 0; // Called when 'pulses remaining' is  negative
  volatile MotorUpdateCallback updateCallback = 0; // Called when veloicty is updated

} Encoder_internal_state_t;


class Encoder {

  private:
    
    int encoderPin;
  
    int interrupt;
  
    static void update(Encoder_internal_state_t *encoder);
    void read_initial_encoder_state();

    inline static void decPulsesToEvent(Encoder_internal_state_t *encoder, int dec);

    inline void getState(){
      
    }

  public:

    static Encoder_internal_state_t * states[CORE_NUM_INTERRUPT];


    Encoder(int pin1, int pin2);

    // Assuming Uno. Can't just attach the interrupt directly b/c the handler's 
    // don't take arguments. 
    static void isr0(void) { update(states[0]); }
    static void isr1(void) { update(states[1]); }

    inline int32_t getPosition() {
      return states[interrupt]->position;
    }

    inline int8_t getDirection() {
      return states[interrupt]->direction;
    }

    inline float getVelocity() {
      return states[interrupt]->velocity;
    }

    inline float getMeanVelocity() {
      return states[interrupt]->velocity;
    }

    inline float getAcceleration() {
      return states[interrupt]->acceleration;
    }

    inline void setUpdateCallback(Motor *motor, MotorUpdateCallback callback){
      states[interrupt]->motor = motor;
      states[interrupt]->updateCallback = callback;
    }

    inline void setEventCallback(Motor *motor, MotorUpdateCallback callback){
      states[interrupt]->motor = motor;
      states[interrupt]->eventCallback = callback;
    }

    inline void setPulsesToEvent(int pulses){
      states[interrupt]->pulsesToEvent = pulses;
    }
    
    inline int getPulsesToEvent(){
      return states[interrupt]->pulsesToEvent;
    }
    
    inline void clearEvent(){
      states[interrupt]->eventTriggered = false;
    }
    
    inline bool isEventTriggered(){
      return states[interrupt]->eventTriggered;
    }
};



#endif


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