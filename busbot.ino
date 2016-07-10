
/* Parts of this code from:
 * Encoder Library, for measuring quadrature encoded signals
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 * Copyright (c) 2011,2013 PJRC.COM, LLC - Paul Stoffregen <paul@pjrc.com>
 * See full license at end of file
 */

// See http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/ for 
// details about how the PID algorithm works. 
#include <Arduino.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <DRV8835MotorShield.h>
#include <TimerOne.h>

#include "digitalWriteFast.h"
#include "direct_pin_read.h"
#include "interrupt_pins.h"

#define LED_PIN 13

#define VELOCITY_CALC_DELAY 100 // How often to calc velocity, in miliseconds

class Motors;

typedef void (Motors::*MotorUpdateCallback)();  

typedef struct {
  
  volatile IO_REG_TYPE *pin1_register;
  volatile IO_REG_TYPE *pin2_register;
  IO_REG_TYPE pin1_bitmask;
  IO_REG_TYPE pin2_bitmask;
  
  volatile uint8_t state; // Works like a shift register to hold bits for current and last encoder states. 
  
  volatile int8_t direction = 0;
  volatile int32_t position = 0; 
  volatile int32_t velocity = 0; 
  //volatile int32_t acceleration = 0; 

  volatile int8_t last_direction = 0;
  volatile int32_t last_position = 0;

  volatile unsigned long last_velocity_calc = 0;

  Motors *motors;
  volatile MotorUpdateCallback callback = 0;    

} Encoder_internal_state_t;

Encoder_internal_state_t encoder;

class Encoder {

  private:

    Encoder_internal_state_t encoder;

    static void update(Encoder_internal_state_t *encoder);
    void read_initial_encoder_state();

  public:

    static Encoder_internal_state_t * interruptArgs[CORE_NUM_INTERRUPT];

    // Assuming Uno. Can't just attach the interrupt directly b/c the handler's 
    // don't take arguments. 
    static void isr0(void) { update(interruptArgs[0]); }
    static void isr1(void) { update(interruptArgs[1]); }

    inline Encoder(int pin1, int pin2) {
    
      switch (pin1) {
          case CORE_INT0_PIN:
            interruptArgs[0] = &encoder;
            attachInterrupt(0, isr0, CHANGE);
            Serial.print("Attach 0 to ");Serial.println(CORE_INT0_PIN);
            break;
          case CORE_INT1_PIN:
            interruptArgs[1] = &encoder;
            attachInterrupt(1, isr1, CHANGE);
            Serial.print("Attach 1 to ");Serial.println(CORE_INT1_PIN);
            break;

      }

      encoder.pin1_register = PIN_TO_BASEREG(pin1);
      encoder.pin1_bitmask = PIN_TO_BITMASK(pin1);

      encoder.pin2_register = PIN_TO_BASEREG(pin1);
      encoder.pin2_bitmask = PIN_TO_BITMASK(pin1);

      read_initial_encoder_state();

    }

    inline int32_t getPosition() {
      return encoder.position;
    }

    inline int8_t getDirection() {
      return encoder.direction;
    }

    inline int8_t getVelocity() {
      return encoder.velocity;
    }

    inline void setCallback(Motors *motors, MotorUpdateCallback callback){
      encoder.motors = motors;
      encoder.callback = callback;
    }
};

Encoder_internal_state_t * Encoder::interruptArgs[CORE_NUM_INTERRUPT];

void Encoder::read_initial_encoder_state(){
    delayMicroseconds(2000);
    uint8_t s = 0;
    if (DIRECT_PIN_READ(encoder.pin1_register, encoder.pin1_bitmask)) {
      s |= 1;
    }
    if (DIRECT_PIN_READ(encoder.pin2_register, encoder.pin2_bitmask)) {
      s |= 2;
    }
    encoder.state = s;
}

void Encoder::update(Encoder_internal_state_t *encoder) {

  

  /// THe important parts of this code originally from: Paul Stoffregen <paul@pjrc.com>

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
      break;
    case 2:  // 2  = B0010
    case 4:  // 4  = B0100
    case 11: // 11 = B1011
    case 13: // 13 = B1101
      encoder->position--;
      encoder->direction = -1;
      break;
    // These are cases when only one pin has an interrupt
    case 3:  // 3  = B0011
    case 12: // 12 = B1100
      encoder->position += 2;
      encoder->direction = 1;
      break;
    case 6: // 6 = B0110
    case 9: // 9 = B1001
      encoder->position -= 2;
      encoder->direction = -1;
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
    if (encoder->last_velocity_calc != 0 && ms > encoder->last_velocity_calc){
            
      encoder->velocity = float( encoder->position - encoder->last_position ) / (ms - encoder->last_velocity_calc) * 1000.0;

      encoder->last_position = encoder->position;
      encoder->last_direction = encoder->direction;

      
      if (encoder->callback != 0){

        Motors *motors = encoder->motors;
        MotorUpdateCallback f = encoder->callback;
        (motors->*f)();

      }
    }

    encoder->last_velocity_calc = ms;
  }
}

class Motors {

  protected:

    Encoder enc1;
    Encoder enc2;

    unsigned long last_read = 0;

    DRV8835MotorShield motors;

    double Kp_p = 1.56;
    double Ki_p = .5;
    double Kd_p = .05;

    PID m1_p_pid;
    PID m2_p_pid;

    double m1_p;
    double m2_p;

    double m1_output_v;
    double m2_output_v;

    double Kp_v = 1;
    double Ki_v = 1;
    double Kd_v = 1;

    double m1_v;
    double m2_v;

    void test_motors(int speed);

    

  public:

      void encoderUpdate();

      double m1_target_p;
      double m2_target_p;

      Motors();

      void autotune();

      void test();

      bool run();

      void runTo(int m1_pos, int m2_pos);
      
      void setSpeed(int m1_v, int m2_v);

      void encoderCallback1(){
        encoderUpdate();
      }

      void encoderCallback2(){
        encoderUpdate();
      }

};

Motors::Motors() : 
  m1_p_pid(&m1_p, &m1_output_v, &m1_target_p, Kp_p, Ki_p, Kd_p, DIRECT),
  m2_p_pid(&m2_p, &m2_output_v, &m2_target_p, Kp_p, Ki_p, Kd_p, DIRECT) ,
  enc1(2,4), enc2(3,5) {

  Serial.println("Init motors");

  m1_p_pid.SetMode(AUTOMATIC);
  m2_p_pid.SetMode(AUTOMATIC);

  m1_p_pid.SetOutputLimits(-400.0,400.0);  
  m2_p_pid.SetOutputLimits(-400.0,400.0);  

  motors.flipM1(true);
  motors.flipM2(true);

  enc1.setCallback(this, &Motors::encoderCallback1);
  enc2.setCallback(this, &Motors::encoderCallback2);
      
}

void Motors::runTo(int m1_pos, int m2_pos) {
  m1_target_p = m1_pos;
  m2_target_p = -m2_pos;

  while(run()){
    delay(50);
  }

  
}

void Motors::setSpeed(int m1_v, int m2_v){
  motors.setM1Speed(m1_v);
  motors.setM2Speed(-m2_v);
}

void Motors::encoderUpdate(){

  int32_t next_m1p = enc1.getPosition();
  int32_t next_m2p = enc2.getPosition();

  float read_span = millis() - last_read;
  
  m1_v = enc1.getVelocity();
  m2_v = enc2.getVelocity();
  
  m1_p = next_m1p;
  m2_p = next_m2p;

  m1_p_pid.Compute();
  m2_p_pid.Compute();

  Serial.print("Motors: "); 
  Serial.print(" p1=");Serial.print(m1_p);Serial.print(" v1=");Serial.print(m1_v);Serial.print(" ov1=");Serial.print(m1_output_v);
  Serial.print(" p2=");Serial.print(m2_p);Serial.print(" v2=");Serial.print(m2_v);Serial.print(" ov2=");Serial.print(m2_output_v);Serial.println("");

  
  last_read = millis();


};

bool Motors::run() {

  encoderUpdate();

  m1_p_pid.Compute();
  m2_p_pid.Compute();

  float rel_pos = float(abs(m1_p)+abs(m2_p)) / float(abs(m1_target_p)+abs(m2_target_p));


  motors.setM1Speed(m1_output_v);
  motors.setM2Speed(m2_output_v);

  if (rel_pos >= .98 and rel_pos <= 1.02) {
    return false;
  } else {
    return true;
  }

}

Motors *motors;


void setup()
{
  
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  motors = new Motors();

}


void loop() {

  motors->setSpeed(75,50);

  motors->encoderUpdate();

  delay(50);


  
}


/* Encoder Library, for measuring quadrature encoded signals
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 * Copyright (c) 2011,2013 PJRC.COM, LLC - Paul Stoffregen <paul@pjrc.com>
 *
 * Version 1.2 - fix -2 bug in C-only code
 * Version 1.1 - expand to support boards with up to 60 interrupts
 * Version 1.0 - initial release
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

