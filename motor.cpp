
#include "motor.h"

#include <advancedSerial.h> // For aSerial, a better debugging print


#define MAX_SPEED 4000
#define MAX_SHIELD_INPUT 400

motor_state_t * Motor::states[N_ENCODER_PINS];

Motor::Motor(int motorN, DRV8835MotorShield *motors, int pin1, int pin2, 
      float encoderCpr, float gearRatio ) :
  motorNumber(motorN), encoderPin(pin1), motors(motors), 
  enc(pin1, pin2), encoderCpr(encoderCpr), gearRatio(gearRatio)
  
{
    
  enc.setUpdateCallback(this, &Motor::update);
  enc.setEventCallback(this, &Motor::stopPositionEvent);
  
  states[encoderPin] = new motor_state_t();

  switch(motorN) {
    case 1:
      motors->flipM1(true);
      break;
    case 2:
      motors->flipM2(true);
      break;
  
  }
      
}

void Motor::update(int pin1){

  motor_state_t *state = states[encoderPin];
  
  state->currentSpeed = enc.getMeanVelocity();

  state->eTerm = state->setpointSpeed - state->currentSpeed;
  state->ITerm += (Ki * state->eTerm);
  state->dTerm = state->lastSpeed - state->currentSpeed;
    
  state->lastSpeed = state->commandSpeed = 
    state->setpointSpeed + 
    ( Kp * state->eTerm) + 
    state->ITerm + 
    ( Kd * state->dTerm);

  if ( state->accelerationSteps ) {
    state->setpointSpeed += state->acceleration;
    state->accelerationSteps --;
  }

  setMotorSpeed();

}

int Motor::scale_speed(double target_speed){
  // The range from 0 to 25 has no or erratic motion, so move half of the range to zero, 
  // the other half up to 25. 

  // 25 is about the lowest practical speed for the motor controller, so an input 
  // value of 0 should get bumped up to 25. 
  
  int dir;
  if ( int(target_speed) == 0){
    return 0;
  } else if ( int(target_speed) < 0){
    dir = -1;
  } else {
    dir = 1;
  }
  
  return map(abs(target_speed),0,MAX_SPEED,25,MAX_SHIELD_INPUT) * dir;

}

void Motor::setMotorSpeed(){

  motor_state_t *state = states[encoderPin];
  
  aSerial.p("setMotorSpeed")
    .p(" cs=").p(state->currentSpeed)
    .p(" ts=").p(state->targetSpeed)
    .p(" sps=").p(state->setpointSpeed)
    .p(" cmds=").p(state->commandSpeed)
    .p(" ss=").p(scale_speed(state->commandSpeed))
    .p(" aa=").p(state->acceleration)
    .p(" as=").p(state->accelerationSteps)  
    .p(" Te=").p(state->eTerm)  
    .p(" Td=").p(state->dTerm)
    .p(" Ti=").p(state->ITerm)  
    .pln("");

  switch(motorNumber){
    case 1:
      motors->setM1Speed(scale_speed(state->commandSpeed));
      break;
    case 2:
      motors->setM2Speed(scale_speed(state->commandSpeed));
      break;
    default:
      Serial.println("ERROR");
  }
}

void Motor::setSpeed(int speed){

  motor_state_t *state = states[encoderPin];
  
  state->targetSpeed = speed;

  // Calc the acceleration parameters, then get started with one acceleration step, 
  // to ensure that there is at least one update().
  // We assume that the current actual velocity has reached the setpoint b/c the
  // velocity measurement is a bit unstable. 
  state->acceleration =  (float)(state->targetSpeed - state->setpointSpeed) / ACCELERATION_STEPS;

  state->accelerationSteps = ACCELERATION_STEPS - 1;

  state->setpointSpeed += state->acceleration;
  state->commandSpeed = state->setpointSpeed;
  state->lastSpeed = state->setpointSpeed;
  state->ITerm = 0;
  
  setMotorSpeed();
}

/// Run for a number of pulses Asynchronously
void Motor::runPulses(int pulses, int speed, bool wait){

    aSerial.p("runPulses ").p(pulses).p(" ").pln(speed);
    
    if (pulses == 0) {
      return; 
    } else if (pulses < 0){
      pulses = -pulses;
      speed = -speed;
    }
   
    enc.clearEvent();
    enc.setPulsesToEvent(pulses);
    
    setSpeed(speed);
    
    while( wait && !enc.isEventTriggered()){
      delay(10);
    }
    
}


