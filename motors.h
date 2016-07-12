/**
 * Interface to the Pololu DRV8835 Motor Shield, to integrate the encoders. 
 */

#ifndef motors_h
#define motors_h

#include <DRV8835MotorShield.h>
#include "motor_encoder.h"

class Motor {

  private:

    DRV8835MotorShield *motors;

    int motorNumber;

    int encoderPin;

    float setpointSpeed; // Current commanded velocity

    float outputSpeed; // spped value sent to the motor board, about 10% of setpointSpeed

    float error; // Erro between target speed and encoder speed. 

    float correction = 0;

    float accelerationStep = .05;

    float accelerationAdjustment = 1; // An adjustable paramter that is mult by velocity

    float encoderCpr; // Encoder pulses per rotation

    float gearRatio; // Motor gearbox ratio 

    float inOutRatio = 10.0;

    float Kp = 0.05;

    float running = true;

  protected: 
    Encoder enc;
    
    void update(int pin1){
      
      error = (setpointSpeed * accelerationAdjustment ) - enc.getMeanVelocity();
      
      correction += Kp * error;

      if (abs(accelerationAdjustment-1.0) > abs(accelerationStep)/2){
        accelerationAdjustment += accelerationStep;
      }

      setMotorSpeed();

      if (true){
        Serial.print(pin1);
        Serial.print(" p=");Serial.print(enc.getPosition());
        Serial.print(" t=");Serial.print(setpointSpeed);
        Serial.print(" v=");Serial.print(enc.getMeanVelocity());
        Serial.print(" e=");Serial.print(error);
        Serial.print(" a=");Serial.print(accelerationAdjustment);
        Serial.print(" s=");Serial.print(accelerationStep);
        Serial.print(" c=");Serial.print(correction);
        Serial.print(" o=");Serial.print(outputSpeed);
        Serial.println("");
      }
      
    }

    void stopPositionEvent(int pin1){
        running = false;
    }

    int scale_speed(double target_speed){
      // The range from 0 to 25 has no or erratic motion, so move half of the range to zero, 
      // the other half up to 25. 

      // 25 is about the lowest practical speed for the motor controller, so an input 
      // value of 0 should get bumped up to 25. 
      
      int dir;
      if (setpointSpeed == 0){
        return 0;
      } else if (setpointSpeed < 0){
        dir = -1;
      } else {
        dir = 1;
      }
      
      return map(abs(target_speed),0,4000,25,400) * dir;

    }

    /**
     * Sends a speed value to the motor driver board, based on setpointSpeed, velocity 
     * corrections and acceleratoin. 
     */
    void setMotorSpeed(){

      if (! running){
        return;
      }

      outputSpeed =  ((setpointSpeed + correction) * accelerationAdjustment);

      // Case when the target speed is too low after accelleration adjustment, so
      // bump it up to get things started. 
      while (setpointSpeed != 0 && outputSpeed == 0){ 
        accelerationAdjustment += accelerationStep;
        outputSpeed =  ((setpointSpeed + correction) * accelerationAdjustment) ;
      }


      switch(motorNumber){
        case 1:
          motors->setM1Speed(scale_speed(outputSpeed));
          break;
        case 2:
          motors->setM2Speed(scale_speed(outputSpeed));
          break;
        default:
          Serial.println("ERROR");
      }
    }

    void setAccelerationParams(){

      #define ACCELERATION_STEPS 10.0

      float current = enc.getVelocity();
      float targetNotZero = setpointSpeed;

      if (targetNotZero == 0){
        targetNotZero = 1;
      }
      
      accelerationStep =  (float)(setpointSpeed - current) / targetNotZero / ACCELERATION_STEPS;
      accelerationAdjustment = current / setpointSpeed;

      correction = 0;
      error = 0;
    }

  public:

    void setSpeed(int speed){

      // if there is a current speed, se the accellerationAdjustment as the ratio between the two, 
      // so the accelleration process will smoothing change between them. 

      setpointSpeed = speed;

      setAccelerationParams();

      running = true;

      setMotorSpeed();
    }

    /// Run for a number of pulses
    void runPulses(int pulses, int speed){

        if (pulses < 0){
          pulses = -pulses;
          speed = -speed;
        }
        
        enc.setPulsesToEvent(pulses);
        setSpeed(speed);
    }

    bool isRunning(){
      return running;
    }

    inline int32_t getPosition() {
      return enc.getPosition();
    }

    inline int8_t getDirection() {
      return enc.getDirection();
    }

    inline float getVelocity() {
      return enc.getVelocity();
    }

    inline float getTargetVelocity() {
      return setpointSpeed;
    }

    inline float getMeanVelocity() {
      return enc.getMeanVelocity();
    }

    inline float getAcceleration() {
      return enc.getAcceleration();
    }

    Motor(int motorN, DRV8835MotorShield *motors, int pin1, int pin2, float encoderCpr=12.0, float gearRatio = 75.0 ) :
      motorNumber(motorN), encoderPin(pin1), motors(motors), enc(pin1, pin2), encoderCpr(encoderCpr), gearRatio(gearRatio) {
      enc.setUpdateCallback(this, &Motor::update);
      enc.setEventCallback(this, &Motor::stopPositionEvent);
    }
  
};

/**
 * Joins two motors and allows commanding them together. 
 */
class Motors {

  protected:

    DRV8835MotorShield motors;

    float pulsesPerTurn = 1600.0; // number of pulses for an in-place turn. 

  public:

      Motor motor1;
      Motor motor2;

      Motors() : motor1(1,&motors,2,4), motor2(2,&motors,3,5) {
        motors.flipM1(true);
        motors.flipM2(true);
     
      }

      void setSpeed(int m1_v, int m2_v) {
        motor1.setSpeed(m1_v);
        motor2.setSpeed(-m2_v);
      }

      void runPulses(int p1, int m1_v, int p2, int m2_v, bool wait = true) {
        motor1.runPulses(p1, m1_v);
        motor2.runPulses(p2, -m2_v);

        while(wait && isRunning()){ 
          delay(10); 
        }
      }

      bool isRunning(){
        return motor1.isRunning() || motor2.isRunning();
      }

      // Turn in place a number of revolutions
      void turn(float rev, float speed){
        runPulses(rev*pulsesPerTurn,speed,rev*pulsesPerTurn,-speed);
      }

      void move(float pulses, float speed){
        runPulses(pulses,speed,pulses,speed);
      }

      // Move forward while turning
      void movingTurn(float pulses, float rev, float speed){

        // Total run time
        float rt = pulses / speed;

        // Distance each motor runs, considering turn. 
        float m1_dist = pulses + (rev*pulsesPerTurn);
        float m2_dist = pulses - (rev*pulsesPerTurn);

        float m1_speed = m1_dist / rt;
        float m2_speed = m2_dist / rt; 
        
        runPulses(m1_dist, m1_speed, m2_dist, m2_speed);
      }

      void stop(){
        setSpeed(0,0);
      }

};

#endif


