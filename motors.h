/**
 * Interface to the Pololu DRV8835 Motor Shield, to integrate the encoders. 
 */

#ifndef motors_h
#define motors_h

#include <DRV8835MotorShield.h>
#include "motor.h"



/**
 * Joins two motors and allows commanding them together. 
 */
class Motors {

  protected:

    DRV8835MotorShield motorShield;

    float pulsesPerTurn = 1600.0; // number of pulses for an in-place turn. 

  public:

      Motor motor1;
      Motor motor2;

      Motors() : motor1(1,&motorShield,2,4), motor2(2,&motorShield,3,5) {
        motorShield.flipM1(true);
        motorShield.flipM2(true);
     
      }

      void setSpeed(int m1_v, int m2_v) {
        motor1.setSpeed(m1_v);
        motor2.setSpeed(-m2_v);
      }

      void runPulses(int p1, int m1_v, int p2, int m2_v, bool wait = true) {
        motor1.runPulses(p1, m1_v);
        motor2.runPulses(p2, -m2_v);

        
        setSpeed(0,0);
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


