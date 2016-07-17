
#ifndef motor_h
#define motor_h


#include "motor_encoder.h"
#include <DRV8835MotorShield.h>
#include <PID_v1.h>

// Number of updates over which to change acceleration
#define ACCELERATION_STEPS 6

#define N_ENCODER_PINS 3


typedef struct {
   
  volatile float targetSpeed = 0;

  double setpointSpeed = 0; // Current desired velocity

  double currentSpeed = 0; // Current velocity

  double commandSpeed = 0; // Velocity value sent to the motor

  volatile int accelerationSteps = 0; // How many acceleration steps remain

  volatile float acceleration = .05;
  
  volatile double ITerm = 0; // PID Integral Term, multipled by Kp
  
  volatile double eTerm = 0; // raw PID diff term
  volatile double dTerm = 0; // raw PID diff term

  
  volatile double lastSpeed = 0; // for the D term

} motor_state_t;


class Motor {

  private:

    DRV8835MotorShield *motors;

    int motorNumber;

    int encoderPin;

    volatile motor_state_t state;

    // Configuration

    float encoderCpr; // Encoder pulses per rotation

    float gearRatio; // Motor gearbox ratio 

    double Kp = 0.0; // 0.006;
    
    double Ki = 0.0; // 0.0005;
    
    double Kd = 0.0 ; // 0.0005;

    static motor_state_t * states[N_ENCODER_PINS];

  protected: 
    Encoder enc;
    
    void update(int pin1);
    
    void stopPositionEvent(int pin1){

    }

    int scale_speed(double target_speed);


  public:

    Motor(int motorN, DRV8835MotorShield *motors, int pin1, int pin2, 
          float encoderCpr=12.0, float gearRatio = 75.0 );

    void setSpeed(int speed);
    
   /**
    * Sends a speed value to the motor driver board, based on setpointSpeed, velocity 
    * corrections and acceleratoin. 
    */
    void setMotorSpeed();
    
    /// Run for a number of pulses
    void runPulses(int pulses, int speed, bool wait = true);


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
      return state.setpointSpeed;
    }

    inline float getMeanVelocity() {
      return enc.getMeanVelocity();
    }

    inline float getAcceleration() {
      return enc.getAcceleration();
    }
  
};

#endif
