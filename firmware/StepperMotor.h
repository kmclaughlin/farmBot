#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H
//#include "Arduino.h"
#include "Encoder.h"
#include "Config.h"

class StepperMotor {
  public:
    StepperMotor(int stepPin, int dirPin, int enablePin, int speed, int minSpeed, unsigned int stepsPerMM, int accelRate);
    StepperMotor(int stepPin1, int dirPin1, int enablePin1, int stepPin2, int dirPin2, int enablePin2,
        boolean opposing, int speed, int minSpeed, unsigned int stepsPerMM, int accelRate);
    void attachEncoder(Encoder *encdr1, Encoder *encdr2 = NULL) {encoder = encdr1; encoder2 = encdr2;};
    void moveEncoderSteps(int steps);
    void setSteps(int steps, bool dir);
    bool isMoving() { return ((steps > 0) || encoderControlledStep);};
    bool isEnabled() { return enabled;};
    Encoder* getEncoder() { return encoder;}; // returns primary encoder
    void step(unsigned int elapsedMicros);
    void enable(bool value);
    long getTargetEncoderValue() {return targetEncoderValue; };
    void setSpeed(int speed);
    void setAccel(int accel) { accelRate = accel;};
    void setMinSpeed(int minSpeed);
    bool movementCheck();
    
  private:
  void commonInitialise();
    void setDirection(bool dir);
    void singleMotorStep(unsigned int elapsedMicros);
    void dualMotorStep(unsigned int elapsedMicros);
    void encoderControlledSteps();
    void updateAcceleration(unsigned long elapsedMicros);
    
    int stepPin, dirPin, enablePin;
    int stepPin2, dirPin2, enablePin2;
    int speed, minSpeed, accelRate, accTimer;
    unsigned int stepsPerMM;
    volatile int steps;
    volatile long currentStepDelayDuration;
    long maxStepDelayDuration, stepDelayDuration, stepDelayDual1, stepDelayDual2;
    bool dualMotor, opposing, stepDelay, stepDelayMotor2, enabled;
    bool direction;
    unsigned int stepRunTime, stepRunTimeMotor2;
    Encoder *encoder, *encoder2;
    long targetEncoderValue;
    bool encoderControlledStep;
};

#endif

