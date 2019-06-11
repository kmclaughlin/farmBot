#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H
#include "Arduino.h"
#include "Encoder.h"


class StepperMotor {
  public:
    StepperMotor(int stepPin, int dirPin, int enablePin, unsigned long pulseDuration = 1000);
    StepperMotor(int stepPin1, int dirPin1, int enablePin1, int stepPin2, int dirPin2, int enablePin2,
        boolean opposing, unsigned long pulseDuration);
    void attachEncoder(Encoder *encdr1, Encoder *encdr2 = NULL) {encoder = encdr1; encoder2 = encdr2;};
    void moveEncoderSteps(int steps);
    void setSteps(int steps, bool dir);
    bool isMoving() {return ((mSteps > 0) || encoderControlledStep);};
    bool isEnabled() {return enabled;};
    Encoder* getEncoder() { return encoder;}; // returns primary encoder
    void step(unsigned long elapsedMicros);
    void enable(bool value);
    void setPulseDuration(unsigned long pulseDuration){ mPulseDuration = pulseDuration; };
    long getTargetEncoderValue() {return targetEncoderValue; };
  protected:
    void setDirection(bool dir);
    void singleMotorStep(unsigned long elapsedMicros);
    void dualMotorStep(unsigned long elapsedMicros);
    void encoderControlledSteps();
    int mStepPin;
    int mDirPin;
    int mEnablePin;
    int mStepPin2;
    int mDirPin2;
    int mEnablePin2;
    int mSteps = 0;
    bool dualMotor = false;
    bool mOpposing;
    bool stepping = false;
    bool stepDelay = false;
    bool steppingMotor2 = false;
    bool enabled = false;
    bool direction;
    unsigned long mPulseDuration = 0;
    unsigned long pulseDurationDual1;
    unsigned long pulseDurationDual2;
    unsigned long mStepRunTime = 0;
    unsigned long mStepRunTimeMotor2 = 0;
    Encoder *encoder;
    Encoder *encoder2;
    long targetEncoderValue = 0;
    bool encoderControlledStep = false;
};

#endif

