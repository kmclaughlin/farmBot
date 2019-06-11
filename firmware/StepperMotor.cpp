#include "StepperMotor.h"
#define MOTOR_ENABLE_DELAY 100
#define ENCODER_PRECISION 2
#define DUAL_MOTOR_CORRECTION 10

StepperMotor::StepperMotor(int stepPin, int dirPin, int enablePin, unsigned long pulseDuration){
  mStepPin = stepPin; 
  mDirPin = dirPin;
  mEnablePin = enablePin;
  
  pinMode(mStepPin, OUTPUT);
  pinMode(mDirPin, OUTPUT);
  pinMode(mEnablePin, OUTPUT);

  dualMotor = false;
  mOpposing = false;

  //set motors off
  enable(false);
  setDirection(true);

  mPulseDuration = pulseDuration;
}

StepperMotor::StepperMotor(int stepPin1, int dirPin1, int enablePin1, int stepPin2, int dirPin2, int enablePin2,
        boolean opposing = true, unsigned long pulseDuration = 1000){
  mStepPin = stepPin1; 
  mDirPin = dirPin1;
  mEnablePin = enablePin1;
  
  mStepPin2 = stepPin2; 
  mDirPin2 = dirPin2;
  mEnablePin2 = enablePin2;
  
  pinMode(mStepPin, OUTPUT);
  pinMode(mDirPin, OUTPUT);
  pinMode(mEnablePin, OUTPUT);
  
  pinMode(mStepPin2, OUTPUT);
  pinMode(mDirPin2, OUTPUT);
  pinMode(mEnablePin2, OUTPUT);

  dualMotor = true;
  mOpposing = opposing;
  
  //set motors off
  enable(false);
  setDirection(true);

  mPulseDuration = pulseDuration;
  pulseDurationDual1 = pulseDuration;
  pulseDurationDual1 = pulseDuration;
}

void StepperMotor::setSteps(int steps, bool dir){
  encoderControlledStep = false;
  //set motor direction
  setDirection(dir);
  mSteps = steps;
}

void StepperMotor::moveEncoderSteps(int steps){
  encoderControlledStep = true;
  targetEncoderValue = encoder->getPosition() + steps;
  //set motor direction
  setDirection(steps > 0);
  // encoder steps are not equal to motor steps. Approx 1 mstep to 1.76 estep 
  mSteps = steps / 2;
}

void StepperMotor::enable(bool value){
  enabled = value;
  digitalWrite(mEnablePin, !value);
  if (dualMotor)
    digitalWrite(mEnablePin2, !value);
  //delay to allow motor state to settle
  delay(MOTOR_ENABLE_DELAY);
  // reset steps if disabling
  if(!value)
    mSteps = 0;
}

void StepperMotor::step(unsigned long elapsedMicros){
  if (dualMotor)
    dualMotorStep(elapsedMicros);
  else
    singleMotorStep(elapsedMicros);
}

void StepperMotor::singleMotorStep(unsigned long elapsedMicros){
  //while steps remain, cycle between setting step pins high for fixed delay time and low for fixed delay time
  // to move motor at desired speed (defined by delay time)
  mStepRunTime += elapsedMicros;
  if(mPulseDuration < mStepRunTime){
    mStepRunTime = 0;
    encoderControlledSteps();
    //TODO replace these digital writes with port manipulation
    if (mSteps > 0) {
      if(!stepping) {
        digitalWrite(mStepPin, HIGH);
        if (dualMotor)
            digitalWrite(mStepPin2, HIGH);
        stepping = true;
      }
      else {
        digitalWrite(mStepPin, LOW);
        if (dualMotor)
            digitalWrite(mStepPin2, LOW);
        stepping = false;
        mSteps--;
      }
    }
  }
}

void StepperMotor::dualMotorStep(unsigned long elapsedMicros){
  //control individual motor speed to keep axis aligned
  mStepRunTime += elapsedMicros;
  mStepRunTimeMotor2 += elapsedMicros;
  

  //long encoderDifference = encoder->getPosition() - encoder2->getPosition();
  //pulseDurationDual1 = mPulseDuration;// + (encoderDifference * DUAL_MOTOR_CORRECTION);
  //pulseDurationDual2 = mPulseDuration;// - (encoderDifference * DUAL_MOTOR_CORRECTION);
    
  if(pulseDurationDual1 < mStepRunTime){
    // set pulse duration for each motor dependant on how far in front or behind it is
    //slow down the one further ahead, ie increase pulse duration
    // only need to update each step
    if(stepping) {
      long encoderDifference = abs(encoder->getPosition()) - abs(encoder2->getPosition());
      pulseDurationDual1 = mPulseDuration + (encoderDifference * DUAL_MOTOR_CORRECTION);
      pulseDurationDual2 = mPulseDuration - (encoderDifference * DUAL_MOTOR_CORRECTION);
    }
    
    mStepRunTime = 0;
    encoderControlledSteps();
    //TODO replace these digital writes with port manipulation
    if (mSteps > 0) {
      if(!stepping) {
        digitalWrite(mStepPin, HIGH);
        stepping = true;
      }
      else {
        digitalWrite(mStepPin, LOW);
        stepping = false;
        //reduce steps only on motor 1
        mSteps--;
      }
    }
  }
  
  if(pulseDurationDual2 < mStepRunTimeMotor2){
    mStepRunTimeMotor2 = 0;
    //TODO replace these digital writes with port manipulation
    if (mSteps > 0) {
      if(!steppingMotor2) {
        digitalWrite(mStepPin2, HIGH);
        steppingMotor2 = true;
      }
      else {
        digitalWrite(mStepPin2, LOW);
        steppingMotor2 = false;
      }
    }
  }
}

void StepperMotor::encoderControlledSteps(){
  //include stepping check as only need to do this every full step
  if(mSteps <= 1 && encoderControlledStep && !stepping){
    long ePos = encoder->getPosition();
    if(abs(encoder->getPosition() - targetEncoderValue) > ENCODER_PRECISION){
      //recalculate steps
      mSteps = abs(encoder->getPosition() - targetEncoderValue) / 2;
      //set direction again if overshot
      //if(((targetEncoderValue - encoder->getPosition()) > 0) != direction)
        //setDirection((targetEncoderValue - encoder->getPosition()) > 0);
    }
    else {
      encoderControlledStep = false;
    }
  }
}

void StepperMotor::setDirection(bool dir){
  direction = dir;
  digitalWrite(mDirPin, dir);
  if (dualMotor){
    if (mOpposing)
      digitalWrite(mDirPin2, !dir);
    else 
      digitalWrite(mDirPin2, dir);
  }
}

