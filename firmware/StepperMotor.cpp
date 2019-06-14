#include "StepperMotor.h"

StepperMotor::StepperMotor(int stepPin, int dirPin, int enablePin, int speed, int minSpeed, unsigned int stepsPerMM, int accelRate){
  this->stepPin = stepPin; 
  this->dirPin = dirPin;
  this->enablePin = enablePin;
  
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  
  dualMotor = false;
  opposing = false;
  this->stepsPerMM = stepsPerMM;
  this->accelRate = accelRate;
  setSpeed(speed);
  setMinSpeed(minSpeed);
  commonInitialise();
}

StepperMotor::StepperMotor(int stepPin1, int dirPin1, int enablePin1, int stepPin2, int dirPin2, int enablePin2,
        boolean opposing, int speed, int minSpeed, unsigned int stepsPerMM, int accelRate){
  this->stepPin = stepPin1; 
  this->dirPin = dirPin1;
  this->enablePin = enablePin1;
  
  this->stepPin2 = stepPin2; 
  this->dirPin2 = dirPin2;
  this->enablePin2 = enablePin2;
  
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(enablePin2, OUTPUT);

  dualMotor = true;
  this->opposing = opposing;
  this->stepsPerMM = stepsPerMM;
  this->accelRate = accelRate;
  setSpeed(speed);
  setMinSpeed(minSpeed);
  commonInitialise();
}

void StepperMotor::commonInitialise(){
  //set motors off
  enable(false);
  setDirection(true);

  stepDelayDual1 = currentStepDelayDuration;
  stepDelayDual2 = currentStepDelayDuration;

  stepRunTime = 0;
  stepRunTimeMotor2 = 0;
  targetEncoderValue = 0;
  encoderControlledStep = false;
  
  stepDelay = false;
  stepDelayMotor2 = false;
  steps = 0;
  accTimer = 0;
}

void StepperMotor::setSteps(int steps, bool dir){
  encoderControlledStep = false;
  //set motor direction
  setDirection(dir);
  this->steps = abs(steps);
  currentStepDelayDuration = maxStepDelayDuration;
}

//TODO doesn't allow for new instructions during movement
void StepperMotor::moveEncoderSteps(int encoderSteps){
  encoderControlledStep = abs(encoderSteps) > 0;
  targetEncoderValue = encoder->getPosition() + encoderSteps;
  // set motor direction
  setDirection(encoderSteps > 0);
  // encoder steps are not equal to motor steps. Approx 1 mstep to 1.76 estep 
  steps = abs(encoderSteps) / ENC_STEP_CONV_FACTOR;
  currentStepDelayDuration = maxStepDelayDuration;
}

void StepperMotor::enable(bool enable){
  enabled = enable;
  digitalWrite(enablePin, !enabled);
  if (dualMotor)
    digitalWrite(enablePin2, !enabled);
  //delay to allow motor state to settle
  delay(MOTOR_ENABLE_DELAY);
  // reset steps if disabling
  if(!enabled)
    steps = 0;
}

void StepperMotor::step(unsigned int elapsedMicros){
  if (dualMotor)
    dualMotorStep(elapsedMicros);
  else
    singleMotorStep(elapsedMicros);
  updateAcceleration(elapsedMicros);
}

void StepperMotor::singleMotorStep(unsigned int elapsedMicros){
  //while steps remain, cycle between setting step pins high for fixed delay time and low for fixed delay time
  // to move motor at desired speed (defined by delay time)
  if (steps > 0) {
    stepRunTime += elapsedMicros;
    if(!stepDelay) {
      digitalWrite(stepPin, HIGH);
      // digital write is slow enough to not need a delay.
      // digital write will take about 6us
      digitalWrite(stepPin, LOW);
      stepDelay = true;
    }
    else {
      if(currentStepDelayDuration < stepRunTime){
        stepRunTime = 0;
        steps--;
        encoderControlledSteps();
        stepDelay = false;
      }
    }
  }
}

void StepperMotor::dualMotorStep(unsigned int elapsedMicros){
  if (steps > 0) {
    //control individual motor speed to keep axis aligned
    stepRunTime += elapsedMicros;
    stepRunTimeMotor2 += elapsedMicros;
     
    if(!stepDelay) {
      digitalWrite(stepPin, HIGH);
      // digital write is slow enough to not need a delay.
      // digital write will take about 6us
      digitalWrite(stepPin, LOW);
      stepDelay = true;
    }
    else {
      if(stepDelayDual2 < stepRunTime){
        stepRunTime = 0;
        stepDelay = false;
      }
    }
    
    if(!stepDelayMotor2) {
      digitalWrite(stepPin2, HIGH);
      // digital write is slow enough to not need a delay.
      // digital write will take about 6us
      digitalWrite(stepPin2, LOW);
      stepDelayMotor2 = true;
    }
    else {
      if(stepDelayDual2 < stepRunTimeMotor2){
        stepRunTimeMotor2 = 0;
        steps--;
        encoderControlledSteps();
        stepDelayMotor2 = false;
        
        // set pulse duration for each motor dependant on how far in front or behind it is
        //slow down the one further ahead, ie increase pulse duration
        // only need to update each step
        int encoderDifference = encoder->getPosition() - encoder2->getPosition();
        int correction = encoderDifference * DUAL_MOTOR_CORRECTION;
        if (abs(correction) > (maxStepDelayDuration - stepDelayDuration) / 10){
          correction = ((maxStepDelayDuration - stepDelayDuration) / 10) * correction / abs(correction);
        }
        if (direction) {
          stepDelayDual1 = currentStepDelayDuration + (encoderDifference * DUAL_MOTOR_CORRECTION);
          stepDelayDual2 = currentStepDelayDuration - (encoderDifference * DUAL_MOTOR_CORRECTION);
        }
        else{
          stepDelayDual1 = currentStepDelayDuration - (encoderDifference * DUAL_MOTOR_CORRECTION);
          stepDelayDual2 = currentStepDelayDuration + (encoderDifference * DUAL_MOTOR_CORRECTION);
        }
      }
    }
  }
}

void StepperMotor::updateAcceleration(unsigned long elapsedMicros){
  accTimer += elapsedMicros;
  if (ACC_TICK < accTimer) {
    //accelerate 
    if(currentStepDelayDuration > stepDelayDuration){
      currentStepDelayDuration -= accelRate;
    }
    accTimer = 0;
  }
}

void StepperMotor::encoderControlledSteps(){
  if(steps < 1 && encoderControlledStep){
    long encoderPos = encoder->getPosition();
    //TODO consider overshoot
    if(abs(encoderPos - targetEncoderValue) > ENCODER_PRECISION){
      //recalculate steps
      steps = abs(encoderPos - targetEncoderValue) / ENC_STEP_CONV_FACTOR;
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
  digitalWrite(dirPin, dir);
  if (dualMotor){
    if (opposing)
      digitalWrite(dirPin2, !dir);
    else 
      digitalWrite(dirPin2, dir);
  }
}

void StepperMotor::setSpeed(int speed){ 
  this->speed = speed;
  stepDelayDuration = (long)1000000 / ((long) speed * (long) stepsPerMM);
}

void StepperMotor::setMinSpeed(int minSpeed){ 
  this->minSpeed = minSpeed;
  maxStepDelayDuration = (long)1000000 / ((long) minSpeed * (long) stepsPerMM);
}

