#include "StepperMotor.h"
#include "pins.h"
#include "Encoder.h"
#include "TimerOne.h"
#include "Config.h"

StepperMotor* motors[NUM_OF_MOTORS];
Encoder* encoders[NUM_OF_ENCODERS];
int instIndex = 0;
int motorIndex = 0;
char instruction[INST_ARRAY_LEN];
bool reportEncoders = false;
bool motorsEnabled = false;
unsigned long motorStuckTimeoutTimer = 0;
unsigned long motorTimeoutTimer = 0;
bool allowMovement = true;
bool selfPreservation = true;
bool verboseResponses = false;
bool lightState = false;
bool waterState = false;
bool vacuumState = false;

void processInstruction(char *instruction);

void enableMotors(bool value) {
  motorsEnabled = value;
  for (int i = 0; i < NUM_OF_MOTORS; i++)
    motors[i]->enable(value);
  //start motor move timeout 
  motorStuckTimeoutTimer = millis();
}

bool interruptBusy = false;
unsigned long lastInterruptTime = micros();
void interrupt(void){
  if(!interruptBusy){
    interruptBusy = true;
    unsigned long currentMicros = micros();
    unsigned long interruptTime = currentMicros - lastInterruptTime;
    lastInterruptTime = currentMicros;
    //handle motor movement by interrupt
    //step motors
    for (int i = 0; i < NUM_OF_MOTORS; i++){
      motors[i]->step(interruptTime);
    }
    //process encoders
    for (int i = 0; i < NUM_OF_ENCODERS; i++){
      encoders[i]->process();
    }
    interruptBusy = false;
  }
}

void setup() {
  //set up peripherals
  pinMode(LIGHTING_PIN, OUTPUT);
  pinMode(WATER_PIN, OUTPUT);
  pinMode(VACUUM_PIN, OUTPUT);
  // start with all off
  digitalWrite(LIGHTING_PIN, LOW);
  digitalWrite(WATER_PIN, LOW);
  digitalWrite(VACUUM_PIN, LOW);
  
  //Serial.begin(115200);
  Serial.begin(9600);
  
  //initialise encoders
  encoders[0] = new Encoder(X_ENCDR_A, X_ENCDR_B, ENC_X_A_BYTE, ENC_X_B_BYTE, 
                            &ENC_X_A_PORT, &ENC_X_B_PORT, false);
  encoders[1] = new Encoder(X2_ENCDR_A, X2_ENCDR_B, ENC_X2_A_BYTE, ENC_X2_B_BYTE, 
                            &ENC_X2_A_PORT, &ENC_X2_B_PORT,  false);
  encoders[2] = new Encoder(Y_ENCDR_A, Y_ENCDR_B, ENC_Y_A_BYTE, ENC_Y_B_BYTE, 
                            &ENC_Y_A_PORT, &ENC_Y_B_PORT, false);
  encoders[3] = new Encoder(Z_ENCDR_A, Z_ENCDR_B, ENC_Z_A_BYTE, ENC_Z_B_BYTE, 
                            &ENC_Z_A_PORT, &ENC_Z_B_PORT, false);
                            
  //initialise Motors
  motors[0] = new StepperMotor(X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, X2_STEP_PIN, X2_DIR_PIN, X2_ENABLE_PIN, true, MAX_STEP_PULSE);
  motors[1] = new StepperMotor(Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, 1000);
  motors[2] = new StepperMotor(Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, 1000);

  motors[0]->attachEncoder(encoders[0], encoders[1]);
  motors[1]->attachEncoder(encoders[2]);
  motors[2]->attachEncoder(encoders[3]);
  
  //set timer interrupt for motor and encoder control
  Timer1.attachInterrupt(interrupt);
  Timer1.initialize(INTERRUPT_TIME);
  Timer1.start();

  if(verboseResponses) {
    Serial.println("Startup complete");
  }
}

void loop() {
  readSerial();
  disableMotorsTimeout();
  checkMotorsStuck();
}

void readSerial(){
  if (Serial.available() > 0)
  {
    char nextChar = Serial.read();
    //if char is new line last instruction complete, process instruction
    if(nextChar == ';'){
      if(instIndex > 0) {
        //make sure rest of instruction is cleared
        for (int i = instIndex; i < INST_ARRAY_LEN; i++)
          instruction[i] = NULL;
        //instruction[instIndex] = '\n';
        //send instruction for processing
        processInstruction(instruction);
        instIndex = 0;
      }
    }
    //add to instruction string
    else {
      if(instIndex >= INST_ARRAY_LEN)
        Serial.println("ERROR - loop() instruction parser: Instruction index out of bounds.");
      else{
        instruction[instIndex] = nextChar;
        instIndex++;
      }
    }
  }
}

void processInstruction(char *input){
  //check first byte
  switch(input[0]){
    case 'i':
      Serial.println("Farmbot v0.001");
      Serial.println("Commands:");
      Serial.println("m #1 #2: move motor #1 distance #2 by stepper motor steps");
      Serial.println("n #1 #2: move motor #1 distance #2 by encoder value");
      Serial.println("l1/0: lights on/off");
      Serial.println("w1/0: water on/off");
      Serial.println("v1/0: vacuum on/off");
      Serial.println("e: get current encoder values");
      Serial.println("r: reset ecnoder values to 0");
      Serial.println("c: return motor enabled status");
      Serial.println("d: disable all motors");
      Serial.println("x: begin/end continuous encoder value stream");
      Serial.println("b: allow movement after jam");
      Serial.println("p1/0: self preservation on/off (not recommended to turn off)");
      Serial.println("s: return total farmbot state");
      break;
    case 'm':
      if (allowMovement) {
        int motor = input[2] - 48;
        int steps = atol(input+4);
        //enable motors for movement
        enableMotors(true);
        motors[motor]->setSteps(abs(steps), steps > 0);
        if(verboseResponses) {
          Serial.print("motor steps: ");
          Serial.print(motor);
          Serial.print(" ");
          Serial.println(steps);
        }
      }
      else
        Serial.println("Error: cannot move. Axis jammed. Send manual reset command 'b'");
      break;
    case 'n':
      if (allowMovement) {
        int motor = input[2] - 48;
        int steps = atol(input+4);
        //enable motors for movement
        enableMotors(true);
        motors[motor]->moveEncoderSteps(steps);
        if(verboseResponses) {
          Serial.print("motor encoder steps: ");
          Serial.print(motor);
          Serial.print(" ");
          Serial.println(steps);
          Serial.print("eVal: ");
          Serial.print(motors[motor]->getEncoder()->getPosition());
          Serial.print(" target: ");
          Serial.println(motors[motor]->getTargetEncoderValue());
        }
      }
      else
        Serial.println("Error: cannot move. Axis jammed. Send manual reset command 'b'");
      break;
    case 'l':
      if(input[1] == '1'){
        digitalWrite(LIGHTING_PIN, HIGH);
        lightState = true;
        if(verboseResponses) {
          Serial.println("Lights: on");
        }
      }
      else if(input[1] == '0'){
        digitalWrite(LIGHTING_PIN, LOW);
        lightState = false;
        if(verboseResponses) {
          Serial.println("Lights: off");
        }
      }
      break;
    case 'w':
      if(input[1] == '1'){
        digitalWrite(WATER_PIN, HIGH);
        waterState = true;
        if(verboseResponses) {
          Serial.println("Water: on");
        }
      }
      else if(input[1] == '0'){
        digitalWrite(WATER_PIN, LOW);
        waterState = false;
        if(verboseResponses) {
          Serial.println("Water: off");
        }
      }
      break;
    case 'v':
      if(input[1] == '1'){
        digitalWrite(VACUUM_PIN, HIGH);
        vacuumState = true;
        if(verboseResponses) {
          Serial.println("Vacuum: on");
        }
      }
      else if(input[1] == '0'){
        digitalWrite(VACUUM_PIN, LOW);
        vacuumState = false;
        if(verboseResponses) {
          Serial.println("Vacuum: off");
        }
      }
      break;
    case 'e':
      if(verboseResponses) {
        for (int i = 0; i < NUM_OF_ENCODERS; i++){
          Serial.print(encoders[i]->getPosition());
          Serial.print(" ");
        }
        Serial.println();
      }
      break;
    case 'r':
      for (int i = 0; i < NUM_OF_ENCODERS; i++){
        encoders[i]->setPosition(0);
      }
      if(verboseResponses) {
        Serial.println("Encoders reset");
      }
      break;
    case 'c':
      for (int i = 0; i < NUM_OF_MOTORS; i++){
        Serial.print(motors[i]->isEnabled());
        Serial.print(" ");
      }
      Serial.println("");
      break;
    case 'x':
      reportEncoders = !reportEncoders;
      break;
    case 'd':
      if(input[1] == '1'){
        enableMotors(true);
        if(verboseResponses) {
          Serial.println("Motors enabled");
        }
      }
      else if(input[1] == '0'){
        enableMotors(false);
        if(verboseResponses) {
          Serial.println("Motors disabled");
        }
      }
      break;
    case 'b':
      allowMovement = true;
      Serial.println("Movement enabled");
      break;
    case 'p':
      if(input[1] == '1'){
        selfPreservation = true;
        Serial.println("selfPreservation: on");
      }
      else if(input[1] == '0'){
        selfPreservation = false;
        Serial.println("selfPreservation: off");
      }
      break;
    case 's':
      if (verboseResponses) {
        Serial.print("x: ");
        Serial.print(motors[0]->getEncoder()->getPosition());
        Serial.print(", ");
        Serial.print(encoders[1]->getPosition());
        Serial.print("  y: ");
        Serial.print(motors[1]->getEncoder()->getPosition());
        Serial.print("  z: ");
        Serial.println(motors[2]->getEncoder()->getPosition());
        Serial.print("x en: ");
        Serial.print(motors[0]->isEnabled());
        Serial.print("  y en: ");
        Serial.print(motors[1]->isEnabled());
        Serial.print("  z en: ");
        Serial.println(motors[2]->isEnabled());
        Serial.print("lights: ");
        Serial.print(lightState);
        Serial.print("  water: ");
        Serial.print(waterState);
        Serial.print("  vacuum: ");
        Serial.println(vacuumState);
      }
      break;
    case 'a':
      if(input[1] == '1'){
        verboseResponses = true;
        Serial.println("Verbose Responses: on");
      }
      else if(input[1] == '0'){
        verboseResponses = false;
      }
      break;
    default:
      Serial.println("Error: invalid command");
  }
  if(!verboseResponses){
    Serial.println(getState());
  }
}

String getState(){
  //Format
  //xxxxxyyyyyzzzzzLE
  //x, y, z - 5 digits of x, y, z encoder absolute position 
  //S - binary encoded x, y, z, encoder sign bit
  //L - binary encoded light water vacuum state
  //E - binary encoded motor enabled x, y, z
  String state = "";
  //get motor encoder positions and set to len 5
  double signBit = 0;
  for (int i = 0; i < NUM_OF_MOTORS; i++){
    int encoderPos = motors[i]->getEncoder()->getPosition();
    if (encoderPos < 0)
      signBit += pow(2, (NUM_OF_MOTORS - i) - 1);
    String encoderState = String(abs(encoderPos));
    for (int j = 0; j < 5 - encoderState.length(); j++){
      state += "0";
    }
    state += encoderState;
  }
  state += String(signBit, 0).charAt(1);
  // get light, water vacuum state and encode
  int LWV = 0;
  if (lightState)
    LWV += 4;
  if (waterState)
    LWV += 2;
  if (vacuumState)
    LWV += 1;
  state += String(LWV);

  //get motor enabled state and encode
  double motorEnabledState = 0;
  for (int i = 0; i < NUM_OF_MOTORS; i++){
    if(motors[i]->isEnabled()){
      motorEnabledState += pow(2, ((NUM_OF_MOTORS - i) - 1));
    }
  }
  state += String(motorEnabledState, 0).charAt(1);
  
  return state;
}

void disableMotorsTimeout(){
  //if motors enabled check if any are moving
  if (motorsEnabled) {
    //check if any motors are moving
    bool anyMotorsMoving = false;
    for (int i = 0; i < NUM_OF_MOTORS; i++){
      anyMotorsMoving = anyMotorsMoving || motors[i]->isMoving();
    }
    //if motors are moving reset the timer
    if (anyMotorsMoving)
      motorTimeoutTimer = millis();
    //if no motors are moving and timer expires, stop the motors
    else if (MOTOR_TIMEOUT < (millis() - motorTimeoutTimer)){
      enableMotors(false);
    }
  }
}

void checkMotorsStuck(){
  //check motor timeout
  if(selfPreservation && motorsEnabled && ((millis() - motorStuckTimeoutTimer) > STUCK_TIMEOUT)){
    //if encoder distance > threshold
    bool noMotorsStuck = true;
    bool isMotorStuck[NUM_OF_MOTORS];
    for (int i = 0; i < NUM_OF_MOTORS; i++)
    //if the motor is trying to move
      if (motors[i]->isMoving()) {
        //check if it has moved
        isMotorStuck[i] = motors[i]->getEncoder()->movementCheck() < STUCK_ENCODER_THRESHOLD;
        noMotorsStuck = noMotorsStuck && !isMotorStuck[i];
      }
      else 
        isMotorStuck[i] = false;
        
    if (noMotorsStuck){
      motorStuckTimeoutTimer = millis();
      for (int i = 0; i < NUM_OF_ENCODERS; i++)
        encoders[i]->resetMovementCheck();
    }
    else {
      //disable motors
      enableMotors(false);
      
      for (int i = 0; i < NUM_OF_ENCODERS; i++)
        encoders[i]->resetMovementCheck();
        
      //set motor movement to 0
      for (int i = 0; i < NUM_OF_MOTORS; i++)
        motors[i]->moveEncoderSteps(0);

      if(verboseResponses) {
        Serial.print("Error: Axis stuck ");
        for (int i = 0; i < NUM_OF_MOTORS; i++) {
          Serial.print(i);
          Serial.print(": ");
          Serial.print(isMotorStuck[i]);
          Serial.print("  ");
        }
        Serial.println();
      }
      else {
        String errorMsg = "E01 ";
        for (int i = 0; i < NUM_OF_MOTORS; i++) {
          if (isMotorStuck[i])
            errorMsg += "1";
          else
            errorMsg += "0";
        }
        Serial.println(errorMsg);
      }
      allowMovement = false;
    }
  }
}

