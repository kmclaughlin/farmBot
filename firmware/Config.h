#ifndef CONFIG_H
#define CONFIG_H

#define NUM_OF_MOTORS           3
#define NUM_OF_ENCODERS         4
// motor speeds in mm/s
#define X_MIN_SPEED             20
#define X_SPEED                 1
#define X_STEPS_PER_MM          100
#define X_ACCEL_RATE            10
#define Y_MIN_SPEED             20
#define Y_SPEED                 1
#define Y_STEPS_PER_MM          100
#define Y_ACCEL_RATE            10
#define Z_MIN_SPEED             20
#define Z_SPEED                 1
#define Z_STEPS_PER_MM          100
#define Z_ACCEL_RATE            10

#define MIN_STEP_PULSE          100
#define INTERRUPT_TIME          50
//time in millisecods to after which to disable motors after the last motor movement
#define MOTOR_TIMEOUT           10000 
//time in millisecods to halt movement/disable motors if should have but hasn't moved
#define STUCK_TIMEOUT           2000 
//encoder steps to have moved during timeout in order to not throw error  
#define STUCK_ENCODER_THRESHOLD  50   

#define INST_ARRAY_LEN          20

#define MOTOR_ENABLE_DELAY 100
#define ENCODER_PRECISION 2
#define DUAL_MOTOR_CORRECTION 10
#define ENC_STEP_CONV_FACTOR 1.76
#define BAUD_RATE 115200

#endif
