#ifndef CONFIG_H
#define CONFIG_H

#define NUM_OF_MOTORS           3
#define NUM_OF_ENCODERS         4
// motor speeds in mm/s
#define X_MIN_SPEED             10
#define X_SPEED                 80
#define X_STEPS_PER_MM          5 // 2710mm full range movement 24340 enc steps expected
#define X_ACCEL_RATE            100
#define Y_MIN_SPEED             10
#define Y_SPEED                 100
#define Y_STEPS_PER_MM          5 // 5963 steps /1195mm full range movement 10733 enc steps
#define Y_ACCEL_RATE            100
#define Z_MIN_SPEED             10
#define Z_SPEED                 80
#define Z_STEPS_PER_MM          5 // 508mm full range movement
#define Z_ACCEL_RATE            100

#define INTERRUPT_TIME          50
//time in millisecods to after which to disable motors after the last motor movement
#define MOTOR_TIMEOUT           10000 
//time in millisecods to halt movement/disable motors if should have but hasn't moved
#define STUCK_TIMEOUT           200 
//encoder steps to have moved during timeout in order to not throw error 
//with STUCK_TIMEOUT gives minimum speed of 1 mm/s, should travel faster than this
#define STUCK_ENCODER_THRESHOLD  10   

#define INST_ARRAY_LEN          20

#define MOTOR_ENABLE_DELAY 100
#define ENCODER_PRECISION 5 // number of enc steps +/- of desired movement
#define DUAL_MOTOR_CORRECTION 15
#define ENC_STEP_CONV_FACTOR 1.8
#define BAUD_RATE 115200
#define ACC_TICK 1000


#endif
