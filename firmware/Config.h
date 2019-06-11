#ifndef CONFIG_H
#define CONFIG_H

#define NUM_OF_MOTORS           3
#define NUM_OF_ENCODERS         4
#define MAX_STEP_PULSE          2000
#define MIN_STEP_PULSE          100
#define INTERRUPT_TIME          50
//time in millisecods to after which to disable motors after the last motor movement
#define MOTOR_TIMEOUT           10000 
//time in millisecods to halt movement/disable motors if should have but hasn't moved
#define STUCK_TIMEOUT           2000 
//encoder steps to have moved during timeout in order to not throw error  
#define STUCK_ENCODER_THRESHOLD  50   

#define INST_ARRAY_LEN          20

#endif
