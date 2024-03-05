#include "msp.h"
#include "Bump.h"
#include "Reflectance.h"
#include "Clock.h"
#include "SysTickInts.h"
#include "CortexM.h"
#include "LaunchPad.h"
#include "TimerA1.h"
#include "Motor.h"
#include <stdbool.h>

#define WHITE     0x07
#define LBLUE     0x06
#define PURPLE    0x05
#define BLUE      0x04
#define YELLOW    0x03
#define GREEN     0x02
#define RED       0x01
#define OFF       0x00

volatile uint8_t reflectance_data, reflectance_posn, bump_data;
volatile int speed = 1500, backup_speed = 4000;
volatile int time1 = 50,
             time2 = 100,
             time3 = 150,
             time4 = 550,
             time_backup = 300;

/*
// Define states
typedef enum {
    // Possible States
    CENTER,
    SLIGHT_LEFT,
    SLIGHT_RIGHT,
    OFF_LEFT,
    OFF_RIGHT,
    WAY_OFF_LEFT,
    WAY_OFF_RIGHT,
    FINISH,
    LOST,
    NUM_STATES
} State;

// Define inputs
typedef enum {
    // Inputs to switch states
    CENTER_INPUT,
    SLIGHT_LEFT_INPUT,
    SLIGHT_RIGHT_INPUT,
    OFF_LEFT_INPUT,
    OFF_RIGHT_INPUT,
    WAY_OFF_LEFT_INPUT,
    WAY_OFF_RIGHT_INPUT,
    FINISH_INPUT,
    LOST_INPUT,
    NUM_INPUTS
} Input;

// Define the FSM table structure
typedef struct {
    State nextState;
    void (*action)(void);
} Transition;

// Define the FSM table as a 2D array
Transition fsmTable[NUM_STATES][NUM_INPUTS];

*/

void goStraight(void){
    Motor_Forward(speed,speed);
    Clock_Delay1ms(time3);
//    Motor_Stop();
}

void goLeft(void){ //very very slightly turns left
    Motor_Left(speed,speed);
    Clock_Delay1ms(time3);
//    Motor_Stop();
}

void goRight(void){ //very very slightly turns right
    Motor_Right(speed,speed);
    Clock_Delay1ms(time3);
//    Motor_Stop();
}

void goSlightLeft(void){ // turns robot slightly left
    Motor_Left(backup_speed,backup_speed);
    Clock_Delay1ms(time2);
    Motor_Stop();
}

void goSlightRight(void){ // turns robot slightly right
    Motor_Right(backup_speed,backup_speed);
    Clock_Delay1ms(time2);
    Motor_Stop();
}

void goHardLeft(void){ // 90 degree turn
    Motor_Left(backup_speed,backup_speed);
    Clock_Delay1ms(time4);
    Motor_Stop();
}

void goHardRight(void){ // 90 degree turn
    Motor_Right(backup_speed,backup_speed);
    Clock_Delay1ms(time4);
    Motor_Stop();
}

void goBackwards(void){
    Motor_Backward(backup_speed,backup_speed);
    Clock_Delay1ms(time_backup);
//    Motor_Stop();
}

void stop(void){
    Motor_Stop();
}

void Motor_Handler(uint8_t data, uint8_t ){
    switch(data) {
        case 0x00:
            Motor_Backward(backup_speed, backup_speed);
            Clock_Delay1ms(Spt->delay);
            break;
        case 0x01:
            Motor_Forward(speed, speed);
            Clock_Delay1ms(Spt->delay);
            break;
        case 0x02:
            Motor_Left(speed, speed);
            Clock_Delay1ms(Spt->delay);
            break;
        case 0x03:
            Motor_Right(speed, speed);
            Clock_Delay1ms(Spt->delay);
            Motor_Stop();
            break;
        case 0x04:
            Motor_Left(backup_speed, backup_speed);
            Clock_Delay1ms(Spt->delay);
            Motor_Stop()
            break;
        case 0x05:
            Motor_Right(backup_speed, backup_speed);
            Clock_Delay1ms(Spt->delay);
            break;
        case 0x06:
            Motor_Stop();
            break;
        default:
            Motor_Stop();
            printf("Invalid data value!\n");
            break;
}

/*
void initializeFSMTable(void) {
    // Lost state transitions
    fsmTable[LOST][LOST_INPUT].nextState = LOST;
    fsmTable[LOST][LOST_INPUT].action = &goStraight;

    fsmTable[LOST][CENTER_INPUT].nextState = CENTER;
    fsmTable[LOST][CENTER_INPUT].action = &goStraight;

    fsmTable[LOST][SLIGHT_LEFT_INPUT].nextState = SLIGHT_LEFT;
    fsmTable[LOST][SLIGHT_LEFT_INPUT].action = &goRight;

    fsmTable[LOST][SLIGHT_RIGHT_INPUT].nextState = SLIGHT_RIGHT;
    fsmTable[LOST][SLIGHT_RIGHT_INPUT].action = &goLeft;

    fsmTable[LOST][WAY_OFF_LEFT_INPUT].nextState = WAY_OFF_LEFT;
    fsmTable[LOST][WAY_OFF_LEFT_INPUT].action = &goLeft;

    fsmTable[LOST][WAY_OFF_RIGHT_INPUT].nextState = WAY_OFF_RIGHT;
    fsmTable[LOST][WAY_OFF_RIGHT_INPUT].action = &goRight;

    // Center state transitions
    fsmTable[CENTER][CENTER_INPUT].nextState = CENTER;
    fsmTable[CENTER][CENTER_INPUT].action = &goStraight;

    fsmTable[CENTER][SLIGHT_LEFT_INPUT].nextState = CENTER;
    fsmTable[CENTER][SLIGHT_LEFT_INPUT].action = &goRight;

    fsmTable[CENTER][SLIGHT_RIGHT_INPUT].nextState = CENTER;
    fsmTable[CENTER][SLIGHT_RIGHT_INPUT].action = &goLeft;

    // Slight left state transitions
    fsmTable[SLIGHT_LEFT][CENTER_INPUT].nextState = CENTER;
    fsmTable[SLIGHT_LEFT][CENTER_INPUT].action = &goStraight;

    fsmTable[SLIGHT_LEFT][OFF_LEFT_INPUT].nextState = SLIGHT_LEFT;
    fsmTable[SLIGHT_LEFT][OFF_LEFT_INPUT].action = &goRight;

    // Slight right state transitions
    fsmTable[SLIGHT_RIGHT][CENTER_INPUT].nextState = CENTER;
    fsmTable[SLIGHT_RIGHT][CENTER_INPUT].action = &goStraight;

    fsmTable[SLIGHT_RIGHT][OFF_RIGHT_INPUT].nextState = SLIGHT_RIGHT;
    fsmTable[SLIGHT_RIGHT][OFF_RIGHT_INPUT].action = &goLeft;

    // Off left state transitions
    fsmTable[OFF_LEFT][SLIGHT_LEFT_INPUT].nextState = SLIGHT_LEFT;
    fsmTable[OFF_LEFT][SLIGHT_LEFT_INPUT].action = &goRight;

    fsmTable[OFF_LEFT][WAY_OFF_LEFT_INPUT].nextState = OFF_LEFT;
    fsmTable[OFF_LEFT][WAY_OFF_LEFT_INPUT].action = &goRight;

    // Off right state transitions
    fsmTable[OFF_RIGHT][SLIGHT_RIGHT_INPUT].nextState = SLIGHT_RIGHT;
    fsmTable[OFF_RIGHT][SLIGHT_RIGHT_INPUT].action = &goLeft;

    fsmTable[OFF_RIGHT][WAY_OFF_RIGHT_INPUT].nextState = OFF_RIGHT;
    fsmTable[OFF_RIGHT][WAY_OFF_RIGHT_INPUT].action = &goLeft;

    // Way off left state transitions
    fsmTable[WAY_OFF_LEFT][SLIGHT_LEFT_INPUT].nextState = SLIGHT_LEFT;
    fsmTable[WAY_OFF_LEFT][SLIGHT_LEFT_INPUT].action = &goRight;

    // Way off right state transitions
    fsmTable[WAY_OFF_RIGHT][SLIGHT_RIGHT_INPUT].nextState = SLIGHT_RIGHT;
    fsmTable[WAY_OFF_RIGHT][SLIGHT_RIGHT_INPUT].action = &goLeft;

    // Finish state transitions (if needed)
    fsmTable[FINISH][FINISH_INPUT].nextState = FINISH;
    fsmTable[FINISH][FINISH_INPUT].action = &stop;

    fsmTable[WAY_OFF_RIGHT][FINISH_INPUT].nextState = FINISH;
    fsmTable[WAY_OFF_RIGHT][FINISH_INPUT].action = &stop;

    fsmTable[WAY_OFF_LEFT][FINISH_INPUT].nextState = FINISH;
    fsmTable[WAY_OFF_LEFT][FINISH_INPUT].action = &stop;

    fsmTable[OFF_RIGHT][FINISH_INPUT].nextState = FINISH;
    fsmTable[OFF_RIGHT][FINISH_INPUT].action = &stop;

    fsmTable[OFF_LEFT][FINISH_INPUT].nextState = FINISH;
    fsmTable[OFF_LEFT][FINISH_INPUT].action = &stop;

    fsmTable[SLIGHT_RIGHT][FINISH_INPUT].nextState = FINISH;
    fsmTable[SLIGHT_RIGHT][FINISH_INPUT].action = &stop;

    fsmTable[SLIGHT_LEFT][FINISH_INPUT].nextState = FINISH;
    fsmTable[SLIGHT_LEFT][FINISH_INPUT].action = &stop;

    fsmTable[CENTER][FINISH_INPUT].nextState = FINISH;
    fsmTable[CENTER][FINISH_INPUT].action = &stop;
}

// Process input function
void processInput(State *currentState, Input input) {
    Transition currentTransition = fsmTable[*currentState][input];
    *currentState = currentTransition.nextState;
    currentTransition.action();
}
*/


void SysTick_Handler(void){
    volatile static uint8_t count=0;

    // Handles reading the reflectance sensor
    if(count==0){
        Reflectance_Start();
    }
    else if (count==1) {
        reflectance_data =  Reflectance_End();
        bump_data = Bump_Read();
    }

    count++;
    if(count==10)count=0;
}


void bump_interrupt(void) {
    uint8_t bumpResult = Bump_Read();
    reflectance_data = Reflectance_Read(1000);
    reflectance_posn = Reflectance_Position(reflectance_data);
    if (bumpResult != 0x3F) {

        int backup_speed = 2500, backup_time = 300;
        Motor_Backward(backup_speed,backup_speed);
        Clock_Delay1ms(backup_time);
        Motor_Stop();

        int turn_speed = 3000, turn_time = 900;
        Motor_Backward(turn_speed,turn_speed);
        Clock_Delay1ms(turn_time);
        Motor_Stop();

        Motor_Forward(4000,4000);
        Clock_Delay1ms(500);
        Motor_Stop();

    }
    return;
}


void TimedPause(uint32_t time){
  Clock_Delay1ms(time);          // run for a while and stop
  Motor_Stop();
  while(LaunchPad_Input()==0);  // wait for touch
  while(LaunchPad_Input());     // wait for release
}


//================================================================

// Linked data structure
struct State {
  uint32_t out;                // 3-bit output (only 0-5 used)
  uint32_t delay;              // time to delay in 1ms
  const struct State *next[8]; // Next if 3-bit input is 0-7
};
typedef const struct State State_t;

#define Center        &fsm[0]
#define Left          &fsm[1]
#define Right         &fsm[2]
#define SharpLeft     &fsm[3]
#define SharpRight    &fsm[4]
#define GapJump       &fsm[5]
// student starter code

/*
3 bit Input:
Bit 0: Far Left Sensor
Bit 1: Far Right Sensor
Bit 2: Transition in and out of only 0’s / 2+ bit difference
 */

/*
3 bit Output:
0x00 = straight
0x01 = slightly left
0x02 = slightly right
0x03 = hard left
0x04 = hard right
0x05 = backwards
*/

// Inputs:               000      010       100        110           001          011          101          111
State_t fsm[5]={
  {0x01, time3,       { Center,   Right,    Left,     Center,      GapJump,     SharpRight,  SharpLeft,  SharpRight}},  // Center
  {0x02, time3,       { Center,   Right,    Left,     SharpRight,  SharpLeft,   Right,       SharpLeft,  SharpRight}},  // Left
  {0x03, time3,       { Center,   Right,    Left,     SharpLeft,   SharpRight,  SharpRight,  Left,       SharpLeft}}   // Right
  {0x04, time4,       { Center,   Right,    GapJump,  Right,       Center,      Right,       Left,       Right}}   // Sharp Left
  {0x05, time4,       { Center,   GapJump,  Left,     Left,        Center,      Right,       Left,       Left}}   // Sharp Right
  {0x01, time_backup, { GapJump,  Right,    Left,     SharpRight,  Center,      Right,       Left,       SharpRight}}   // Gap Jump
};



State_t *Spt;  // pointer to the current state
uint32_t Input;
uint32_t Output;

//================================================================


void main(void){
    // Initialization
    initializeFSMTable();
    Clock_Init48MHz();
    LaunchPad_Init(); // built-in switches and LEDs
    TimerA1_Init(&bump_interrupt,48000);
    EnableInterrupts();
    Bump_Init();      // bump switches
    Motor_Init();
    Reflectance_Init();
    SysTick_Init(48000,2);  // set up SysTick for 1000 Hz interrupts

    TimedPause(1000);

    Spt = Center;

    while(1){
        WaitForInterrupt();
        //Perform State Actions
        Output = Spt->out;            // Set LED to output from FSM
        LaunchPad_Output(Output);     // LED Output for debugging
        Motor_Handler(output);

        //Read Data
        reflectance_data = Reflectance_Read(1000);      // read input data
        new input = Transform_Input(reflectance_data);      // turn 8 bit data into 3 bit input
        reflectance_posn = Reflectance_Position(reflectance_data);      // calculate position
        Spt = Spt->next[Input];       // next depends on input and state


    }
}




/*
void main1(void){

    // Initialization
    initializeFSMTable();
    Clock_Init48MHz();
    LaunchPad_Init(); // built-in switches and LEDs
    TimerA1_Init(&bump_interrupt,48000);
    EnableInterrupts();
    Bump_Init();      // bump switches
    Motor_Init();
    Reflectance_Init();
    SysTick_Init(48000,2);  // set up SysTick for 1000 Hz interrupts

    TimedPause(1000);

    State currentState = CENTER;
    bool FSM = true;

    while(1){
        WaitForInterrupt();
        reflectance_data = Reflectance_Read(1000);
        reflectance_posn = Reflectance_Position(reflectance_data);
        Clock_Delay1ms(10);

//        if (reflectance_data == 0b11111111 && reflectance_posn == 0) { // 2nd t-join
        if (reflectance_data == 0b11111111 && reflectance_posn == 0) { // 2nd t-join
            if (!FSM){
                Motor_Right(2000,2000);
                Clock_Delay1ms(1500);
                Motor_Stop();
                Motor_Forward(4000,4000);
                Clock_Delay1ms(500);
                Motor_Stop();
            } else{
                processInput(&currentState, FINISH_INPUT);
            }
        }
//        else if (reflectance_posn > -47 && reflectance_posn < 47) { //center
        else if (reflectance_posn > -47 && reflectance_posn < 47) { //center
            if (!FSM){
                Motor_Forward(speed,speed);
                Clock_Delay1ms(time3);
                Motor_Stop();
            } else{
                processInput(&currentState, CENTER_INPUT);
            }
//        } else if (reflectance_posn <= -47 && reflectance_posn > -142) { //slightly off to the left
        } else if (reflectance_posn <= -47 && reflectance_posn > -142) { //slightly off to the left
            if (!FSM){
                Motor_Left(speed,speed);
                Clock_Delay1ms(time1);
                Motor_Stop();
            } else{
                processInput(&currentState, SLIGHT_LEFT_INPUT);
            }
//        } else if (reflectance_posn >= 47 && reflectance_posn <142) { //slightly off to the right
        } else if (reflectance_posn >= 47 && reflectance_posn <142) { //slightly off to the right
            if (!FSM){
                Motor_Right(speed,speed);
                Clock_Delay1ms(time1);
                Motor_Stop();
            } else{
                processInput(&currentState, SLIGHT_RIGHT_INPUT);
            }
//        } else if (reflectance_posn <= -142 && reflectance_posn >-237) { //off to the left
        } else if (reflectance_posn <= -142 && reflectance_posn >-237) { //off to the left
            if (!FSM){
                Motor_Left(speed,speed);
                Clock_Delay1ms(time2);
                Motor_Stop();
            } else{
                processInput(&currentState, OFF_LEFT_INPUT);
            }
//        } else if (reflectance_posn >= 142 && reflectance_posn < 237) { // off to the right
        } else if (reflectance_posn >= 142 && reflectance_posn < 237) { // off to the right
            if (!FSM){
                Motor_Right(speed,speed);
                Clock_Delay1ms(time2);
                Motor_Stop();
            } else{
                processInput(&currentState, OFF_RIGHT_INPUT);
            }
//        } else if (reflectance_posn <= -237 && reflectance_posn > -332) { // way off left
        } else if (reflectance_posn <= -237 && reflectance_posn > -332) { // way off left
            if (!FSM){
                Motor_Left(speed,speed);
                Clock_Delay1ms(time3);
                Motor_Stop();
            } else{
                processInput(&currentState, WAY_OFF_LEFT_INPUT);
            }
//        } else if (reflectance_posn >= 237 && reflectance_posn < 332) { // way off right
        } else if (reflectance_posn >= 237 && reflectance_posn < 332) { // way off right
            if (!FSM){
                Motor_Right(speed,speed);
                Clock_Delay1ms(time3);
                Motor_Stop();
            } else{
                processInput(&currentState, WAY_OFF_RIGHT_INPUT);
            }
//        } else if (reflectance_data == 0b00000000 && reflectance_posn == 333) { // goal
        } else if (reflectance_data == 0b00000000 && reflectance_posn == 333) { // goal
            if (!FSM){
                Motor_Stop();
            } else{
                processInput(&currentState, FINISH_INPUT);
            }
        } else {
            if (!FSM){
                Motor_Backward(backup_speed,backup_speed);
                Clock_Delay1ms(time_backup);
                Motor_Stop();
            } else{
                processInput(&currentState, LOST_INPUT);
            }
        }
    }
}
*/
