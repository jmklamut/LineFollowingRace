#include "msp.h"
#include "Bump.h"
#include "Reflectance.h"
#include "Clock.h"
#include "SysTickInts.h"
#include "CortexM.h"
#include "LaunchPad.h"
#include "TimerA1.h"
#include "Motor.h"

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
             time_backup = 300;

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
    NUM_INPUTS
} Input;

// Define the FSM table structure
typedef struct {
    State nextState;
    void (*action)(void);
} Transition;

// Define the FSM table as a 2D array
Transition fsmTable[NUM_STATES][NUM_INPUTS];

void goStraight(){
    Motor_Forward(speed,speed);
    Clock_Delay1ms(time3);
    Motor_Stop();
}

void goLeft(){
    Motor_Left(speed,speed);
    Clock_Delay1ms(time3);
    Motor_Stop();
}

void goRight(){
    Motor_Right(speed,speed);
    Clock_Delay1ms(time2);
    Motor_Stop();
}

void goBackwards(){
    Motor_Backward(backup_speed,backup_speed);
    Clock_Delay1ms(time_backup);
    Motor_Stop();
}

void stop(){
    Motor_Stop();
}

void initializeFSMTable() {
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
    if (bumpResult != 0x00) {

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

void main(void){

    // Initialization
//    initializeFSMTable();
    Clock_Init48MHz();
    LaunchPad_Init(); // built-in switches and LEDs
    TimerA1_Init(&bump_interrupt,50000);  // 10 Hz
    EnableInterrupts();
    Bump_Init();      // bump switches
    Motor_Init();
    Reflectance_Init();
    SysTick_Init(48000,2);  // set up SysTick for 1000 Hz interrupts

    TimedPause(1000);

//    State currentState = CENTER;

    while(1){
        reflectance_data = Reflectance_Read(1000);
        reflectance_posn = Reflectance_Position(reflectance_data);
        Clock_Delay1ms(10);

        if (reflectance_data == 0b11111111 && reflectance_posn == 0) { // 2nd t-join
            Motor_Right(2000,2000);
            Clock_Delay1ms(1500);
            Motor_Stop();
            Motor_Forward(4000,4000);
            Clock_Delay1ms(500);
            Motor_Stop();
//            processInput(&currentState, FINISH_INPUT);
        }
        else if (reflectance_posn > -47 && reflectance_posn < 47) { //center
            Motor_Forward(speed,speed);
            Clock_Delay1ms(time3);
            Motor_Stop();
            //break;
//            processInput(&currentState, CENTER_INPUT);
        } else if (reflectance_posn <= -47 && reflectance_posn > -142) { //slightly off to the left
            Motor_Left(speed,speed);
            Clock_Delay1ms(time1);
            Motor_Stop();
//            processInput(&currentState, SLIGHT_LEFT_INPUT);
        } else if (reflectance_posn >= 47 && reflectance_posn <142) { //slightly off to the right
            Motor_Right(speed,speed);
            Clock_Delay1ms(time1);
            Motor_Stop();
//            processInput(&currentState, SLIGHT_RIGHT_INPUT);
        } else if (reflectance_posn <= -142 && reflectance_posn >-237) { //off to the left
            Motor_Left(speed,speed);
            Clock_Delay1ms(time2);
            Motor_Stop();
//            processInput(&currentState, OFF_LEFT_INPUT);
        } else if (reflectance_posn >= 142 && reflectance_posn < 237) { // off to the right
            Motor_Right(speed,speed);
            Clock_Delay1ms(time2);
            Motor_Stop();
//            processInput(&currentState, OFF_RIGHT_INPUT);
        } else if (reflectance_posn <= -237 && reflectance_posn > -332) { // way off left
            Motor_Left(speed,speed);
            Clock_Delay1ms(time3);
            Motor_Stop();
//            processInput(&currentState, WAY_OFF_LEFT_INPUT);
        } else if (reflectance_posn >= 237 && reflectance_posn < 332) { // way off right
            Motor_Right(speed,speed);
            Clock_Delay1ms(time3);
            Motor_Stop();
//            processInput(&currentState, WAY_OFF_RIGHT_INPUT);
        } else if (reflectance_data == 0b00000000 && reflectance_posn == 333) { // goal
            Motor_Stop();
            //break;
//            processInput(currentState, FINISH_INPUT);
        } else {
            Motor_Backward(backup_speed,backup_speed);
            Clock_Delay1ms(time_backup);
            Motor_Stop();
//            processInput(currentState, FINISH_INPUT);
        }
    }
}
