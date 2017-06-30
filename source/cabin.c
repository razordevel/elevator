#define __AVR_ATmega64__
#include <Arduino.h>


#ifdef DEBUG_WIRE
#include <Wire.h>
#endif

// Generic constants
#define LEVEL_NUMBER 3 // Amount of the levels, corresponds with levels 0, 1 and 2.

// Door related constants
#define DOOR_SPEED            2000  // Milliseconds for full open or close movement.
#define DOOR_CLOSED_POSITION     0  // Position value for a closed door.
#define DOOR_OPEN_POSITION     100  // Position value for an open door.
#define DOOR_WAIT_TIME        4000  // Milliseconds the doors will wait in the open position.
#define DOOR_LEFT_OPEN          50  // TODO: Check this value!
#define DOOR_LEFT_CLOSED       200  // TODO: Check this value!
#define DOOR_RIGHT_OPEN        200  // TODO: Check this value!
#define DOOR_RIGHT_CLOSED       50  // TODO: Check this value!

// Motor related constants
#define FAST_MOTOR_SPEED       255
#define SLOW_MOTOR_SPEED       128

// register interface constants
#define I2C_BUFFER 5                // 1 address byte and 4 byte to set

// Pin constants
#define PIN_O_MOTOR_ENABLE       2  // port pin 3
#define PIN_O_MOTOR_UP           3  // port pin 4
#define PIN_I_SAFETY_UP         52  // port pin 5
#define PIN_O_MOTOR_DOWN         4  // port pin 6
#define PIN_I_SAFETY_DOWN       50  // port pin 7
#define PIN_I_TEMPERATURE       11  // port pin 10
#define PIN_I_ENCODER_A         -1  // port pin 11
#define PIN_I_ENCODER_B         -1  // port pin 12
#define PIN_I_LEVEL_1           -1  // port pin 13
#define PIN_I_LEVEL_2           -1  // port pin 14
#define PIN_I_LEVEL_3           -1  // port pin 15
#define PIN_O_DOOR_LEFT          5  // port pin 16
#define PIN_O_DOOR_RIGHT         6  // port pin 17
#define PIN_I_LEVEL_BUTTON_1    -1  // port pin 18
#define PIN_O_LEVEL_LIGHT_1      7  // port pin 19
#define PIN_I_LEVEL_BUTTON_2    -1  // port pin 20
#define PIN_O_LEVEL_LIGHT_2      8  // port pin 21
#define PIN_I_LEVEL_BUTTON_3    -1  // port pin 22
#define PIN_O_LEVEL_LIGHT_3      9  // port pin 23
#define PIN_O_CABIN_LIGHT       10  // port pin 24

// ------------------------------
// Enum definitions
// ------------------------------

// State of the elivator
enum OperationState {
  init_state, sleep_state, move_state, opendoors_state, wait_state, closedoors_state, maintenance_state, testc_state
};

enum PositionState {
  unknown, far_below, close_below, reached, close_above, far_above
};

enum MotorSpeed {
  stopped, slow, fast
};

enum MotorDirection {
  up, down
};

enum LightMode {
  off, on, flashing
};

enum DoorState {
  open, closed, opening, closing
};

// Funktonsprototype
void stopCabinMotor(void);
void testc(void);
void setState(enum OperationState newState);
void setOutsideLevelStates(int level);
void transferInputs(void);
int findTargetLevel(void);
void moveCabin(void);
void openDoors(void);
void closeDoors(void);
void moveDoors(int fromPosition, int toPosition);
void wait(void);
void maintenance(void);
void initialize(void);
void sleep(void);

//Description
//Re-maps a number from one range to another. That is, a value of fromLow would get mapped to toLow, 
//a value of fromHigh to toHigh, values in-between to values in-between, etc. 
//Parameters
//
//value: the number to map
//fromLow: the lower bound of the value's current range
//fromHigh: the upper bound of the value's current range
//toLow: the lower bound of the value's target range
//toHigh: the upper bound of the value's target range
//Returns
//The mapped value.
// Code:
//   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
extern long map(long x, long in_min, long in_max, long out_min, long out_max);

// ------------------------------
// Global variables
// ------------------------------

// State handling
enum OperationState state = init_state; // Current state of the state machine
enum OperationState last_state = init_state; // Previous state of the state machine in the last cycle
long state_time = 0;
long state_cycle = 0; // Holds the number of times the current state is iterated.

// Position handling
int level_position = -1;
int level_target = -1;
enum PositionState level_position_state[LEVEL_NUMBER];
int last_blocked_level = -1;
boolean button_state[LEVEL_NUMBER];
boolean safetyUp = false; // State of the safety switch above the highest floor
boolean safetyDown = false; // State of the safety switch below the lowest floor

// Temperature variable
float motorTemperature = -1;

// Global variables for door movement
long door_start_time = 0; // Moment of door movement begin.
int door_position = 0; // Holds door position as value between DOOR_CLOSED_POSITION and DOOR_OPEN_POSITION.

// Global variables for encoder interpretion
int encoder_value = 0;
long encoder_time = 0;
long encoder_ticks = 0;
float encoder_speed = 0;
boolean encoder_overspeed = false;

// Global variable for motor control
enum MotorDirection motor_direction;
enum MotorSpeed motor_speed;

// Global variable for register interface
int nbr_of_received_bytes = 0;
int received_values[I2C_BUFFER];
int request_queue[100];
int queue_start = 0;
int queue_end = 0;

void setButtonLight(int level, enum LightMode light);
// ------------------------------

// Write functions for outputs
void setDoorPositions(int position) {
  int left = map(position, DOOR_CLOSED_POSITION, DOOR_OPEN_POSITION, DOOR_LEFT_CLOSED, DOOR_LEFT_OPEN);
  int right = map(position, DOOR_CLOSED_POSITION, DOOR_OPEN_POSITION, DOOR_RIGHT_CLOSED, DOOR_RIGHT_OPEN);
  analogWrite(PIN_O_DOOR_LEFT, left);
  analogWrite(PIN_O_DOOR_RIGHT, right);
}

void moveCabinMotor(enum MotorSpeed speed, enum MotorDirection direction) {
  int speedValue;
  motor_direction = direction;
  motor_speed = speed;
  
  if (speed == stopped) {
    stopCabinMotor();
    return;
  }
  speedValue = speed == fast ? FAST_MOTOR_SPEED : SLOW_MOTOR_SPEED;
  digitalWrite(PIN_O_MOTOR_ENABLE, HIGH);
  if (direction == up) {
    analogWrite(PIN_O_MOTOR_UP, speedValue);
    digitalWrite(PIN_O_MOTOR_DOWN, LOW);
  } else {
    analogWrite(PIN_O_MOTOR_DOWN, speedValue);
    digitalWrite(PIN_O_MOTOR_UP, LOW);
  }
}

void stopCabinMotor(void) {
  motor_direction = down;
  motor_speed = stopped;
  
  digitalWrite(PIN_O_MOTOR_ENABLE, LOW);
  digitalWrite(PIN_O_MOTOR_UP, LOW);
  digitalWrite(PIN_O_MOTOR_DOWN, LOW);
}

void setCabinLight(enum LightMode light) {
  if (light == on) {
    digitalWrite(PIN_O_CABIN_LIGHT, HIGH);
  } else {
    digitalWrite(PIN_O_CABIN_LIGHT, LOW);
  }
}

// ------------------------------


void moveCabin(void) {
  enum PositionState p;
  long time = millis(); // The current time in millis

  // Check if we just entered the current door movement state
  if (state_cycle == 1) {
    level_target = findTargetLevel();
  }

  p = level_position_state[level_target];
  if (p == far_above) {
    moveCabinMotor(fast, down);
  } else if (p == close_above) {
    moveCabinMotor(slow, down);
  } else if (p == far_below) {
    moveCabinMotor(fast, up);
  } else if (p == close_below) {
    moveCabinMotor(slow, up);
  } else {
    stopCabinMotor();

    setState(opendoors_state);
  }

  // TODO: check safety switch values
  // TODO: check movement with encoder!
}

void openDoors() {
  if (door_position == DOOR_OPEN_POSITION) {
    button_state[level_position] = false;
    setButtonLight(level_position, off);
    setState(wait_state);
  } else {
    moveDoors(DOOR_CLOSED_POSITION, DOOR_OPEN_POSITION);
  }
}

void closeDoors() {
  if (button_state[level_position] == true) {
    setState(opendoors_state);
    return;
  }

  if (door_position == DOOR_CLOSED_POSITION) {
    setState(sleep_state);
  } else {
    moveDoors(DOOR_OPEN_POSITION, DOOR_CLOSED_POSITION);
  }
}

void moveDoors(int fromPosition, int toPosition) {
  long time; 
  long remainTime;
  long door_end_time;
  
  time = millis(); // The current time in millis

  // Check if we just entered the current door movement state
  if (state_cycle == 1) {
    // Init door movement parameters
    remainTime = map(door_position, fromPosition, toPosition, DOOR_SPEED, 0);
    door_start_time = map(fromPosition, door_position, toPosition, time, time + remainTime);
  }

  door_end_time = door_start_time + DOOR_SPEED;

  door_position = map(time, door_start_time, door_end_time, fromPosition, toPosition);

  setDoorPositions(door_position);
}

