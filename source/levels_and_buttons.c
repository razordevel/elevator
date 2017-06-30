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

// ------------------------------
void setButtonLight(int level, enum LightMode light) {
  int pin = PIN_O_LEVEL_LIGHT_1;
  if (level == 2) {
    pin = PIN_O_LEVEL_LIGHT_2;
  } else if (level == 3) {
    pin = PIN_O_LEVEL_LIGHT_3;
  }

  if (light == on) {
    digitalWrite(pin, HIGH);
  } else {
    digitalWrite(pin, LOW);
  }
}

// ------------------------------

// Read functions for inputs
boolean readLevelButton(int level) {
  int pin = PIN_I_LEVEL_BUTTON_1;
  if (level == 2) {
    pin = PIN_I_LEVEL_BUTTON_2;
  } else if (level == 3) {
    pin = PIN_I_LEVEL_BUTTON_3;
  }

  return digitalRead(pin) == HIGH; // Maybe read as analog value
}

boolean readLevelSensor(int level) {
  int pin = PIN_I_LEVEL_1;
  if (level == 2) {
    pin = PIN_I_LEVEL_2;
  } else if (level == 3) {
    pin = PIN_I_LEVEL_3;
  }

  // TODO: Wert entprellen!
  return digitalRead(pin) == HIGH; // Maybe read as analog value
}

// Possible values are 0, 1, 2 and 3. To interpret this value handle the two bits as separate values.
int readEncoderValue() {
  int a = digitalRead(PIN_I_ENCODER_A);
  int b = digitalRead(PIN_I_ENCODER_B);
  return a + (b << 2);
}

float readTemperature() {
  int i;
  float temp              = 82;
  ADCSRA = 0x00;
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  ADMUX = 0x00;
  ADMUX = (1 << REFS0);
  ADMUX |= PIN_I_TEMPERATURE;

  for (i = 0; i <= 64; i++)
  {
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    temp += (ADCL + ADCH * 256);
  }

  temp /= 101;
  temp -= 156;
  return (temp);
}

void transferButtonInputs() {
  int i;
  // Level Button Inputs
  for (i = 0; i < LEVEL_NUMBER; i++) {
    boolean pressed = readLevelButton(i);
    if (pressed && !button_state[i]) {
      button_state[i] = true;
      setButtonLight(i, on);
    }
  }
}

void transferEncoderInput() {
  // Encoder Input
  int newValue = readEncoderValue();
  long newTime = millis();

  if (newValue != encoder_value) {
    boolean oldA = encoder_value & 1;
    boolean oldB = encoder_value & 2;
    boolean newA = newValue & 1;
    boolean newB = newValue & 2;

    if (oldA != newA && oldB != newB) {
      encoder_overspeed = true;
      encoder_value = newValue;
      encoder_time = newTime;
      encoder_ticks += 2;
      encoder_speed = 0;
      return;
    }

    encoder_overspeed = false;
    encoder_ticks++;

    float t = 24 * (newTime - encoder_time);
    encoder_speed = 1000 / t;

    if ((encoder_value == 0 && newValue == 1) || (encoder_value == 1 && newValue == 3) ||
        (encoder_value == 3 && newValue == 2) || (encoder_value == 2 && newValue == 0)) {
      encoder_speed *= -1;
    }

    encoder_time = newTime;
    encoder_value = newValue;
  }
}

int readBlockedLevelSensor() {
  int i;
  int blockedLevel = -1;
  for (i = 0; i < LEVEL_NUMBER; i++) {
    if (readLevelSensor(i)) {
      if (blockedLevel == -1) {
        blockedLevel = i;
      } else {
        // Two level sensors are blocked! Switch to maintenance.
        setState(maintenance_state);
      }
    }
  }
  return blockedLevel;
}

void transferLevelSensors() {
  int blockedLevel = readBlockedLevelSensor();
  enum PositionState oldState;
  enum PositionState newState;

  if (blockedLevel != last_blocked_level) {
    if (last_blocked_level == -1) {
      oldState = level_position_state[blockedLevel];
      newState = unknown;

      if (oldState == unknown) {
        // TODO: unknown handling
      } else if (oldState == far_above) {
        newState = close_above;
      } else if (oldState == far_below) {
        newState = close_below;
      } else if (oldState == reached) {
        if (motor_direction == up) {
          newState = close_above;
        } else {
          newState = close_below;
        }
      } else {
        // Bug! Switch to maintenance.
        setState(maintenance_state);
      }

      level_position_state[blockedLevel] = newState;
      setOutsideLevelStates(blockedLevel);
    } else {
      if (blockedLevel == -1) {
        oldState = level_position_state[blockedLevel];
        newState = unknown;

        if (oldState == unknown) {
          // TODO: unknown handling
        } else if (oldState == close_above) {
          if (motor_direction == up) {
            newState = far_above;
          } else {
            newState = reached;
            level_position = last_blocked_level;
          }
        } else if (oldState == close_below) {
          if (motor_direction == up) {
            newState = reached;
            level_position = last_blocked_level;
          } else {
            newState = far_below;
          }
        } else {
          // Bug! Switch to maintenance.
          setState(maintenance_state);
        }

        level_position_state[last_blocked_level] = newState;
        setOutsideLevelStates(last_blocked_level);
      } else {
        // Jump from one level directly to another! Switch to maintenance.
        setState(maintenance_state);
      }
    }

    last_blocked_level = blockedLevel;
  }
}

void setOutsideLevelStates(int level) {
  int i;
  for (i = 0; i < level; i++) {
    level_position_state[i] = far_above;
  }
  for (i = level + i; i < LEVEL_NUMBER; i++) {
    level_position_state[i] = far_below;
  }
}

void transferTemperature(void) {
  motorTemperature = readTemperature();
}

void transferInputs(void) {
  transferButtonInputs();
  transferEncoderInput();
  transferLevelSensors();
  transferTemperature();
}

int findTargetLevel(void) {
    int i;
// TODO: improve algorithm to find the next close floor that was not visited previously
  for (i = 0; i < LEVEL_NUMBER; i++) {
    if (button_state[i] == true) {
      return i;
    }
  }
  return -1;
}
