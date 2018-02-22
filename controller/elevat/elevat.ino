#include <Wire.h>
#include <Servo.h>

// Generic constants
#define LEVEL_NUMBER 3 // Corresponds with levels 0, 1 and 2.

// Door related constants
#define DOOR_SPEED 2000         // Milliseconds for full open or close movement.
#define DOOR_CLOSED_POSITION 0  // Position value for a closed door.
#define DOOR_OPEN_POSITION 100  // Position value for an open door.
#define DOOR_WAIT_TIME 4000     // Milliseconds the doors will wait in the open position.
#define DOOR_LEFT_OPEN 10
#define DOOR_LEFT_CLOSED 120
#define DOOR_RIGHT_OPEN 120
#define DOOR_RIGHT_CLOSED 10

// Motor related constants
#define FAST_MOTOR_SPEED 255        // Maximum motor speed. Value between 0 and 255
#define SLOW_MOTOR_SPEED_UP 127     // Motor speed for slow movement up. Value between 0 and 255
#define SLOW_MOTOR_SPEED_DOWN 63    // Motor speed for slow movement down. Value between 0 and 255
#define ACCELERATE_MOTOR_TIME 2000  // Time in milliseconds to accelerate the motor to fast speed
#define DECELERATE_MOTOR_TIME 1000  // Time in milliseconds to decelerate the motor from fast to slow speed
#define MOTOR_STOP_REVERSE_TIME 10  // Time in milliseconds to move the motor up to do a good stop

// Safety switch debounce buffer handling
#define SAFETY_TIMING 2 // Milliseconds between measurements
#define SAFETY_BUFFER_SIZE 8
#define SAFETY_BUFFER_THRESHOLD 6

// Encoder related constants
#define ENCODER_RESOLUTION 24
#define MILLI_PER_SECOND 1000
#define REFERENCE_TURN 100
#define STOP_TIME 100

// register interface constants
#define I2C_BUFFER 5 // 1 address byte and 4 byte to set

// Pin constants
#define PIN_O_MOTOR_ENABLE 2     // port pin 3
#define PIN_O_MOTOR_UP 3         // port pin 4
#define PIN_I_SAFETY_UP 50       // port pin 5
#define PIN_O_MOTOR_DOWN 4       // port pin 6
#define PIN_I_SAFETY_DOWN 52     // port pin 7
#define PIN_I_TEMPERATURE 1      // port pin 10
#define PIN_I_ENCODER_A 38       // port pin 11
#define PIN_I_ENCODER_B 40       // port pin 12
#define PIN_I_LEVEL_0 36         // port pin 13
#define PIN_I_LEVEL_1 28         // port pin 14
#define PIN_I_LEVEL_2 30         // port pin 15
#define PIN_O_DOOR_LEFT 6        // port pin 16
#define PIN_O_DOOR_RIGHT 5       // port pin 17
#define PIN_I_LEVEL_BUTTON_0 34  // port pin 18
#define PIN_O_LEVEL_LIGHT_0 46   // port pin 19
#define PIN_I_LEVEL_BUTTON_1 32  // port pin 20
#define PIN_O_LEVEL_LIGHT_1 44   // port pin 21
#define PIN_I_LEVEL_BUTTON_2 26  // port pin 22
#define PIN_O_LEVEL_LIGHT_2 42   // port pin 23
#define PIN_O_CABIN_LIGHT 48     // port pin 24
#define PIN_O_GREEN_STATUS 12    // Green Status LED
#define PIN_O_RED_STATUS 13      // Red Status LED

// ------------------------------
// Enum definitions
// ------------------------------

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

// ------------------------------
// Global variables
// ------------------------------

// State handling
OperationState state = init_state; // Current state of the state machine
OperationState last_state = init_state; // Previous state of the state machine in the last cycle
long state_time = 0; // Holds the time the current state was entered.
long state_cycle = 0; // Holds the number of times the current state is iterated.
long cycle_time = 0; // Holds the time of the current cycle.

// Position handling
int level_position = -1;
int level_target = -1;
PositionState level_position_state[LEVEL_NUMBER];
int last_blocked_level = -1;
boolean button_state[LEVEL_NUMBER];

// Safety switch handling
boolean safetyUpBuffer[SAFETY_BUFFER_SIZE];
boolean safetyDownBuffer[SAFETY_BUFFER_SIZE];
int safetyBufferIndex = 0;
boolean safetyUp = false; // State of the safety switch above the highest floor
boolean safetyDown = false; // State of the safety switch below the lowest floor
long safetyTime = 0;

// Temperature variable
float motorTemperature = -1;

// Global variables for door movement
Servo servo_left;
Servo servo_right;
boolean servo_attached = false;
long door_start_time = 0; // Moment of door movement begin.
long door_end_time = 0; // Moment of expected door movement end.
int door_position = 0; // Holds door position as value between DOOR_CLOSED_POSITION and DOOR_OPEN_POSITION.

// Global variables for encoder interpretion
int encoder_value = 0;
long encoder_time = 0;
long encoder_ticks = 0;
long encoder_speed = 0;
boolean encoder_overspeed = false;

// Global variable for motor control
MotorDirection motor_direction;  // Current direction of the motor rotation
MotorSpeed motor_speed;          // Current speed of the motor
long motor_change_time;          // Time of the last motor speed change
long motor_final_time;           // Calculated time of the final motor speed
int motor_change_value;          // Value of the motor speed at the last change time

// Global variable for level init
MotorDirection init_direction = down;

// Global variable for register interface
int nbr_of_received_bytes = 0;
int received_values[I2C_BUFFER];
int request_queue[100];
int queue_start = 0;
int queue_end = 0;
int request_for_testc_state = 0;
int leave_testc_state = 0;

// ------------------------------

// Write functions for outputs
void setDoorPositions(int position) {
  if (!servo_attached) {
    servo_left.attach(PIN_O_DOOR_LEFT);
    servo_right.attach(PIN_O_DOOR_RIGHT);
    servo_attached = true;
  }
  int left = map(position, DOOR_CLOSED_POSITION, DOOR_OPEN_POSITION, DOOR_LEFT_CLOSED, DOOR_LEFT_OPEN);
  int right = map(position, DOOR_CLOSED_POSITION, DOOR_OPEN_POSITION, DOOR_RIGHT_CLOSED, DOOR_RIGHT_OPEN);
  servo_left.write(left);
  servo_right.write(right);
}

void stopDoors() {
  if (servo_attached) {
    servo_left.detach();
    servo_right.detach();
    servo_attached = false;
  }
}

void moveCabinMotor(MotorSpeed speed, MotorDirection direction) {
  if (motor_speed != speed) {
    motor_change_time = cycle_time;
    motor_final_time = motor_change_time;
    switch (motor_speed) {
      case stopped:
        motor_change_value = 0;
        break;
      case fast:
        motor_change_value = FAST_MOTOR_SPEED;
        break;
      case slow:
        motor_change_value = motor_direction == up ? SLOW_MOTOR_SPEED_UP : SLOW_MOTOR_SPEED_DOWN;
        break;
    }
    switch (speed) {
      case fast:
        motor_final_time += ACCELERATE_MOTOR_TIME;
        break;
      case slow:
        motor_final_time += motor_speed == fast ? DECELERATE_MOTOR_TIME : ACCELERATE_MOTOR_TIME;
        break;
    }
  }

  if (speed == stopped) {
    stopCabinMotor();
    return;
  }

  motor_direction = direction;
  motor_speed = speed;

  long time = cycle_time;

  digitalWrite(PIN_O_MOTOR_ENABLE, HIGH);
  if (direction == up) {
    int speedValue = speed == fast ? FAST_MOTOR_SPEED : SLOW_MOTOR_SPEED_UP;
    if (time <= motor_final_time) {
      speedValue = map(time, motor_change_time, motor_final_time, motor_change_value, speedValue);
    }
    analogWrite(PIN_O_MOTOR_UP, speedValue);
    digitalWrite(PIN_O_MOTOR_DOWN, LOW);
  } else {
    int speedValue = speed == fast ? FAST_MOTOR_SPEED : SLOW_MOTOR_SPEED_DOWN;
    if (time <= motor_final_time) {
      speedValue = map(time, motor_change_time, motor_final_time, motor_change_value, speedValue);
    }
    analogWrite(PIN_O_MOTOR_DOWN, speedValue);
    digitalWrite(PIN_O_MOTOR_UP, LOW);
  }
}

void stopCabinMotor() {
  if (cycle_time - state_time <= MOTOR_STOP_REVERSE_TIME) {
    digitalWrite(PIN_O_MOTOR_ENABLE, HIGH);
    analogWrite(PIN_O_MOTOR_UP, SLOW_MOTOR_SPEED_UP);
    digitalWrite(PIN_O_MOTOR_DOWN, LOW);
    return;
  }

  motor_direction = down;
  motor_speed = stopped;

  // Reset safety switch values
  safetyUp = false;
  safetyDown = false;

  digitalWrite(PIN_O_MOTOR_ENABLE, LOW);
  digitalWrite(PIN_O_MOTOR_UP, LOW);
  digitalWrite(PIN_O_MOTOR_DOWN, LOW);
}

void setCabinLight(LightMode light) {
  if (light == on) {
    digitalWrite(PIN_O_CABIN_LIGHT, HIGH);
  } else if (light == off) {
    digitalWrite(PIN_O_CABIN_LIGHT, LOW);
  } else {
    digitalWrite(PIN_O_CABIN_LIGHT, getFlashingValue());
  }
}
void setButtonLight(int level, LightMode light) {
  int pin = PIN_O_LEVEL_LIGHT_0;
  if (level == 1) {
    pin = PIN_O_LEVEL_LIGHT_1;
  } else if (level == 2) {
    pin = PIN_O_LEVEL_LIGHT_2;
  }

  if (light == on) {
    digitalWrite(pin, HIGH);
  } else if (light == off) {
    digitalWrite(pin, LOW);
  } else {
    digitalWrite(pin, getFlashingValue());
  }
}

int getFlashingValue() {
  long t = cycle_time;
  t = t - state_time;
  t = t % 500;
  return t < 250 ? HIGH : LOW;
}

// ------------------------------

// Read functions for inputs
boolean readLevelButton(int level) {
  int pin = PIN_I_LEVEL_BUTTON_0;
  if (level == 1) {
    pin = PIN_I_LEVEL_BUTTON_1;
  } else if (level == 2) {
    pin = PIN_I_LEVEL_BUTTON_2;
  }

  return digitalRead(pin) == HIGH;
}

boolean readLevelSensor(int level) {
  int pin = PIN_I_LEVEL_0;
  if (level == 1) {
    pin = PIN_I_LEVEL_1;
  } else if (level == 2) {
    pin = PIN_I_LEVEL_2;
  }

  return digitalRead(pin) == LOW;
}

// Possible values are 0, 1, 2 and 3. To interpret this value handle the two bits as separate values.
int readEncoderValue() {
  int a = digitalRead(PIN_I_ENCODER_A) == HIGH ? 1 : 0;
  int b = digitalRead(PIN_I_ENCODER_B) == HIGH ? 2 : 0;
  return a + b;
}

float readTemperature() {
  // resistor value of voltage divider in ohm
  float resistor = 2700;
  float sensorValue = analogRead(PIN_I_TEMPERATURE);
  float resistance = sensorValue / (1023 - sensorValue) * resistor;
  // resistor values from kty81-210 data sheet, written as polynomial trend line
  return -1.332e-11 * pow(resistance, 4) + 6.621e-8 * pow(resistance, 3) - 0.0002 * pow(resistance, 2) + 0.2947 * resistance - 230.55;
}

// ------------------------------

// Arduino Setup Function. Will be called once after system boot.
void setup() {
  // put your setup code here, to run once:

  // Setup input and output channels
  pinMode(PIN_O_RED_STATUS, OUTPUT);
  pinMode(PIN_O_GREEN_STATUS, OUTPUT);
  pinMode(PIN_O_MOTOR_ENABLE, OUTPUT);
  pinMode(PIN_O_MOTOR_UP, OUTPUT);
  pinMode(PIN_O_MOTOR_DOWN, OUTPUT);
  servo_left.attach(PIN_O_DOOR_LEFT);
  servo_right.attach(PIN_O_DOOR_RIGHT);
  pinMode(PIN_O_LEVEL_LIGHT_0, OUTPUT);
  pinMode(PIN_O_LEVEL_LIGHT_1, OUTPUT);
  pinMode(PIN_O_LEVEL_LIGHT_2, OUTPUT);
  pinMode(PIN_O_CABIN_LIGHT, OUTPUT);
  pinMode(PIN_I_LEVEL_BUTTON_0, INPUT);
  pinMode(PIN_I_LEVEL_BUTTON_1, INPUT);
  pinMode(PIN_I_LEVEL_BUTTON_2, INPUT);
  pinMode(PIN_I_SAFETY_UP, INPUT);
  pinMode(PIN_I_SAFETY_DOWN, INPUT);
  pinMode(PIN_I_TEMPERATURE, INPUT);
  pinMode(PIN_I_ENCODER_A, INPUT);
  pinMode(PIN_I_ENCODER_B, INPUT);
  pinMode(PIN_I_LEVEL_0, INPUT);
  pinMode(PIN_I_LEVEL_1, INPUT);
  pinMode(PIN_I_LEVEL_2, INPUT);

  // Setup arrays
  for (int i = 0; i < LEVEL_NUMBER; i++) {
    button_state[i] = false;
    level_position_state[i] = unknown;
  }

  // Setup i2c bus
  Wire.begin(4);                // join i2c bus with address #4
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event

  // Init state machine
  setState(init_state);

  // Set initial outputs
  stopCabinMotor();
  setCabinLight(off);
  for (int i = 0; i < LEVEL_NUMBER; i++) {
    setButtonLight(i, off);
  }
}

void setStatusLEDs() {
  switch (state) {
    case init_state:
      digitalWrite(PIN_O_RED_STATUS, HIGH);
      digitalWrite(PIN_O_GREEN_STATUS, HIGH);
      break;
    case maintenance_state:
      digitalWrite(PIN_O_RED_STATUS, HIGH);
      digitalWrite(PIN_O_GREEN_STATUS, LOW);
      break;
    case testc_state:
      digitalWrite(PIN_O_RED_STATUS, HIGH);
      digitalWrite(PIN_O_GREEN_STATUS, HIGH);
      break;
    default:
      digitalWrite(PIN_O_RED_STATUS, LOW);
      digitalWrite(PIN_O_GREEN_STATUS, HIGH);
      break;
  }
}

// Arduino Loop Function. Will be called in an endless loop.
void loop() {
  state_cycle++;
  cycle_time = millis();

  if (state == testc_state && leave_testc_state == 0) {
	  return;
  }
  
  transferInputs();

  setStatusLEDs();

  switch (state) {
    case init_state:
      initialize();
      break;
    case sleep_state:
      sleep();
      break;
    case move_state:
      moveCabin();
      break;
    case opendoors_state:
      openDoors();
      break;
    case wait_state:
      wait();
      break;
    case closedoors_state:
      closeDoors();
      break;
    case maintenance_state:
      maintenance();
      break;
    case testc_state:
      testc();
      break;
    default:
      setState(maintenance_state);
      break;
  }
}

void testc() {
  if (leave_testc_state == 1) {
    leave_testc_state = 0;
    setState(sleep_state);
  }
}

void setState(OperationState newState) {
  state_time = cycle_time;

  // TODO: accept only valid state changes

  last_state = state;
  state = newState;
  state_cycle = 0; // reset the state cycle
}

void transferButtonInputs() {
  // Level Button Inputs
  for (int i = 0; i < LEVEL_NUMBER; i++) {
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
  long newTime = cycle_time;

  if (encoder_time == newTime) {
    return;
  }

  if (newValue != encoder_value) {
    boolean oldA = (encoder_value & 1) != 0;
    boolean oldB = (encoder_value & 2) != 0;
    boolean newA = (newValue & 1) != 0;
    boolean newB = (newValue & 2) != 0;

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

    long timeDelta = newTime - encoder_time;
    long t = ENCODER_RESOLUTION * timeDelta;
    encoder_speed = REFERENCE_TURN * MILLI_PER_SECOND / t;

    if ((encoder_value == 0 && newValue == 1) || (encoder_value == 1 && newValue == 3) ||
        (encoder_value == 3 && newValue == 2) || (encoder_value == 2 && newValue == 0)) {
      encoder_speed *= -1;
    }

    encoder_time = newTime;
    encoder_value = newValue;
  } else if (newTime > encoder_time + STOP_TIME) {
    encoder_overspeed = false;
    encoder_value = newValue;
    encoder_time = newTime;
    encoder_speed = 0;
  }
}

int readBlockedLevelSensor() {
  int blockedLevel = -1;
  for (int i = 0; i < LEVEL_NUMBER; i++) {
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

  if (blockedLevel != last_blocked_level) {
    if (last_blocked_level == -1) {
      PositionState oldState = level_position_state[blockedLevel];
      PositionState newState = unknown;

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
        PositionState oldState = level_position_state[last_blocked_level];
        PositionState newState = unknown;

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
  for (int i = 0; i < level; i++) {
    level_position_state[i] = far_above;
  }
  for (int i = level + 1; i < LEVEL_NUMBER; i++) {
    level_position_state[i] = far_below;
  }
}

void transferTemperature() {
  motorTemperature = readTemperature();
}

void transferInputs() {
  transferSafetySwitch();
  transferButtonInputs();
  transferEncoderInput();
  transferLevelSensors();
  transferTemperature();
}

boolean readSafetySwitch(int pin) {
  return digitalRead(pin) == HIGH;
}

void transferSafetySwitch() {
  if (cycle_time < safetyTime + SAFETY_TIMING) {
    return;
  }

  safetyTime = cycle_time;
  safetyBufferIndex++;
  if (safetyBufferIndex >= SAFETY_BUFFER_SIZE) {
    safetyBufferIndex = 0;
  }
  safetyUpBuffer[safetyBufferIndex] = readSafetySwitch(PIN_I_SAFETY_UP);
  safetyDownBuffer[safetyBufferIndex] = readSafetySwitch(PIN_I_SAFETY_DOWN);

  int upCount = 0;
  int downCount = 0;
  int i;
  for (i = 0; i < SAFETY_BUFFER_SIZE; i++) {
    if (safetyUpBuffer[i]) {
      upCount++;
    }
    if (safetyDownBuffer[i]) {
      downCount++;
    }
  }
  safetyUp = upCount >= SAFETY_BUFFER_THRESHOLD;
  safetyDown = downCount >= SAFETY_BUFFER_THRESHOLD;
}

int findTargetLevel() {
  // TODO: improve algorithm to find the next close floor that was not visited previously
  for (int i = 0; i < LEVEL_NUMBER; i++) {
    if (button_state[i]) {
      return i;
    }
  }
  return -1;
}

void moveCabin() {
  // Check if we just entered the current door movement state
  if (state_cycle == 1) {
    level_target = findTargetLevel();
  }

  PositionState p = level_position_state[level_target];
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
  if (button_state[level_position]) {
    setState(opendoors_state);
    return;
  }

  if (door_position == DOOR_CLOSED_POSITION) {
    stopDoors();
    setState(sleep_state);
  } else {
    moveDoors(DOOR_OPEN_POSITION, DOOR_CLOSED_POSITION);
  }
}

void moveDoors(int fromPosition, int toPosition) {
  // Check if we just entered the current door movement state
  if (state_cycle == 1) {
    // Init door movement parameters
    long remainTime = map(door_position, fromPosition, toPosition, DOOR_SPEED, 0);
    door_start_time = map(fromPosition, door_position, toPosition, cycle_time, cycle_time + remainTime);
    door_end_time = door_start_time + DOOR_SPEED;
  }

  door_position = map(cycle_time, door_start_time, door_end_time, fromPosition, toPosition);

  setDoorPositions(door_position);
}

void wait() {
  if (cycle_time >= state_time + DOOR_WAIT_TIME) {
    setState(closedoors_state);
  } else {
    delay(1);
  }
}

void maintenance() {
  stopCabinMotor();
  setCabinLight(off);

  for (int i = 0; i < LEVEL_NUMBER; i++) {
    setButtonLight(i, flashing);
  }
}

void sleep() {

  if (state == testc_state) {
	  return;
  }
  
  if (request_for_testc_state == 1) {
    request_for_testc_state = 0;
    setState(testc_state);
    return;
  }
  
  for (int i = 0; i < LEVEL_NUMBER; i++) {
    if (button_state[i] && level_position_state[i] == reached) {
      setState(opendoors_state);
      return;
    }
  }

  for (int i = 0; i < LEVEL_NUMBER; i++) {
    if (button_state[i] && level_position_state[i] != reached) {
      setState(move_state);
      return;
    }
  }
}

void initialize() {
  setCabinLight(on);
  for (int i = 0; i < LEVEL_NUMBER; i++) {
    setButtonLight(i, off);
    button_state[i] = false;
  }

  setDoorPositions(DOOR_CLOSED_POSITION);

  // Move Cabin down until motor stops because of safety switch.
  // If Cabin at lowest point and level 0 sensor blocked move up to level sensor gap.
  if (init_direction == down) {
    moveCabinMotor(fast, down);

    if (safetyDown) {
      stopCabinMotor();
      int blockedLevel = readBlockedLevelSensor();
      if (blockedLevel == 0) {
        level_position_state[0] = close_below;
      } else if (blockedLevel < 0) {
        level_position_state[0] = far_below;
      }
      setOutsideLevelStates(0);
      init_direction = up;
    }
  } else if (init_direction == up) {
    MotorSpeed initSpeed = isAnyLevelClose() ? slow : fast;
    moveCabinMotor(initSpeed, up);

    for (int i = 0; i < LEVEL_NUMBER; i++) {
      if (level_position_state[i] == reached) {
        stopCabinMotor();
        stopDoors();
        setState(sleep_state);
        return;
      }
    }

    if (safetyUp) {
      stopCabinMotor();
      init_direction = down;
    }
  } else {
    init_direction = down;
  }
}

boolean isAnyLevelClose() {
  for (int i = 0; i < LEVEL_NUMBER; i++) {
    if (level_position_state[i] == close_below || level_position_state[i] == close_above) {
      return true;
    }
  }
  return false;
}

void intToCharArray(char *buf, int value) {
  unsigned int val = (unsigned int) value;
  buf[0] = (val >> 24) & 0xff;
  buf[1] = (val >> 16) & 0xff;
  buf[2] = (val >> 8) & 0xff;
  buf[3] = val & 0xff;
}

int arrayToInt(int *buf) {
  int val = buf[0] << 24;
  val |= buf[1] << 16;
  val |= buf[2] << 8;
  val |= buf[3];
  return val;
}

// i2c receive event
void receiveEvent(int howMany) {

  nbr_of_received_bytes = 0;
  while (Wire.available() > 0) {
    if (nbr_of_received_bytes < I2C_BUFFER) {
      received_values[nbr_of_received_bytes++] = Wire.read();
    } else {
      Wire.read();
    }
  }
  if (nbr_of_received_bytes == I2C_BUFFER) {
    int address = received_values[0];
    
    if (state != testc_state && request_for_testc_state == 0) {
      int request = arrayToInt(&received_values[1]);
      if (address == 0 && request == 1) {
        request_for_testc_state = 1;
      }
      return;
      
    } else if (state != testc_state) {
      return;
     
    } else if (address == 0) {
      int request = arrayToInt(&received_values[1]);
      if (request == 2) {
        leave_testc_state = 1;
      }
      return;
    
    } else if (address == 1) {
      state = (OperationState) arrayToInt(&received_values[1]);

    } else if (address == 2) {
      last_state = (OperationState) arrayToInt(&received_values[1]);

    } else if (address == 3) {
      int tmp = arrayToInt(&received_values[1]);
      state_time = *((long*)&tmp);

    } else if (address == 4) {
      int tmp = arrayToInt(&received_values[1]);
      state_cycle = *((long*)&tmp);

    } else if (address == 5) {
      level_position = arrayToInt(&received_values[1]);

    } else if (address == 6) {
      level_target = arrayToInt(&received_values[1]);

    } else if (address == 7) {
      level_position_state[0] = (PositionState) arrayToInt(&received_values[1]);

    } else if (address == 8) {
      level_position_state[1] = (PositionState) arrayToInt(&received_values[1]);

    } else if (address == 9) {
      level_position_state[2] = (PositionState) arrayToInt(&received_values[1]);

    } else if (address == 10) {
      last_blocked_level = arrayToInt(&received_values[1]);

    } else if (address == 11) {
      int boolean_as_int = arrayToInt(&received_values[1]);
      button_state[0] = false;
      if (boolean_as_int == 1) {
        button_state[0] = true;
      }

    } else if (address == 12) {
      int boolean_as_int = arrayToInt(&received_values[1]);
      button_state[1] = false;
      if (boolean_as_int == 1) {
        button_state[1] = true;
      }

    } else if (address == 13) {
      int boolean_as_int = arrayToInt(&received_values[1]);
      button_state[2] = false;
      if (boolean_as_int == 1) {
        button_state[2] = true;
      }

    } else if (address == 14) {
      int tmp = arrayToInt(&received_values[1]);
      door_start_time = *((long*)&tmp);

    } else if (address == 15) {
      door_position = arrayToInt(&received_values[1]);

    } else if (address == 16) {
      encoder_value = arrayToInt(&received_values[1]);

    } else if (address == 17) {
      int tmp = arrayToInt(&received_values[1]);
      encoder_time = *((long*)&tmp);

    } else if (address == 18) {
      int tmp = arrayToInt(&received_values[1]);
      encoder_ticks = *((long*)&tmp);

    } else if (address == 19) {
      int tmp = arrayToInt(&received_values[1]);
      encoder_speed = *((float*)&tmp);

    } else if (address == 20) {
      int boolean_as_int = arrayToInt(&received_values[1]);
      encoder_overspeed = false;
      if (boolean_as_int == 1) {
        encoder_overspeed = true;
      }

    }
  } else if (nbr_of_received_bytes == 1) {
    request_queue[queue_end++] = received_values[0];
    if (queue_end == 100) {
      queue_end = 0;
    }
  }
}

// i2c request event
void requestEvent() {

  if (state != testc_state) {
    queue_start = 0;
    queue_end = 0;
    Wire.write("ERRO");
    return;
  }

  if (queue_start < queue_end) {
    int requested_address = request_queue[queue_start++];
    if (queue_start == 100) {
      queue_start = 0;
    }
    if (requested_address == 0) {
      char val[4];
      intToCharArray(val, 1);
      Wire.write(val, 4);

    } else if (requested_address == 1) { // OperationState state
      char val[4];
      intToCharArray(val, state);
      Wire.write(val, 4);

    } else if (requested_address == 2) { // OperationState last_state
      char val[4];
      intToCharArray(val, last_state);
      Wire.write(val, 4);

    } else if (requested_address == 3) { // long state_time
      char val[4];
      intToCharArray(val, *((int*)&state_time));
      Wire.write(val, 4);

    } else if (requested_address == 4) { // long state_cycle
      char val[4];
      intToCharArray(val, *((int*)&state_cycle));
      Wire.write(val, 4);

    } else if (requested_address == 5) { // int level_position
      char val[4];
      intToCharArray(val, level_position);
      Wire.write(val, 4);

    } else if (requested_address == 6) { // int level_target
      char val[4];
      intToCharArray(val, level_target);
      Wire.write(val, 4);

    } else if (requested_address == 7) { // PositionState level_position_state[0]
      char val[4];
      intToCharArray(val, level_position_state[0]);
      Wire.write(val, 4);

    } else if (requested_address == 8) { // PositionState level_position_state[1]
      char val[4];
      intToCharArray(val, level_position_state[1]);
      Wire.write(val, 4);

    } else if (requested_address == 9) { // PositionState level_position_state[2]
      char val[4];
      intToCharArray(val, level_position_state[2]);
      Wire.write(val, 4);

    } else if (requested_address == 10) { // int last_blocked_level
      char val[4];
      intToCharArray(val, last_blocked_level);
      Wire.write(val, 4);

    } else if (requested_address == 11) { // boolean button_state[0]
      char val[4];
      int bool_as_int = 0;
      if (button_state[0] == 1) {
        bool_as_int = 1;
      }
      intToCharArray(val, bool_as_int);
      Wire.write(val, 4);
      Serial.println("send 0");

    } else if (requested_address == 12) { // boolean button_state[1]
      char val[4];
      int bool_as_int = 0;
      if (button_state[1] == 1) {
        bool_as_int = 1;
      }
      intToCharArray(val, bool_as_int);
      Wire.write(val, 4);
      Serial.println("send 1");

    } else if (requested_address == 13) { // boolean button_state[2]
      char val[4];
      int bool_as_int = 0;
      if (button_state[2] == 1) {
        bool_as_int = 1;
      }
      intToCharArray(val, bool_as_int);
      Wire.write(val, 4);
      Serial.println("send 2");

    } else if (requested_address == 14) { // long door_start_time
      char val[4];
      intToCharArray(val, *((int*)&door_start_time));
      Wire.write(val, 4);

    } else if (requested_address == 15) { // int door_position
      char val[4];
      intToCharArray(val, door_position);
      Wire.write(val, 4);

    } else if (requested_address == 16) { // int encoder_value
      char val[4];
      intToCharArray(val, encoder_value);
      Wire.write(val, 4);

    } else if (requested_address == 17) { // long encoder_time
      char val[4];
      intToCharArray(val, *((int*)&encoder_time));
      Wire.write(val, 4);

    } else if (requested_address == 18) { // long encoder_ticks
      char val[4];
      intToCharArray(val, *((int*)&encoder_ticks));
      Wire.write(val, 4);

    } else if (requested_address == 19) { // float encoder_speed
      char val[4];
      intToCharArray(val, *((int*)&encoder_speed));
      Wire.write(val, 4);

    } else if (requested_address == 20) { // boolean encoder_overspeed
      char val[4];
      int bool_as_int = 0;
      if (button_state[1] == 1) {
        bool_as_int = 1;
      }
      intToCharArray(val, encoder_overspeed);
      Wire.write(val, 4);

    } else if (requested_address == 21) { // MotorDirection motor_direction
      char val[4];
      intToCharArray(val, motor_direction);
      Wire.write(val, 4);

    } else if (requested_address == 22) { // MotorSpeed motor_speed
      char val[4];
      intToCharArray(val, motor_speed);
      Wire.write(val, 4);

    }
  }
}


