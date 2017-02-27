// Generic constants
const int LEVEL_NUMBER = 3; // Corresponds with levels 0, 1 and 2.

// Door related constants
const int DOOR_SPEED = 2000; // Milliseconds for full open or close movement.
const int DOOR_CLOSED_POSITION = 0; // Position value for a closed door.
const int DOOR_OPEN_POSITION = 100; // Position value for an open door.
const long DOOR_WAIT_TIME = 4000; // Milliseconds the doors will wait in the open position.

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
long state_time = 0;
long state_cycle = 0; // Holds the number of times the current state is iterated.

int level_position = -1;
int level_target = -1;
PositionState level_position_state[LEVEL_NUMBER];
int last_blocked_level = -1;
boolean button_state[LEVEL_NUMBER];

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
MotorDirection motor_direction;
MotorSpeed motor_speed;

// ------------------------------

// Stub functions for outputs
void setDoorPositions(int position);

void moveCabinMotor(MotorSpeed speed, MotorDirection direction);
void stopCabinMotor();

void setCabinLight(LightMode light);
void setButtonLight(int level, LightMode light);

// ------------------------------

// Stub functions for inputs
boolean readLevelButton(int level) {
  return false;
}

boolean readLevelSensor(int level) {
  // TODO: Wert entprellen!
  return false;
}

// Possible values are 0, 1, 2 and 3. To interpret this value handle the two bits as separate values.
int readEncoderValue() {
  return 0;
}
// ------------------------------

// Arduino Setup Function. Will be called once after system boot.
void setup() {
  // put your setup code here, to run once:
  // Setup arrays
  for (int i = 0; i < LEVEL_NUMBER; i++) {
    button_state[i] = false;
    level_position_state[i] = unknown;
  }

  // Setup input and output channels

  // Init state machine and input variables
  setState(init_state);
}

// Arduino Loop Function. Will be called in an endless loop.
void loop() {
  state_cycle++;

  transferInputs();

  switch (state) {
init_state:
      initialize();
      break;
sleep_state:
      sleep();
      break;
move_state:
      moveCabin();
      break;
opendoors_state:
      openDoors();
      break;
wait_state:
      wait();
      break;
closedoors_state:
      closeDoors();
      break;
maintenance_state:
      maintenance();
      break;
testc_state:
      testc();
  }
}

void testc() {
  // TODO:
}

void setState(OperationState newState) {
  state_time = millis();

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
        PositionState oldState = level_position_state[blockedLevel];
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
  for (int i = level + i; i < LEVEL_NUMBER; i++) {
    level_position_state[i] = far_below;
  }
}

void transferInputs() {
  transferButtonInputs();
  transferEncoderInput();
  transferLevelSensors();
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
  long time = millis(); // The current time in millis

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
    setState(sleep_state);
  } else {
    moveDoors(DOOR_OPEN_POSITION, DOOR_CLOSED_POSITION);
  }
}

void moveDoors(int fromPosition, int toPosition) {
  long time = millis(); // The current time in millis

  // Check if we just entered the current door movement state
  if (state_cycle == 1) {
    // Init door movement parameters
    long remainTime = map(door_position, fromPosition, toPosition, DOOR_SPEED, 0);
    door_start_time = map(fromPosition, door_position, toPosition, time, time + remainTime);
  }

  long door_end_time = door_start_time + DOOR_SPEED;

  door_position = map(time, door_start_time, door_end_time, fromPosition, toPosition);

  setDoorPositions(door_position);
}

void wait() {
  long time = millis(); // The current time in millis

  if (time >= state_time + DOOR_WAIT_TIME) {
    setState(closedoors_state);
  } else {
    delay(1);
  }
}

void maintenance() {
  setCabinLight(off);

  for (int i = 0; i < LEVEL_NUMBER; i++) {
    setButtonLight(i, flashing);
  }
}

void sleep() {
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
  closeDoors();
  setCabinLight(on);
  for (int i = 0; i < LEVEL_NUMBER; i++) {
    setButtonLight(i, off);
    button_state[i] = false;
  }

  for (int i = 0; i < LEVEL_NUMBER; i++) {
    if (level_position_state[i] == reached) {
      setState(sleep_state);
      return;
    }
  }

  // Move Cabin slowly down until level is reached or motor stops because of safety switch.
  // If Cabin at lowest point move slowly up until level is reached.

  moveCabinMotor(slow, down);

  // TODO: Stop wenn all level_position_states are known!
}


