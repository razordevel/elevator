package com.razorcat.elevator.test;

public class LevelSensor {

    public enum MotorDirection {
        up, down
    };

    public enum PositionState {
        unknown, far_below, close_below, reached, close_above, far_above
    };

    private static final int LEVEL_NUMBER           = 3;

    private int              level_position         = -1;
    private PositionState    level_position_state[] = new PositionState[LEVEL_NUMBER];
    private int              last_blocked_level     = -1;
    
    public LevelSensor() {
        for (int i = 0; i < LEVEL_NUMBER; i++) {
            level_position_state[i] = PositionState.unknown;
        }
    }

    public void transferLevelSensors(int blockedLevel, MotorDirection motor_direction) {
        if (blockedLevel != last_blocked_level) {
            if (last_blocked_level == -1) {
                PositionState oldState = level_position_state[blockedLevel];
                PositionState newState = PositionState.unknown;

                if (oldState == PositionState.unknown) {
                    // TODO: unknown handling
                } else if (oldState == PositionState.far_above) {
                    newState = PositionState.close_above;
                } else if (oldState == PositionState.far_below) {
                    newState = PositionState.close_below;
                } else if (oldState == PositionState.reached) {
                    if (motor_direction == MotorDirection.up) {
                        newState = PositionState.close_above;
                    } else {
                        newState = PositionState.close_below;
                    }
                } else {
                    // Bug! Switch to maintenance.
                    throw new IllegalStateException("Maintenance");
                }

                level_position_state[blockedLevel] = newState;
                setOutsideLevelStates(blockedLevel);
            } else {
                if (blockedLevel == -1) {
                    PositionState oldState = level_position_state[last_blocked_level];
                    PositionState newState = PositionState.unknown;

                    if (oldState == PositionState.unknown) {
                        // TODO: unknown handling
                    } else if (oldState == PositionState.close_above) {
                        if (motor_direction == MotorDirection.up) {
                            newState = PositionState.far_above;
                        } else {
                            newState = PositionState.reached;
                            level_position = last_blocked_level;
                        }
                    } else if (oldState == PositionState.close_below) {
                        if (motor_direction == MotorDirection.up) {
                            newState = PositionState.reached;
                            level_position = last_blocked_level;
                        } else {
                            newState = PositionState.far_below;
                        }
                    } else {
                        // Bug! Switch to maintenance.
                        throw new IllegalStateException("Maintenance");
                    }

                    level_position_state[last_blocked_level] = newState;
                    setOutsideLevelStates(last_blocked_level);
                } else {
                    // Jump from one level directly to another! Switch to maintenance.
                    throw new IllegalStateException("Maintenance");
                }
            }

            last_blocked_level = blockedLevel;
        }
    }
    
    public PositionState getPositionState(int level) {
        return level_position_state[level];
    }
    
    public int getLevelPosition() {
        return level_position;
    }

    public void setOutsideLevelStates(int level) {
        for (int i = 0; i < level; i++) {
            level_position_state[i] = PositionState.far_above;
        }
        for (int i = level + 1; i < LEVEL_NUMBER; i++) {
            level_position_state[i] = PositionState.far_below;
        }
    }
}
