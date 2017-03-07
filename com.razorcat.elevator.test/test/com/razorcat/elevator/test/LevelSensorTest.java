package com.razorcat.elevator.test;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import com.razorcat.elevator.test.LevelSensor.MotorDirection;
import com.razorcat.elevator.test.LevelSensor.PositionState;

public class LevelSensorTest {

    @Test
    public void testMoveUpScenario() {
        LevelSensor sensor = new LevelSensor();
        
        sensor.transferLevelSensors(1, MotorDirection.up);
        
        assertEquals(PositionState.far_above, sensor.getPositionState(0));
        assertEquals(PositionState.unknown, sensor.getPositionState(1));
        assertEquals(PositionState.far_below, sensor.getPositionState(2));
        
        sensor.transferLevelSensors(-1, MotorDirection.up);

        assertEquals(PositionState.far_above, sensor.getPositionState(0));
        assertEquals(PositionState.unknown, sensor.getPositionState(1));
        assertEquals(PositionState.far_below, sensor.getPositionState(2));
        
        sensor.transferLevelSensors(1, MotorDirection.up);
        
        assertEquals(PositionState.far_above, sensor.getPositionState(0));
        assertEquals(PositionState.unknown, sensor.getPositionState(1));
        assertEquals(PositionState.far_below, sensor.getPositionState(2));

        sensor.transferLevelSensors(-1, MotorDirection.up);

        assertEquals(PositionState.far_above, sensor.getPositionState(0));
        assertEquals(PositionState.unknown, sensor.getPositionState(1));
        assertEquals(PositionState.far_below, sensor.getPositionState(2));

        sensor.transferLevelSensors(2, MotorDirection.up);
        
        assertEquals(PositionState.far_above, sensor.getPositionState(0));
        assertEquals(PositionState.far_above, sensor.getPositionState(1));
        assertEquals(PositionState.close_below, sensor.getPositionState(2));

        sensor.transferLevelSensors(-1, MotorDirection.up);

        assertEquals(PositionState.far_above, sensor.getPositionState(0));
        assertEquals(PositionState.far_above, sensor.getPositionState(1));
        assertEquals(PositionState.reached, sensor.getPositionState(2));
    }
    
    @Test
    public void testMoveDownScenario() {
        LevelSensor sensor = new LevelSensor();
        
        sensor.transferLevelSensors(2, MotorDirection.down);
        
        assertEquals(PositionState.far_above, sensor.getPositionState(0));
        assertEquals(PositionState.far_above, sensor.getPositionState(1));
        assertEquals(PositionState.unknown, sensor.getPositionState(2));
        
        sensor.transferLevelSensors(-1, MotorDirection.down);

        assertEquals(PositionState.far_above, sensor.getPositionState(0));
        assertEquals(PositionState.far_above, sensor.getPositionState(1));
        assertEquals(PositionState.unknown, sensor.getPositionState(2));
        
        sensor.transferLevelSensors(1, MotorDirection.down);
        
        assertEquals(PositionState.far_above, sensor.getPositionState(0));
        assertEquals(PositionState.close_above, sensor.getPositionState(1));
        assertEquals(PositionState.far_below, sensor.getPositionState(2));

        sensor.transferLevelSensors(-1, MotorDirection.down);

        assertEquals(PositionState.far_above, sensor.getPositionState(0));
        assertEquals(PositionState.reached, sensor.getPositionState(1));
        assertEquals(PositionState.far_below, sensor.getPositionState(2));

        sensor.transferLevelSensors(1, MotorDirection.down);
        
        assertEquals(PositionState.far_above, sensor.getPositionState(0));
        assertEquals(PositionState.close_below, sensor.getPositionState(1));
        assertEquals(PositionState.far_below, sensor.getPositionState(2));

        sensor.transferLevelSensors(-1, MotorDirection.down);

        assertEquals(PositionState.far_above, sensor.getPositionState(0));
        assertEquals(PositionState.far_below, sensor.getPositionState(1));
        assertEquals(PositionState.far_below, sensor.getPositionState(2));
    }
    
    @Test
    public void testSetOutsideLevelStates0() {
        LevelSensor sensor = new LevelSensor();
        
        sensor.setOutsideLevelStates(0);
        assertEquals(PositionState.unknown, sensor.getPositionState(0));
        assertEquals(PositionState.far_below, sensor.getPositionState(1));
        assertEquals(PositionState.far_below, sensor.getPositionState(2));
    }

    @Test
    public void testSetOutsideLevelStates1() {
        LevelSensor sensor = new LevelSensor();
        
        sensor.setOutsideLevelStates(1);
        assertEquals(PositionState.far_above, sensor.getPositionState(0));
        assertEquals(PositionState.unknown, sensor.getPositionState(1));
        assertEquals(PositionState.far_below, sensor.getPositionState(2));
    }

    @Test
    public void testSetOutsideLevelStates2() {
        LevelSensor sensor = new LevelSensor();
        
        sensor.setOutsideLevelStates(2);
        assertEquals(PositionState.far_above, sensor.getPositionState(0));
        assertEquals(PositionState.far_above, sensor.getPositionState(1));
        assertEquals(PositionState.unknown, sensor.getPositionState(2));
    }

    @Test
    public void testSetOutsideLevelStates3() {
        LevelSensor sensor = new LevelSensor();
        
        sensor.setOutsideLevelStates(3);
        assertEquals(PositionState.far_above, sensor.getPositionState(0));
        assertEquals(PositionState.far_above, sensor.getPositionState(1));
        assertEquals(PositionState.far_above, sensor.getPositionState(2));
    }
    
    @Test
    public void testSetOutsideLevelStatesMin1() {
        LevelSensor sensor = new LevelSensor();
        
        sensor.setOutsideLevelStates(-1);
        assertEquals(PositionState.far_below, sensor.getPositionState(0));
        assertEquals(PositionState.far_below, sensor.getPositionState(1));
        assertEquals(PositionState.far_below, sensor.getPositionState(2));
    }
    
    
}
