package com.razorcat.elevator.test;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class GeneratorTest {

    @Test
    public void testSplit() {
        assertArrayEquals(new boolean[] { false, false }, Generator.split(0));
        assertArrayEquals(new boolean[] { false, true }, Generator.split(1));
        assertArrayEquals(new boolean[] { true, false }, Generator.split(2));
        assertArrayEquals(new boolean[] { true, true}, Generator.split(3));
    }
    
    @Test
    public void testCombine() {
        assertEquals(0, Generator.combine(false, false));
        assertEquals(1, Generator.combine(false, true));
        assertEquals(2, Generator.combine(true, false));
        assertEquals(3, Generator.combine(true, true));
    }
    
    @Test
    public void testGeneratorBackward() {
        final int STEP = -1;
        Generator generator = new Generator();
        
        assertEquals(Generator.ROUNDTRIP[0], generator.getValue());
        assertEquals(Generator.ROUNDTRIP[1], generator.move(STEP));
        assertEquals(Generator.ROUNDTRIP[2], generator.move(STEP));
        assertEquals(Generator.ROUNDTRIP[3], generator.move(STEP));
        assertEquals(Generator.ROUNDTRIP[0], generator.move(STEP));
        assertEquals(Generator.ROUNDTRIP[1], generator.move(STEP));
        assertEquals(Generator.ROUNDTRIP[2], generator.move(STEP));
        assertEquals(Generator.ROUNDTRIP[3], generator.move(STEP));
    }

    @Test
    public void testGeneratorForward() {
        final int STEP = 1;
        Generator generator = new Generator();
        
        assertEquals(Generator.ROUNDTRIP[0], generator.getValue());
        assertEquals(Generator.ROUNDTRIP[3], generator.move(STEP));
        assertEquals(Generator.ROUNDTRIP[2], generator.move(STEP));
        assertEquals(Generator.ROUNDTRIP[1], generator.move(STEP));
        assertEquals(Generator.ROUNDTRIP[0], generator.move(STEP));
        assertEquals(Generator.ROUNDTRIP[3], generator.move(STEP));
        assertEquals(Generator.ROUNDTRIP[2], generator.move(STEP));
        assertEquals(Generator.ROUNDTRIP[1], generator.move(STEP));
    }

    private void assertArrayEquals(boolean[] expected, boolean[] value) {
        assertEquals(expected.length, value.length);
        for (int i = 0; i < expected.length; i++) {
            assertEquals(expected[i], value[i]);
        }
    }
}
