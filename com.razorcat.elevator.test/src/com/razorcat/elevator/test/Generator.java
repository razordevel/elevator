package com.razorcat.elevator.test;

public class Generator {

    public static int combine(boolean a, boolean b) {
        return (a ? 2 : 0) + (b ? 1 : 0);
    }

    public static boolean[] split(int i) {
        boolean a = (i & 2) != 0;
        boolean b = (i & 1) != 0;
        return new boolean[] { a, b };
    }

    public static int find(int value) {
        for (int index = 0; index < ROUNDTRIP.length; index++) {
            if (ROUNDTRIP[index] == value) {
                return index;
            }
        }
        throw new IllegalArgumentException("Illegal value " + value);
    }

    public static int[] ROUNDTRIP = { 0, 1, 3, 2 };

    private int         value;

    public Generator() {
        this.value = 0;
    }

    public boolean isA() {
        return split(value)[0];
    }

    public boolean isB() {
        return split(value)[1];
    }

    public int getValue() {
        return value;
    }

    public int move(int move) {
        if (move == 0) {
            return getValue();
        }
        
        int index = find(value);
        index -= move;
        if (index < 0) {
            index += ROUNDTRIP.length;
        }
        index = index % ROUNDTRIP.length;
        value = ROUNDTRIP[index];
        return getValue();
    }
}
