package com.razorcat.elevator.test;

public class NewEncoder {

    public static final int    ENCODER_RESOLUTION = 24;
    public static final long   MILLI_PER_SECOND   = 1000;
    public static final long   REFERENCE_TURN     = 100;
    public static final long   STOP_TIME          = 100;

    private int                encoder_value      = 0;
    private long               encoder_time       = 0;
    private long               encoder_ticks      = 0;
    private long               encoder_speed      = 0;

    private int                encoder_step       = 0;

    private int getStepIndex(int value) {
        // Rount trip cycle is 0, 2, 3, 1
        switch (value) {
            case 2:
                return 1;
            case 3: 
                return 2;
            case 1: 
                return 3;
            case 0:
            default:
                return 0;
        }
    }
    
    private int getInitialStep(int oldValue, int newValue) {
        int oldStep = getStepIndex(oldValue);
        int newStep = getStepIndex(newValue);
        int step = newStep - oldStep;
        if (step == 3) {
            return -1;
        }
        if (step == -3) {
            return 1;
        }
        return step;
    }
    
    private int getFastStep(int oldValue, int newValue, int direction) {
        int oldStep = getStepIndex(oldValue);
        int newStep = getStepIndex(newValue);
        int step = newStep - oldStep;
        if (direction < 0 && step > 0) {
            step -= 4;
        }
        if (direction > 0 && step < 0) {
            step += 4;
        }
        return step;
    }
 
    public void loop(int newValue, long newTime) {
        if (encoder_time == newTime) {
            return;
        }

        if (newValue != encoder_value) {
            if (encoder_speed == 0) {
                // Start into one direction
                encoder_step = getInitialStep(encoder_value, newValue);
            } else {
                // Keep direction
                encoder_step = getFastStep(encoder_value, newValue, encoder_step);
            }

            encoder_ticks += encoder_step;

            long timeDelta = newTime - encoder_time;
            long t = ENCODER_RESOLUTION * timeDelta;
            encoder_speed = encoder_step * REFERENCE_TURN * MILLI_PER_SECOND / t;

            encoder_time = newTime;
            encoder_value = newValue;
        } else if (newTime > encoder_time + STOP_TIME) {
            encoder_value = newValue;
            encoder_time = newTime;
            encoder_speed = 0;
        }
    }

    public long getEncoderSpeed() {
        return encoder_speed;
    }

    public long getEncoderTicks() {
        return encoder_ticks;
    }

    public long getEncoderTime() {
        return encoder_time;
    }

    public int getEncoderValue() {
        return encoder_value;
    }

    public boolean isEncoderOverspeed() {
        return false;
    }
}
