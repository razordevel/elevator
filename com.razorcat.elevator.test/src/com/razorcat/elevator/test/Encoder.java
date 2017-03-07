package com.razorcat.elevator.test;

public class Encoder {

    public static final int  ENCODER_RESOLUTION = 24;
    public static final long MILLI_PER_SECOND   = 1000;
    public static final long REFERENCE_TURN     = 100;
    public static final long STOP_TIME          = 100;

    private int              encoder_value      = 0;
    private long             encoder_time       = 0;
    private long             encoder_ticks      = 0;
    private long             encoder_speed      = 0;
    private boolean          encoder_overspeed  = false;

    public void loop(int newValue, long newTime) {
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

            if ((encoder_value == 0 && newValue == 1) || (encoder_value == 1 && newValue == 3)
                || (encoder_value == 3 && newValue == 2)
                || (encoder_value == 2 && newValue == 0)) {
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
        return encoder_overspeed;
    }
}
