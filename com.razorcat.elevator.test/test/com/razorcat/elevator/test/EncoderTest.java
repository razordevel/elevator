package com.razorcat.elevator.test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class EncoderTest {

    private static void write(String... values) {
        StringBuilder sb = new StringBuilder();
        for (String s : values) {
            sb.append(s);
            sb.append(';');
            sb.append('\t');
        }
        System.out.println(sb.toString());
    }

    private static void write(long time, Encoder encoder, Generator generator, long ticks) {
        write(Long.toString(time),
              Long.toString(encoder.getEncoderSpeed()),
              encoder.isEncoderOverspeed() ? "OVERSPEED" : "",
              generator.isA() ? "1" : "0",
              generator.isB() ? "1" : "0",
              Long.toString(ticks));
    }

    @Test
    public void testConstantBackward() {
        final int ticks = 25;

        Generator generator = new Generator();
        Encoder encoder = new Encoder();

        long moves = 0;
        long time;
        for (time = 1; time <= 1000; time++) {
            if (time % ticks == 0) {
                generator.move(-1);
                moves++;
            }
            int value = generator.getValue();
            encoder.loop(value, time);

            assertEquals(generator.getValue(), encoder.getEncoderValue());
            assertEquals(moves, encoder.getEncoderTicks());
            if (time < ticks) {
                assertEquals(0, encoder.getEncoderSpeed());
            } else {
                assertTrue(encoder.getEncoderSpeed() < 0);
            }
        }
    }

    @Test
    public void testConstantForward() {
        final int ticks = 25;

        Generator generator = new Generator();
        Encoder encoder = new Encoder();

        long moves = 0;
        long time;
        for (time = 1; time <= 1000; time++) {
            if (time % ticks == 0) {
                generator.move(1);
                moves++;
            }
            int value = generator.getValue();
            encoder.loop(value, time);

            assertEquals(generator.getValue(), encoder.getEncoderValue());
            assertEquals(moves, encoder.getEncoderTicks());
            if (time < ticks) {
                assertEquals(0, encoder.getEncoderSpeed());
            } else {
                assertTrue(encoder.getEncoderSpeed() > 0);
            }
        }
    }

    @Test
    public void testSpeedUp() {
        Generator generator = new Generator();
        Encoder encoder = new Encoder();

        long moves = 0;
        long time;
        long lastTime = 0;
        for (time = 1; time <= 1000; time++) {
            long ticks = getSpeedingTicks(time);
            if (time >= lastTime + ticks) {
                generator.move(1);
                moves++;
                lastTime = time;
            }
            int value = generator.getValue();
            encoder.loop(value, time);

            assertEquals(generator.getValue(), encoder.getEncoderValue());
            assertEquals(moves, encoder.getEncoderTicks());
            // write(time, encoder, generator, ticks);
        }
    }

    @Test
    public void testSlowingUp() {
        Generator generator = new Generator();
        Encoder encoder = new Encoder();

        long moves = 0;
        long time;
        long lastTime = 0;
        for (time = 1; time <= 1000; time++) {
            long ticks = getSlowingTicks(time);
            if (time >= lastTime + ticks) {
                generator.move(1);
                moves++;
                lastTime = time;
            }
            int value = generator.getValue();
            encoder.loop(value, time);

            assertEquals(generator.getValue(), encoder.getEncoderValue());
            assertEquals(moves, encoder.getEncoderTicks());
            write(time, encoder, generator, ticks);
        }
    }

    @Test
    public void testSpeedDown() {
        Generator generator = new Generator();
        Encoder encoder = new Encoder();

        long moves = 0;
        long time;
        long lastTime = 0;
        for (time = 1; time <= 1000; time++) {
            long ticks = getSpeedingTicks(time);
            if (time >= lastTime + ticks) {
                generator.move(-1);
                moves++;
                lastTime = time;
            }
            int value = generator.getValue();
            encoder.loop(value, time);

            assertEquals(generator.getValue(), encoder.getEncoderValue());
            assertEquals(moves, encoder.getEncoderTicks());
            // write(time, encoder, generator, ticks);
        }
    }

    @Test
    public void testSlowingDown() {
        Generator generator = new Generator();
        Encoder encoder = new Encoder();

        long moves = 0;
        long time;
        long lastTime = 0;
        for (time = 1; time <= 1000; time++) {
            long ticks = getSlowingTicks(time);
            if (time >= lastTime + ticks) {
                generator.move(-1);
                moves++;
                lastTime = time;
            }
            int value = generator.getValue();
            encoder.loop(value, time);

            assertEquals(generator.getValue(), encoder.getEncoderValue());
            assertEquals(moves, encoder.getEncoderTicks());
            // write(time, encoder, generator, ticks);
        }
    }

    private long getSpeedingTicks(long time) {
        return 100 - (time / 10);
    }

    private long getSlowingTicks(long time) {
        return time / 10;
    }
}
