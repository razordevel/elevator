package com.razorcat.elevator.test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import org.junit.BeforeClass;
import org.junit.Test;

public class NewEncoderFileTest {

    private static Importer encoder_a;
    private static Importer encoder_b;
    private static Importer encoder_speed;

    private static void write(String... values) {
        StringBuilder sb = new StringBuilder();
        for (String s : values) {
            sb.append(s);
            sb.append(';');
            sb.append('\t');
        }
        System.out.println(sb.toString());
    }

    private static void write(long time, int value, long a, long b, long speed, long encoderSpeed, boolean overspeed) {
        write(Long.toString(time),
              Integer.toString(value),
              Long.toString(a),
              Long.toString(b),
              Long.toString(encoderSpeed),
              Long.toString(speed),
              overspeed ? "OVERSPEED" : "-");
    }

    @BeforeClass
    public static void setUp() throws IOException {
        encoder_a = new Importer(Paths.get("test", "encoder_a.txt"));
        encoder_b = new Importer(Paths.get("test", "encoder_b.txt"));
        encoder_speed = new Importer(Paths.get("test", "encoder_speed.txt"));
    }

    @Test
    public void testFileInput() {
        NewEncoder encoder = new NewEncoder();

        assertEquals(3, encoder_a.getFramerate());
        assertEquals(3, encoder_b.getFramerate());
        assertEquals(3, encoder_speed.getFramerate());

        assertEquals(encoder_a.getNumberOfValues(), encoder_b.getNumberOfValues());
        assertEquals(encoder_a.getNumberOfValues(), encoder_speed.getNumberOfValues());

        List<Long> valuesA = encoder_a.getValues();
        List<Long> valuesB = encoder_b.getValues();
        List<Long> valuesSpeed = encoder_speed.getValues();

        List<Integer> changeValues = new LinkedList<Integer>();

        int lastValue = 0;
        for (int i = 0; i < valuesA.size(); i++) {
            long time = 3 * i;

            long a = valuesA.get(i);
            long b = valuesB.get(i);
            long speed = valuesSpeed.get(i);

            int value = (a != 0 ? 1 : 0) + (b != 0 ? 2 : 0);

            if (value != lastValue) {
                changeValues.add(i);
            }

            encoder.loop(value, time);
            long encoderSpeed = encoder.getEncoderSpeed();
            boolean overspeed = encoder.isEncoderOverspeed();

            write(time, value, a, b, speed, encoderSpeed, overspeed);
            
            assertFalse(overspeed);

            lastValue = value;
        }

        Set<Integer> stepSet = new HashSet<>();
        for (int i = 1; i < changeValues.size(); i++) {
            int a = changeValues.get(i - 1);
            int b = changeValues.get(i);

            int steps = b - a;
            stepSet.add(steps);
        }
        
        List<Integer> stepList = new ArrayList<>(stepSet);
        Collections.sort(stepList);
        stepList.forEach(s -> System.out.println("--> " + s));
    }
}
