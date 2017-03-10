package com.razorcat.elevator.test;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class Importer {

    private final List<Long> values;

    private long             framerate;

    private void readLine(String line) {
        if (line.isEmpty()) {
            return;
        }

        if (line.startsWith("#")) {
            readComment(line);
        } else {
            readValue(line);
        }
    }

    private void readComment(String line) {
        if (line.contains("FrameLength:")) {
            int i1 = line.indexOf(':') + 1;
            int i2 = line.indexOf("uSec");
            String value = line.substring(i1, i2).trim();
            framerate = Long.parseLong(value) / 1000;
        }
    }

    private void readValue(String line) {
        if (line.contains(";")) {
            int i = line.indexOf(';');
            String value = line.substring(0, i).trim();
            values.add(Long.parseLong(value));
        }
    }

    public Importer(Path path) throws IOException {
        this.values = new ArrayList<>();

        Objects.requireNonNull(path);
        Files.lines(path).forEach(s -> readLine(s));
    }

    public long getFramerate() {
        return framerate;
    }

    public List<Long> getValues() {
        return new ArrayList<>(values);
    }

    public int getNumberOfValues() {
        return values.size();
    }
}
