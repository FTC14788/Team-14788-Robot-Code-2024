package org.firstinspires.ftc.teamcode.util;

import java.util.LinkedList;
import java.util.NavigableMap;
import java.util.TreeMap;

public class TimeInterpolatableBuffer<T extends Interpolatable<T>> {
    private final NavigableMap<Double, T> m_buffer = new TreeMap<>();

    // lets try to keep everything in seconds okay?
    private double maxDurationSeconds;
    
    public TimeInterpolatableBuffer(double maxDurationSeconds) {
        this.maxDurationSeconds = maxDurationSeconds;
    }

    public void addMeasurement(double time, T value) {
        // Add the new sample to the map
        m_buffer.put(time, value);

        // Remove old samples outside the maxDuration
        double oldestAllowedTime = time - maxDurationSeconds;
        m_buffer.headMap(oldestAllowedTime).clear();
    }

    public T getInterpolatedValue(double time) {
        // Handle cases where the buffer is empty or the requested time is out of range
        if (m_buffer.isEmpty()) {
            return null; // or throw an exception or return a default value
        }

        // Find the closest timestamps before and after the given time
        Double lowerKey = m_buffer.floorKey(time);
        Double upperKey = m_buffer.ceilingKey(time);

        if (lowerKey == null) {
            return m_buffer.get(upperKey); // time is before the first entry
        }
        if (upperKey == null) {
            return m_buffer.get(lowerKey); // time is after the last entry
        }
        if (lowerKey.equals(upperKey)) {
            return m_buffer.get(lowerKey); // exact match
        }

        // Perform interpolation using the interpolate method
        T lowerValue = m_buffer.get(lowerKey);
        T upperValue = m_buffer.get(upperKey);

        double ratio = (time - lowerKey) / (upperKey - lowerKey);
        return lowerValue.interpolate(upperValue, ratio);
    }

    public NavigableMap<Double, T> getRawBuffer() {
        return m_buffer;
    }
}
