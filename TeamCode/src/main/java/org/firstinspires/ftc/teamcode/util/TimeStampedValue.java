package org.firstinspires.ftc.teamcode.util;

public class TimeStampedValue<InterpolationRecord> {
    public double time;
    public InterpolationRecord value;

    public TimeStampedValue(double time, InterpolationRecord value) {
        this.time = time;
        this.value = value;
    }

    public double getTimestamp(){
        return this.time;
    }
}
