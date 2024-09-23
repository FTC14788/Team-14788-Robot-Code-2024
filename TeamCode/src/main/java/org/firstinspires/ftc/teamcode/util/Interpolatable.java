package org.firstinspires.ftc.teamcode.util;

public interface Interpolatable<T> {
    T interpolate(T other, double t);
}
