package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.pathGeneration.Vector;

public interface Derivator<T> {
    T calculate(double t1, double t2, T p1, T p2);
}
