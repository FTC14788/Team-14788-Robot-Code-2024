package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

public class Deadband {

    public static double apply(double x, double y, boolean isY, double deadband, double feedForward) {
        Translation2d fromSticks = new Translation2d(x, y);
        double newNorm = (fromSticks.getNorm() - deadband) / (1 - deadband);
        double newTheta = Math.atan2(y, x);
        double newX = newNorm * Math.cos(newTheta);
        double newY = newNorm * Math.sin(newTheta);

        if (fromSticks.getNorm() <= deadband) {
            return 0;
        }

        if (newY > 0) {
            newY += feedForward * (1 - newNorm);
        } else {
            newY -= feedForward * (1 - newNorm);
        }

        if (newX > 0) {
            newX += feedForward * (1 - newNorm);
        } else {
            newX -= feedForward * (1 - newNorm);
        }

        if (isY) {
            return newY;
        } else {
            return newX;
        }
    }
}
