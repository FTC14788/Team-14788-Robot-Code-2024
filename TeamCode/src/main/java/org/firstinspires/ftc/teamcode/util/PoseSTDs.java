package org.firstinspires.ftc.teamcode.util;

public class PoseSTDs {
    private double translationSTD;
    private double rotationSTD;

    public PoseSTDs(double translationSTD, double rotationSTD) {
        this.translationSTD = translationSTD;
        this.rotationSTD = rotationSTD;
    }

    public double getTranslationVariance() {
        return Math.pow(translationSTD, 2);
    }

    public double getRotationVariance() {
        return Math.pow(rotationSTD, 2);
    }

    public void setTranslationSTD(double newTranslationSTD) {
        translationSTD = newTranslationSTD;
    }

    public void setRotationSTD(double newRotationSTD) {
        rotationSTD = newRotationSTD;
    }

    public void setSTDs(PoseSTDs newPoseSTDs) {
        translationSTD = newPoseSTDs.translationSTD;
        rotationSTD = newPoseSTDs.rotationSTD;
    }
}
