package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.util.PIDFController;

import java.util.function.DoubleSupplier;

public class ElbowSubsystem extends SubsystemBase {
    private final DcMotorEx elbow;

    private final DoubleSupplier remoteSensorAbsolute;
    private final DoubleSupplier currentPosition;
    private final DoubleSupplier supplyCurrent;

    private double encoderOffset = 0;

    private PIDFController elbowPIDF;
    private CustomPIDFCoefficients pidfCoefficients;

    public ElbowSubsystem(HardwareMap hardwareMap) {

        elbow = hardwareMap.get(DcMotorEx.class, "elbow");

        remoteSensorAbsolute = (() -> OctoquadManager.getInstance().getPosition(3));
        currentPosition = (() -> applyOffset(encoderOffset));
        supplyCurrent = (() -> elbow.getCurrent(CurrentUnit.AMPS));

        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elbowPIDF = new PIDFController(pidfCoefficients);
    }

    private double calculateOutput(int target) {
        elbowPIDF.setTargetPosition((double)target);
        elbowPIDF.updatePosition(currentPosition.getAsDouble());

        return elbowPIDF.runPIDF();
    }

    private double applyOffset(double offset) {
        return remoteSensorAbsolute.getAsDouble() - offset;
    }

    public void findOffset(double currentCutoff) {
        double start = remoteSensorAbsolute.getAsDouble();

        while (supplyCurrent.getAsDouble() < currentCutoff) {
            elbow.setPower(-0.7);
        }
        double end = remoteSensorAbsolute.getAsDouble();
        elbow.setPower(0);

        double offset = end - start;
        encoderOffset = offset;
    }
}
