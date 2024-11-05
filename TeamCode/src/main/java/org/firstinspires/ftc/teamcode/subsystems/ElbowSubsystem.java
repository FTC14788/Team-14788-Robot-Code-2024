package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.constants.ElbowConstants;
import org.firstinspires.ftc.teamcode.constants.PivotConstants;
import org.firstinspires.ftc.teamcode.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.util.PIDFController;

import java.util.function.DoubleSupplier;

public class ElbowSubsystem extends SubsystemBase {

    private final DcMotorEx elbow;

    private final DoubleSupplier remoteSensorAbsolute;
    private final DoubleSupplier currentPosition;
    private final DoubleSupplier supplyCurrent;

    private double encoderOffset = 0;
    private boolean homing = true;
    private double currentCutoff = 4;

    private double targetPosition = 0;

    private PIDFController pivotPIDF;
    private CustomPIDFCoefficients pidfCoefficients = new CustomPIDFCoefficients(0.002, 0, 0, 0);

    public ElbowSubsystem(HardwareMap hardwareMap) {

        elbow = hardwareMap.get(DcMotorEx.class, "elbow");

        remoteSensorAbsolute = (() -> OctoquadManager.getInstance().getPosition(4));
        currentPosition = (() -> applyOffset(encoderOffset));
        supplyCurrent = (() -> elbow.getCurrent(CurrentUnit.AMPS));

        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow.setDirection(DcMotorSimple.Direction.REVERSE);

        pivotPIDF = new PIDFController(pidfCoefficients);
    }

    @Override
    public void periodic() {
        System.out.println("Current: " + currentPosition.getAsDouble() + " Target: " + targetPosition);

        elbow.setDirection(DcMotorSimple.Direction.REVERSE);
        if (homing) {
            elbow.setPower(0.8);
        }

        if (supplyCurrent.getAsDouble() > currentCutoff && homing) {
            elbow.setPower(0);
            encoderOffset = getAbsolutePosition();
            homing = false;
        }

        if (!homing) {
            elbow.setPower(calculateOutput((int) Math.round(targetPosition)));
        }
    }

    private double calculateOutput(int target) {
        pivotPIDF.setTargetPosition((double)target);
        pivotPIDF.updatePosition(currentPosition.getAsDouble());

        return pivotPIDF.runPIDF();
    }

    private double getAbsolutePosition() {
        return remoteSensorAbsolute.getAsDouble();
    }

    private double applyOffset(double offset) {
        return remoteSensorAbsolute.getAsDouble() - offset;
    }

    public void findOffset(double currentCutoff) {
        this.currentCutoff = currentCutoff;
        homing = true;
    }

    public void setTeleopDefaultCommand() {
        setDefaultCommand(goToPositionCommand(() -> -0.3));
    }

    public Command goToPositionCommand(DoubleSupplier position) {
        return new RunCommand(() -> goToPosition(position.getAsDouble()), this);
    }

    private void goToPosition(double position) {
        targetPosition = (position * 8192) - (ElbowConstants.ELBOW_TO_STRAIGHT * 8192);
    }
}
