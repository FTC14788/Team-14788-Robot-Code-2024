package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
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
    private static ElbowSubsystem instance;

    private final DcMotorEx elbow;

    private final DoubleSupplier remoteSensorAbsolute;
    private final DoubleSupplier currentPosition;
    private final DoubleSupplier supplyCurrent;

    private double encoderOffset = 0;
    public boolean homing = false;
    private double currentCutoff = 1.75;

    private double manualInputPower = 0;
    private double targetPosition = 0;

    private PIDFController pivotPIDF;
    private CustomPIDFCoefficients pidfCoefficients = new CustomPIDFCoefficients(0.005, 0.000, 0, 0);

    public ElbowSubsystem(HardwareMap hardwareMap) {

        elbow = hardwareMap.get(DcMotorEx.class, "elbow");

        remoteSensorAbsolute = (() -> OctoquadManager.getInstance().getPosition(4));
        currentPosition = (() -> applyOffset(encoderOffset));
        supplyCurrent = (() -> elbow.getCurrent(CurrentUnit.AMPS));

        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow.setDirection(DcMotorSimple.Direction.FORWARD);

        pivotPIDF = new PIDFController(pidfCoefficients);
    }

    public static void createInstance(HardwareMap hardwareMap) {
        if (instance == null) {
            instance = new ElbowSubsystem(hardwareMap);
        }
    }

    public static ElbowSubsystem getInstance() {
        if (instance != null) {
            return instance;
        } else {
            System.out.println("Elbow Subsystem not instantiated");
            return null;
        }
    }

    @Override
    public void periodic() {
//        System.out.println("Current: " + currentPosition.getAsDouble() + " Target: " + targetPosition);
//        System.out.println(supplyCurrent.getAsDouble());

        elbow.setDirection(DcMotorSimple.Direction.FORWARD);
        if (homing) {
            elbow.setPower(1.0);
        }

        if (supplyCurrent.getAsDouble() > currentCutoff && homing) {
            elbow.setPower(0);
            encoderOffset = getAbsolutePosition();
            homing = false;
        }

        if (!homing) {
            elbow.setPower(manualInputPower);
//            elbow.setPower(calculateOutput((int) Math.round(targetPosition)));
        }
    }

    public void setManualInputPower(Double input) {
        manualInputPower = input;
    }

    public boolean isHoming() {
        return homing;
    }

    public double getCurrentPosition() {
        return currentPosition.getAsDouble();
    }

    public double getTargetPosition() {
        return targetPosition;
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

    public Command setPower() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> setManualInputPower(1.0)),
                new WaitCommand(2500),
                new InstantCommand(() -> setManualInputPower(0.0))
        );
    }

    public void setTeleopDefaultCommand() {}

    public Command goToPositionCommand(DoubleSupplier position) {
        return new RunCommand(() -> goToPosition(position.getAsDouble()), this);
    }

    private void goToPosition(double position) {
        targetPosition = (position * 8192) - (ElbowConstants.ELBOW_TO_STRAIGHT * 8192);
    }
}
