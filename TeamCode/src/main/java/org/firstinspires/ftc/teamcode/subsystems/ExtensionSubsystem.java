package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.constants.ElbowConstants;
import org.firstinspires.ftc.teamcode.constants.ExtensionConstants;
import org.firstinspires.ftc.teamcode.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.util.PIDFController;

import java.util.function.DoubleSupplier;

import kotlin.Unit;

public class ExtensionSubsystem extends SubsystemBase {

    private final DcMotorEx extension;

    private final double revolutionsPerMillimeter = (5.8 / 696.0);
    private final double ticksPerRevolution = 537.7;

    private final DoubleSupplier currentPosition;
    private final DoubleSupplier supplyCurrent;

    private double overTimeStart = 0;
    private double overTime = 0;
    private double inches = 0.25;
    private double targetPosition = 0;

    public boolean homing = true;
    private double currentCutoff = 2;

    private double lastReset;

    private double currentTime;

    private PIDFCoefficients pidfCoefficients = new PIDFCoefficients(13, 0, 0, 0);

    public ExtensionSubsystem(HardwareMap hardwareMap) {
        extension = hardwareMap.get(DcMotorEx.class, "extension");

        extension.setDirection(DcMotorSimple.Direction.REVERSE);
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        currentTime = System.currentTimeMillis();
        lastReset = Double.NEGATIVE_INFINITY;

        currentPosition = (() -> extension.getCurrentPosition());
        supplyCurrent = (() -> extension.getCurrent(CurrentUnit.AMPS));
    }

    @Override
    public void periodic() {
        currentTime = System.currentTimeMillis();
        targetPosition = ((inches * 25.4) * revolutionsPerMillimeter) * (ticksPerRevolution / 1);


        if (inches == 0.25) {
            if (supplyCurrent.getAsDouble() >= currentCutoff && extension.getTargetPosition() <= ((0.25 * 25.4) * revolutionsPerMillimeter) * (ticksPerRevolution / 1)) {
                homing = true;
            }
        }

        if (homing && supplyCurrent.getAsDouble() <= currentCutoff) {
            extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            extension.setPower(-0.6);
            System.out.println("DEALUFDHLKJDHF");
        }

        if (supplyCurrent.getAsDouble() > currentCutoff && homing) {
            if (Math.abs(currentTime - lastReset) >= 500) {
                System.out.println("MOKEY MONK");
                extension.setPower(1);
                extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                homing = false;
                extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extension.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfCoefficients);

                lastReset = System.currentTimeMillis();
            }

        }



        if (!homing) {
//            System.out.println("OVER CURRENT");

            extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extension.setPower(1);
            extension.setTargetPosition((int) Math.round(targetPosition));

        }
    }

    public void setManualInputPosition(Double input) {
        inches += input / 2.0;

        if (inches > ExtensionConstants.MAX_EXTENSION_SPEEDY) {
            inches = ExtensionConstants.MAX_EXTENSION_SPEEDY;
        }

        if (inches < 0.25) {
            inches = 0.25;

        }


    }

    public double getCurrentPosition() {
        return extension.getCurrentPosition();
    }

    public double getTargetPosition() {
        return extension.getTargetPosition();
    }

    public void setTeleopDefaultCommand() {
        setDefaultCommand(goToPositionCommand(() -> 0.25));
    }

    public void findOffset(double currentCutoff) {
        this.currentCutoff = currentCutoff;
        homing = true;
    }

    public Command goToPositionCommand(DoubleSupplier position) {
        return new RunCommand(() -> goToPosition(position.getAsDouble()), this);
    }

    private void goToPosition(double position) {
        if (position > ExtensionConstants.MAX_EXTENSION_SPEEDY) {
            position = ExtensionConstants.MAX_EXTENSION_SPEEDY;
        }

        double revolutionsPerMillimeter = (5.8 / 696.0);
        double ticksPerRevolution = 537.7;
        targetPosition = ((position * 25.4) * revolutionsPerMillimeter) * (ticksPerRevolution / 1);

    }
}
