package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.constants.ElbowConstants;
import org.firstinspires.ftc.teamcode.constants.ExtensionConstants;
import org.firstinspires.ftc.teamcode.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.util.PIDFController;

import java.util.function.DoubleSupplier;

import kotlin.Unit;

public class ExtensionSubsystem extends SubsystemBase {

    private final DcMotorEx extension;

    private final DoubleSupplier currentPosition;
    private final DoubleSupplier supplyCurrent;

    private double targetPosition = 0;

    public boolean homing = true;
    private double currentCutoff = 4;

    private PIDFCoefficients pidfCoefficients = new PIDFCoefficients(13, 0, 0, 0);

    public ExtensionSubsystem(HardwareMap hardwareMap) {
        extension = hardwareMap.get(DcMotorEx.class, "extension");

        extension.setDirection(DcMotorSimple.Direction.REVERSE);
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        currentPosition = (() -> extension.getCurrentPosition());
        supplyCurrent = (() -> extension.getCurrent(CurrentUnit.AMPS));
    }

    @Override
    public void periodic() {
//        System.out.println(extension.getCurrentPosition() + " " + extension.getTargetPosition());
        if (homing) {
            extension.setPower(-0.6);
        }

        if (supplyCurrent.getAsDouble() > currentCutoff && homing) {
            extension.setPower(0);
            extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            homing = false;
            extension.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfCoefficients);
        }

        if (!homing) {
            extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extension.setPower(1);
            extension.setTargetPosition((int) Math.round(targetPosition));
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
