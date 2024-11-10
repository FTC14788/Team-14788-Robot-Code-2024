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
import org.firstinspires.ftc.teamcode.constants.ExtensionConstants;

import java.util.function.DoubleSupplier;

public class LiftSubsystem extends SubsystemBase {

    private final DcMotorEx life;

    private final DoubleSupplier currentPosition;
    private final DoubleSupplier supplyCurrent;

    private double targetPosition = 0;

    private boolean homing = true;
    private double currentCutoff = 3;

    private PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0.1, 0, 0, 0);

    public LiftSubsystem(HardwareMap hardwareMap) {
        life = hardwareMap.get(DcMotorEx.class, "lift");

        life.setDirection(DcMotorSimple.Direction.FORWARD);

        currentPosition = (() -> life.getCurrentPosition());
        supplyCurrent = (() -> life.getCurrent(CurrentUnit.AMPS));
    }

    @Override
    public void periodic() {
//        System.out.println(extension.getCurrentPosition() + " " + extension.getTargetPosition());
        if (homing) {
            life.setPower(-0.6);
        }

        if (supplyCurrent.getAsDouble() > currentCutoff && homing) {
            life.setPower(0);
            life.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            homing = false;
            life.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfCoefficients);
        }

        if (!homing) {
            life.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            life.setPower(1);
            life.setTargetPosition((int) Math.round(targetPosition));
        }
    }

    public double getCurrentPosition() {
        return life.getCurrentPosition();
    }

    public double getTargetPosition() {
        return life.getTargetPosition();
    }

    public void setTeleopDefaultCommand() {
        setDefaultCommand(goToPositionCommand(() -> 0));
    }

    public void findOffset(double currentCutoff) {
        this.currentCutoff = currentCutoff;
        homing = true;
    }

    public Command goToPositionCommand(DoubleSupplier position) {
        return new RunCommand(() -> goToPosition(position.getAsDouble()), this);
    }

    private void goToPosition(double position) {
        if (position > ExtensionConstants.MAX_EXTENSION) {
            position = ExtensionConstants.MAX_EXTENSION;
        }
        targetPosition = ((position * 25.4) / 2) * (537.7 / 4);

    }
}
