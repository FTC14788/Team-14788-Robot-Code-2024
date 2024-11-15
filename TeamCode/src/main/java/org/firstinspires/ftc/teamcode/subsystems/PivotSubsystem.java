package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.constants.PivotConstants;
import org.firstinspires.ftc.teamcode.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.util.PIDFController;

import java.util.function.DoubleSupplier;

public class PivotSubsystem extends SubsystemBase {
    private static PivotSubsystem instance;

    private final DcMotorEx pivot;

    private final DoubleSupplier remoteSensorAbsolute;
    private final DoubleSupplier currentPosition;
    private final DoubleSupplier supplyCurrent;

    private double encoderOffset = 0;
    public boolean homing = true;
    private double currentCutoff = 4;

    private double targetPosition = 0;

    private PIDFController pivotPIDF;
    private CustomPIDFCoefficients pidfCoefficients = new CustomPIDFCoefficients(0.006, 0.0002, 0.000, 0);

    public PivotSubsystem(HardwareMap hardwareMap) {

        pivot = hardwareMap.get(DcMotorEx.class, "pivot");

        remoteSensorAbsolute = (() -> OctoquadManager.getInstance().getPosition(3));
        currentPosition = (() -> applyOffset(encoderOffset));
        supplyCurrent = (() -> pivot.getCurrent(CurrentUnit.AMPS));

        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setDirection(DcMotorSimple.Direction.REVERSE);

        pivotPIDF = new PIDFController(pidfCoefficients);
    }

    public static void createInstance(HardwareMap hardwareMap) {
        if (instance == null) {
            instance = new PivotSubsystem(hardwareMap);
        }
    }

    public static PivotSubsystem getInstance() {
        if (instance != null) {
            return instance;
        } else {
            System.out.println("Pivot Subsystem not instantiated");
            return null;
        }
    }

    @Override
    public void periodic() {
        pivot.setDirection(DcMotorSimple.Direction.REVERSE);
//        System.out.println("Current: " + currentPosition.getAsDouble() + " Target: " + targetPosition);

        if (homing) {
            pivot.setPower(-0.6);
        }

        if (supplyCurrent.getAsDouble() > currentCutoff && homing) {
            pivot.setPower(0);
            encoderOffset = getAbsolutePosition();
            homing = false;
        }

        if (!homing) {
            if (StateMachine.getInstance().elbowHoming().getAsBoolean()) {
                goToPosition(0.13);
            }
            pivot.setPower(calculateOutput((int) Math.round(targetPosition)));
        }
    }

    public double getCurrentPosition() {
        return currentPosition.getAsDouble();
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public void setTeleopDefaultCommand() {
        setDefaultCommand(goToPositionCommand(() -> 0.25));
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

    public Command goToPositionCommand(DoubleSupplier position) {
        return new RunCommand(() -> goToPosition(position.getAsDouble()), this);
    }

    private void goToPosition(double position) {
        targetPosition = (position * 8192) - (PivotConstants.PIVOT_TO_HORIZONTAL * 8192);
    }
}
