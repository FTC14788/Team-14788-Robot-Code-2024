package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.util.PIDFController;

import java.util.function.DoubleSupplier;

public class PivotSubsystem extends SubsystemBase {

    private final DcMotorEx pivot;

    private final DoubleSupplier remoteSensorAbsolute;
    private final DoubleSupplier currentPosition;
    private final DoubleSupplier supplyCurrent;

    private double encoderOffset = 0;

    private PIDFController pivotPIDF;
    private CustomPIDFCoefficients pidfCoefficients = new CustomPIDFCoefficients(0.8, 0, 0, 0);

    public PivotSubsystem(HardwareMap hardwareMap) {

        pivot = hardwareMap.get(DcMotorEx.class, "pivot");

        remoteSensorAbsolute = (() -> OctoquadManager.getInstance().getPosition(3));
        currentPosition = (() -> applyOffset(encoderOffset));
        supplyCurrent = (() -> pivot.getCurrent(CurrentUnit.AMPS));

        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pivotPIDF = new PIDFController(pidfCoefficients);
    }

    @Override
    public void periodic() {
        System.out.println("Absolute: " + remoteSensorAbsolute.getAsDouble() + " Offset: " + encoderOffset);
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
        while (supplyCurrent.getAsDouble() < currentCutoff) {
            pivot.setPower(0.7);
        }
        pivot.setPower(0);
        try {
            Thread.sleep(250); // Let the arm come to a complete stop
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        encoderOffset = getAbsolutePosition();
    }

    public void goToPosition(double position) {
        position *= 8192;
        pivot.setPower(calculateOutput((int) Math.round(position)));
    }
}
