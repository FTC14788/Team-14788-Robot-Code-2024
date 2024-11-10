package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.DoubleSupplier;

public class IntakeSubsystem extends SubsystemBase {

    private CRServo intake;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intake = hardwareMap.get(CRServo.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void periodic() {
    }

    public void setTeleopDefaultCommand() {
        setDefaultCommand(runWithPowerCommand(() -> 0));
    }

    public Command runWithPowerCommand(DoubleSupplier power) {
        return new RunCommand(() -> runWithPower(power.getAsDouble()), this);
    }

    private void runWithPower(double power) {
        intake.setPower(power);
    }
}
