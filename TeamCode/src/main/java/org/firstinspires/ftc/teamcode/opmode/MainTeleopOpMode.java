package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryInternal;
import org.firstinspires.ftc.teamcode.constants.ExtensionConstants;
import org.firstinspires.ftc.teamcode.util.OpModeContainer;

@TeleOp(name = "FieldOriented")
public class MainTeleopOpMode extends OpModeContainer {

    @Override
    public void initialize() {
        initHardware(OpModeType.TELEOP);
        mecanumSubsystem.startTeleopDrive();

        pivotSubsystem.setTeleopDefaultCommand();
        elbowSubsystem.setTeleopDefaultCommand();
        extensionSubsystem.setTeleopDefaultCommand();

        new GamepadButton(operatorController, GamepadKeys.Button.B).whileHeld(
            new ParallelCommandGroup(
                pivotSubsystem.goToPositionCommand(() -> 0.29),
                elbowSubsystem.goToPositionCommand(() -> 0),
                extensionSubsystem.goToPositionCommand(() -> ExtensionConstants.MAX_EXTENSION)
            )
        );
    }

    @Override
    public void run() {
        super.run();

//        mecanumSubsystem.setTeleOpMovementVectors(driverController.getLeftY(), -driverController.getLeftX(), -driverController.getRightX(), true);
        mecanumSubsystem.setTeleOpMovementVectors(driverController.getLeftY(), -driverController.getLeftX(), -driverController.getRightX(), false);
    }
}
