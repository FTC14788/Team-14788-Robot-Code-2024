package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryInternal;
import org.firstinspires.ftc.teamcode.util.OpModeContainer;

public class RobotOrientedTeleopOpMode extends OpModeContainer {

    @Override
    public void initialize() {
        initHardware(OpModeType.TELEOP);
        mecanumSubsystem.startTeleopDrive();

//        new GamepadButton(driverController, GamepadKeys.Button.A).whenPressed(() -> pivotSubsystem.findOffset(3.5));
//        new GamepadButton(driverController, GamepadKeys.Button.B).whenPressed(() -> elbowSubsystem.findOffset(3.5));
//        new GamepadButton(driverController, GamepadKeys.Button.Y).whileHeld(() -> pivotSubsystem.goToPosition(-0.2));
    }

    @Override
    public void run() {
        super.run();

        mecanumSubsystem.setTeleOpMovementVectors(driverController.getLeftY(), -driverController.getLeftX(), -driverController.getRightX(), true);
//        mecanumSubsystem.setTeleOpMovementVectors(driverController.getLeftY(), -driverController.getLeftX(), -driverController.getRightX(), false);
    }
}
