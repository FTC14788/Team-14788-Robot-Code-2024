package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryInternal;
import org.firstinspires.ftc.teamcode.constants.ExtensionConstants;
import org.firstinspires.ftc.teamcode.constants.SuperstructureConstants;
import org.firstinspires.ftc.teamcode.util.IkController;
import org.firstinspires.ftc.teamcode.util.OpModeContainer;

import java.util.function.DoubleSupplier;

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

        new GamepadButton(operatorController, GamepadKeys.Button.RIGHT_BUMPER).whileHeld(
            intakeWithStick(() -> -operatorController.getRightY())
        );
    }

    @Override
    public void run() {
        super.run();

//        mecanumSubsystem.setTeleOpMovementVectors(driverController.getLeftY(), -driverController.getLeftX(), -driverController.getRightX(), true);
        mecanumSubsystem.setTeleOpMovementVectors(driverController.getLeftY(), -driverController.getLeftX(), -driverController.getRightX(), false);
    }

    private Command intakeWithDist(DoubleSupplier dist) {
        return new ParallelCommandGroup(
            pivotSubsystem.goToPositionCommand(() -> IkController.intake(dist.getAsDouble())[0]),
            elbowSubsystem.goToPositionCommand(() -> IkController.intake(dist.getAsDouble())[1]),
            extensionSubsystem.goToPositionCommand(() -> IkController.intake(dist.getAsDouble())[2])
        );
    }

    private Command intakeWithStick(DoubleSupplier stick) {
        return new ParallelCommandGroup(
            pivotSubsystem.goToPositionCommand(() -> IkController.intake((stick.getAsDouble() * SuperstructureConstants.STICK_INTAKE_DIST) + SuperstructureConstants.STICK_INTAKE_IDLE_DIST)[0]),
            elbowSubsystem.goToPositionCommand(() -> IkController.intake((stick.getAsDouble() * SuperstructureConstants.STICK_INTAKE_DIST) + SuperstructureConstants.STICK_INTAKE_IDLE_DIST)[1]),
            extensionSubsystem.goToPositionCommand(() -> IkController.intake((stick.getAsDouble() * SuperstructureConstants.STICK_INTAKE_DIST) + SuperstructureConstants.STICK_INTAKE_IDLE_DIST)[2])
        );
    }
}
