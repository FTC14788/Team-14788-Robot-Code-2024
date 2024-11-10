package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.ExtensionConstants;
import org.firstinspires.ftc.teamcode.constants.SuperstructureConstants;
import org.firstinspires.ftc.teamcode.subsystems.StateMachine;
import org.firstinspires.ftc.teamcode.util.IkController;
import org.firstinspires.ftc.teamcode.util.OpModeContainer;

import java.util.function.DoubleSupplier;

@TeleOp(name = "Main")
public class MainTeleopOpMode extends OpModeContainer {

    @Override
    public void initialize() {
        initHardware(OpModeType.TELEOP);
        mecanumSubsystem.startTeleopDrive();

        pivotSubsystem.setTeleopDefaultCommand();
        elbowSubsystem.setTeleopDefaultCommand();
        extensionSubsystem.setTeleopDefaultCommand();
        intakeSubsystem.setTeleopDefaultCommand();
        liftSubsystem.setTeleopDefaultCommand();

        new GamepadButton(operatorController, GamepadKeys.Button.B).whileHeld(
            scoringPositionManual()
        );

        new GamepadButton(operatorController, GamepadKeys.Button.RIGHT_BUMPER).whileHeld(
            intakeWithStick(() -> -operatorController.getRightY())
        );

        new GamepadButton(operatorController, GamepadKeys.Button.LEFT_BUMPER).whileHeld(
            intakeSubsystem.runWithPowerCommand(() -> -1)
        );

        new GamepadButton(operatorController, GamepadKeys.Button.Y).whenPressed(climb());

        new GamepadButton(driverController, GamepadKeys.Button.X).whenPressed(
            new ParallelDeadlineGroup(
                new WaitCommand(150),
                new RunCommand(() -> switchDriveMode())
            )
        );
    }

    @Override
    public void run() {
        super.run();

        mecanumSubsystem.setTeleOpMovementVectors(driverController.getLeftY(), -driverController.getLeftX(), -driverController.getRightX(), true);
//        mecanumSubsystem.setTeleOpMovementVectors(driverController.getLeftY(), -driverController.getLeftX(), -driverController.getRightX(), false);
    }

    private void switchDriveMode() {
        if (fieldOrientedTeleop) {
            fieldOrientedTeleop = false;
        } else {
            fieldOrientedTeleop = true;
        }
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
            extensionSubsystem.goToPositionCommand(() -> IkController.intake((stick.getAsDouble() * SuperstructureConstants.STICK_INTAKE_DIST) + SuperstructureConstants.STICK_INTAKE_IDLE_DIST)[2]),
            intakeSubsystem.runWithPowerCommand(() -> 1)
        );
    }

    private Command scoringPositionManual() {
        return new ParallelCommandGroup(
            pivotSubsystem.goToPositionCommand(() -> 0.2),
            elbowSubsystem.goToPositionCommand(() -> -0.03),
            extensionSubsystem.goToPositionCommand(() -> 15)
        );
    }

    private Command climb() {
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                new WaitUntilCommand(StateMachine.getInstance().liftOnTarget()),
                liftSubsystem.goToPositionCommand(() -> ExtensionConstants.MAX_EXTENSION),
                pivotSubsystem.goToPositionCommand(() -> 0.3),
                elbowSubsystem.goToPositionCommand(() -> -0.05)
            ),
            new ParallelDeadlineGroup(
                new WaitCommand(2000),
                pivotSubsystem.goToPositionCommand(() -> 0.4),
                elbowSubsystem.goToPositionCommand(() -> -0.05),
                extensionSubsystem.goToPositionCommand(() -> 5)
            ),
            new ParallelDeadlineGroup(
                new WaitUntilCommand(StateMachine.getInstance().extensionOnTarget()),
                pivotSubsystem.goToPositionCommand(() -> 0.25),
                elbowSubsystem.goToPositionCommand(() -> 0)
            )
        );
    }

}
