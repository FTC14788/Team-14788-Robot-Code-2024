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
import org.firstinspires.ftc.teamcode.localization.Pose;
import org.firstinspires.ftc.teamcode.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.paths.Paths;
import org.firstinspires.ftc.teamcode.subsystems.StateMachine;
import org.firstinspires.ftc.teamcode.util.Deadband;
import org.firstinspires.ftc.teamcode.util.IkController;
import org.firstinspires.ftc.teamcode.util.OpModeContainer;

import java.util.function.DoubleSupplier;

@TeleOp(name = "Main")
public class MainTeleopOpMode extends OpModeContainer {

    @Override
    public void initialize() {
        initHardware(OpModeType.TELEOP);
        mecanumSubsystem.startTeleopDrive();

//        pivotSubsystem.setTeleopDefaultCommand();
//        elbowSubsystem.setTeleopDefaultCommand();
//        extensionSubsystem.setTeleopDefaultCommand();
        intakeSubsystem.setTeleopDefaultCommand();
        liftSubsystem.setTeleopDefaultCommand();

//        new GamepadButton(operatorController, GamepadKeys.Button.B).whileHeld(
//                () -> mecanumSubsystem.followPath(Paths.spikeThreePieceScorePush(), false)
//        );
        new GamepadButton(operatorController, GamepadKeys.Button.Y).whileHeld(
                new RunCommand(() -> liftSubsystem.setManualInputPower(1.0), liftSubsystem)
        );

        new GamepadButton(operatorController, GamepadKeys.Button.A).whileHeld(
                new RunCommand(() -> liftSubsystem.setManualInputPower(-1.0), liftSubsystem)
        );

        new GamepadButton(operatorController, GamepadKeys.Button.X).whenPressed(
                () -> mecanumSubsystem.setPose(new Pose(-9, 10, Math.PI / 2))
        );


        new GamepadButton(operatorController, GamepadKeys.Button.RIGHT_BUMPER).whileHeld(
                intakeSubsystem.runWithPowerCommand(() -> 1)
        );

        new GamepadButton(operatorController, GamepadKeys.Button.LEFT_BUMPER).whileHeld(
            intakeSubsystem.runWithPowerCommand(() -> -1)
        );


    }

    @Override
    public void run() {
        super.run();

        mecanumSubsystem.setTeleOpMovementVectors(Math.pow(Deadband.apply(driverController.getLeftX(), driverController.getLeftY(), true, 0.01, 0.125), 3), Math.pow(-Deadband.apply(driverController.getLeftX(), driverController.getLeftY(), false, 0.01, 0.125), 3), Deadband.apply(-driverController.getRightX(), 0, false, 0.01, 0), true);
        pivotSubsystem.setManualInputPower(-operatorController.getLeftY());
        elbowSubsystem.setManualInputPower(-operatorController.getRightY());
        extensionSubsystem.setManualInputPosition(operatorController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
        extensionSubsystem.setManualInputPosition(-operatorController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

//        mecanumSubsystem.setTeleOpMovementVectors(driverController.getLeftY(), -driverController.getLeftX(), -driverController.getRightX(), false);
    }

    private void switchDriveMode() {
        if (fieldOrientedTeleop) {
            fieldOrientedTeleop = false;
        } else {
            fieldOrientedTeleop = true;
        }
    }

    private void zero() {
        pivotSubsystem.homing = true;
        elbowSubsystem.homing = true;
        extensionSubsystem.homing = true;
    }

    private void bumpUp() {
        SuperstructureConstants.INTAKE_HEIGHT += 0.15;
    }

    private void bumpDown() {
        SuperstructureConstants.INTAKE_HEIGHT += 0.15;
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
            elbowSubsystem.goToPositionCommand(() -> 0.0),
            extensionSubsystem.goToPositionCommand(() -> 20)
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
