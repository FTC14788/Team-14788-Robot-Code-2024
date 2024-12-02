package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.subsystems.ElbowSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OctoquadManager;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StateMachine;

abstract public class OpModeContainer extends CommandOpMode {

    protected OctoquadManager octoquadManager;
    protected MecanumSubsystem mecanumSubsystem;
    protected PivotSubsystem pivotSubsystem;
    protected ElbowSubsystem elbowSubsystem;
    protected ExtensionSubsystem extensionSubsystem;
    protected LiftSubsystem liftSubsystem;
    protected IntakeSubsystem intakeSubsystem;

    protected StateMachine state;


    protected GamepadEx driverController;
    protected GamepadEx operatorController;

    protected boolean fieldOrientedTeleop = false;


    protected void initHardware(Enum OpModeType) {
        OctoquadManager.createInstance(hardwareMap);
        MecanumSubsystem.createInstance(hardwareMap);
        PivotSubsystem.createInstance(hardwareMap);
        ElbowSubsystem.createInstance(hardwareMap);

        octoquadManager = OctoquadManager.getInstance();
        mecanumSubsystem = MecanumSubsystem.getInstance();
        pivotSubsystem = PivotSubsystem.getInstance();
        elbowSubsystem = ElbowSubsystem.getInstance();
        extensionSubsystem = new ExtensionSubsystem(hardwareMap);
        liftSubsystem = new LiftSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);

        StateMachine.createInstance(pivotSubsystem, elbowSubsystem, extensionSubsystem, liftSubsystem);

        register(mecanumSubsystem, octoquadManager, pivotSubsystem, elbowSubsystem, extensionSubsystem, liftSubsystem, intakeSubsystem);

        driverController = new GamepadEx(gamepad1);
        operatorController = new GamepadEx(gamepad2);
    }


    public enum Alliance{
        RED,
        BLUE;
        public Pose2d mirror(Pose2d input) {
            Pose2d mirroredPose = new Pose2d(
                    144 - input.getX(),
                    input.getY(),
                    new Rotation2d(input.getHeading())
            );
            return this == BLUE ? input : mirroredPose;
        }
    }

    public enum OpModeType{
        AUTON,
        TELEOP
    }
}
