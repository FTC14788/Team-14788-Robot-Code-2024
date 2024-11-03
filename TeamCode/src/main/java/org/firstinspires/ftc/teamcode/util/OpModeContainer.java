package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.subsystems.ElbowSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OctoquadManager;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;

abstract public class OpModeContainer extends CommandOpMode {

    protected OctoquadManager octoquadManager;
    protected MecanumSubsystem mecanumSubsystem;
    protected PivotSubsystem pivotSubsystem;
    protected ElbowSubsystem elbowSubsystem;


    protected GamepadEx driverController;
    protected GamepadEx operatorController;

    protected void initHardware(Enum OpModeType) {
        OctoquadManager.createInstance(hardwareMap);
        MecanumSubsystem.createInstance(hardwareMap);

        octoquadManager = OctoquadManager.getInstance();
        mecanumSubsystem = MecanumSubsystem.getInstance();
        pivotSubsystem = new PivotSubsystem(hardwareMap);
        elbowSubsystem = new ElbowSubsystem(hardwareMap);

        register(mecanumSubsystem, octoquadManager, pivotSubsystem);

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
