package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.PathPlan;
import org.firstinspires.ftc.teamcode.commands.PathfolowingCommand;
import org.firstinspires.ftc.teamcode.paths.Paths;
import org.firstinspires.ftc.teamcode.util.OpModeContainer;

import java.util.HashMap;
import java.util.Map;

@Autonomous(name = "Monkey Monkey Monkey")
public class PushAuton extends OpModeContainer {
    private boolean hasStarted = false;

    @Override
    public void initialize() {
        initHardware(OpModeType.TELEOP);
//        mecanumSubsystem.startTeleopDrive();

//        pivotSubsystem.setTeleopDefaultCommand();
//        elbowSubsystem.setTeleopDefaultCommand();
//        extensionSubsystem.setTeleopDefaultCommand();
        intakeSubsystem.setTeleopDefaultCommand();
        liftSubsystem.setTeleopDefaultCommand();


    }

    @Override
    public void run() {
        super.run();

        if (hasStarted == false) {
            mecanumSubsystem.followPath(Paths.spikeThreePieceScorePush());
            hasStarted = true;
        }
    }

}
