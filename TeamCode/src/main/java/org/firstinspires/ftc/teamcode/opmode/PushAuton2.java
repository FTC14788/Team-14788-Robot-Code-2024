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

@Autonomous(name = "Banana Banana Banana")
public class PushAuton2 extends OpModeContainer {
    private boolean hasStarted = false;

    private PathPlan myPlan;

    @Override
    public void initialize() {
        initHardware(OpModeType.TELEOP);
//        mecanumSubsystem.startTeleopDrive();

//        pivotSubsystem.setTeleopDefaultCommand();
//        elbowSubsystem.setTeleopDefaultCommand();
//        extensionSubsystem.setTeleopDefaultCommand();
        intakeSubsystem.setTeleopDefaultCommand();
        liftSubsystem.setTeleopDefaultCommand();

        PathfolowingCommand path = new PathfolowingCommand(Paths.parkAtSubmerisble());

        HashMap<Double, Command> hashMap = new HashMap<Double, Command>();
        hashMap.put(6.75, elbowSubsystem.setPower());

        myPlan = new PathPlan(path, hashMap);
    }

    @Override
    public void run() {
        super.run();

        if (hasStarted == false) {
            myPlan.schedule();
            hasStarted = true;
        }
    }

}
