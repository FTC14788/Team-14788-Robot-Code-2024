package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;

import java.util.HashMap;
import java.util.Map;

public class PathPlan extends CommandBase {
    private PathfolowingCommand pathFollowingCommand;
    private HashMap<Map.Entry<Integer, Double>, Command> subsystemCommands;

    public PathPlan(PathfolowingCommand pathFollowingCommand, HashMap<Map.Entry<Integer, Double>, Command> subsystemCommands) {
        this.pathFollowingCommand = pathFollowingCommand;
        this.subsystemCommands = subsystemCommands;

        addRequirements();
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().schedule(pathFollowingCommand);
    }

    @Override
    public void execute() {
        for (Map.Entry<Map.Entry<Integer, Double>, Command> subsystemCommand : subsystemCommands.entrySet()) {
            if (!pathFollowingCommand.getPathStamp().containsKey(subsystemCommand.getKey().getKey())) {
                return;
            }

            if (subsystemCommand.getKey().getKey() == MecanumSubsystem.getInstance().getCurrentPathNumber()) {
                if (subsystemCommand.getKey().getValue() >= pathFollowingCommand.getPathStamp().get(subsystemCommand.getKey().getKey())) {
                    CommandScheduler.getInstance().schedule(subsystemCommand.getValue());
                    System.out.println("## Autocommand " + pathFollowingCommand.getName() + " scheduled ##");
                    subsystemCommands.remove(subsystemCommand.getKey(), subsystemCommand.getValue());
                }
            } else {
                return;
            }
        }
    }
}
