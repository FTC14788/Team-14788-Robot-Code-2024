package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;

public class PathPlan extends CommandBase {
    private PathfolowingCommand pathFollowingCommand;
    private volatile HashMap<Double, Command> subsystemCommands;

    public PathPlan(PathfolowingCommand pathFollowingCommand, HashMap<Double, Command> subsystemCommands) {
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
        LinkedList<Double> remove = new  LinkedList<Double>();

        for (Map.Entry<Double, Command> subsystemCommand : subsystemCommands.entrySet()) {
            if (subsystemCommand.getKey().intValue() == (int) MecanumSubsystem.getInstance().getCurrentPathNumber()) {
                if (subsystemCommand.getKey() - subsystemCommand.getKey().intValue() >= pathFollowingCommand.getPathStamp().get(subsystemCommand.getKey().intValue())) {
                    CommandScheduler.getInstance().schedule(subsystemCommand.getValue());
                    System.out.println("## Autocommand " + pathFollowingCommand.getName() + " scheduled ##");
                    remove.add(subsystemCommand.getKey());
                }
            } else {
                return;
            }
        }

        for (int i = 0; i < remove.size(); i++) {
            subsystemCommands.remove(remove.get(i));
        }
    }
}
