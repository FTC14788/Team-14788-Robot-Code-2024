package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;

import java.util.HashMap;
import java.util.NavigableMap;

public class PathfolowingCommand extends CommandBase {
    private PathChain paths;

    private final HashMap<Integer, Double> tValues = new HashMap<Integer, Double>();

    public PathfolowingCommand(PathChain paths) {
        this.paths = paths;

        addRequirements(MecanumSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        for (int i = 0; i < paths.size(); i++) {
            tValues.put(i, 0.0);
        }

        MecanumSubsystem.getInstance().followPath(paths, false);
    }

    @Override
    public void execute() {
        if (MecanumSubsystem.getInstance().atParametricEnd()) {
            tValues.put((int) MecanumSubsystem.getInstance().getCurrentPathNumber(), 1.0);
        } else {
            tValues.put((int) MecanumSubsystem.getInstance().getCurrentPathNumber(), MecanumSubsystem.getInstance().getCurrentTValue());
        }
    }

    @Override
    public boolean isFinished() {
        if (MecanumSubsystem.getInstance().getCurrentPathNumber() == paths.size() - 1) {
            return MecanumSubsystem.getInstance().atParametricEnd();
        } else {
            return false;
        }
    }

    public HashMap<Integer, Double> getPathStamp() {
        return tValues;
    }
}