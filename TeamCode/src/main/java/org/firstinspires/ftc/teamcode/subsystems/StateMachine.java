package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.constants.ElbowConstants;
import org.firstinspires.ftc.teamcode.constants.ExtensionConstants;
import org.firstinspires.ftc.teamcode.constants.PivotConstants;

import java.util.function.BooleanSupplier;

public class StateMachine {
    private static StateMachine instance;

    private final BooleanSupplier pivotOnTarget;
    private final BooleanSupplier elbowOnTarget;
    private final BooleanSupplier extensionOnTarget;
    private final BooleanSupplier liftOnTarget;
    private final BooleanSupplier homingElbow;

    private StateMachine(PivotSubsystem pivotSubsystem, ElbowSubsystem elbowSubsystem, ExtensionSubsystem extensionSubsystem, LiftSubsystem liftSubsystem) {
        pivotOnTarget = () -> Math.abs(pivotSubsystem.getTargetPosition() - pivotSubsystem.getCurrentPosition()) < PivotConstants.TARGET_TOLERANCE;
        elbowOnTarget = () -> Math.abs(elbowSubsystem.getTargetPosition() - elbowSubsystem.getCurrentPosition()) < ElbowConstants.TARGET_TOLERANCE;
        extensionOnTarget = () -> Math.abs(extensionSubsystem.getTargetPosition() - extensionSubsystem.getCurrentPosition()) < ExtensionConstants.TARGET_TOLERANCE_SPEEDY;
        liftOnTarget = () -> Math.abs(liftSubsystem.getTargetPosition() - liftSubsystem.getCurrentPosition()) < ExtensionConstants.TARGET_TOLERANCE;
        homingElbow = () -> elbowSubsystem.isHoming();
    }

    public static void createInstance(PivotSubsystem pivotSubsystem, ElbowSubsystem elbowSubsystem, ExtensionSubsystem extensionSubsystem, LiftSubsystem liftSubsystem) {
        instance = new StateMachine(pivotSubsystem, elbowSubsystem, extensionSubsystem, liftSubsystem);
    }

    public static StateMachine getInstance() {
        if (instance != null) {
            return instance;
        } else {
            System.out.println("Statemachine not instantiated");
            return null;
        }
    }

    public BooleanSupplier pivotOnTarget() {
        return pivotOnTarget;
    }

    public BooleanSupplier elbowOnTarget() {
        return elbowOnTarget;
    }

    public BooleanSupplier extensionOnTarget() {
        return extensionOnTarget;
    }

    public BooleanSupplier liftOnTarget() {
        return liftOnTarget;
    }

    public BooleanSupplier elbowHoming() {return homingElbow; }
}
