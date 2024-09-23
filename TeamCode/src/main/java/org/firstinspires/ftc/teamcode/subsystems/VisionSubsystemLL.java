package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class VisionSubsystemLL extends CommandBase  {

    private Limelight3A limelight;

    private int pipeline;

    public enum Pipeline {
        Apriltags,
        NeuralNetworks,
        Python
    }

    public VisionSubsystemLL(HardwareMap hardwareMap, String name, Pipeline pipeline) {
        switch (pipeline) {
            case Apriltags:
                this.pipeline = 0;
            case NeuralNetworks:
                this.pipeline = 1;
            case Python:
                this.pipeline = 2;
        }

        limelightInit(hardwareMap, name);
    }

    public void limelightInit(HardwareMap hardwareMap, String name) {
        limelight = hardwareMap.get(Limelight3A.class, name);

        limelight.pipelineSwitch(pipeline);
        limelight.start();
    }

}

