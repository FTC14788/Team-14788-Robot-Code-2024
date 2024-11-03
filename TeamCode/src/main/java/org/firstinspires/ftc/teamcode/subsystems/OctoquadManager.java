package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OctoquadManager extends SubsystemBase {
    private static OctoquadManager instance;

    private final int PAR_0 = 0;
    private final int PAR_1 = 1;
    private final int PAR_2 = 2;

    private final int PIVOT_REMOTE = 3;
    
    private OctoQuad octo;

    private int par0Pos;
    private int par1Pos;
    private int par2Pos;

    private int pivotRemotePos;

    private short par0Vel;
    private short par1Vel;
    private short par2Vel;

    private OctoquadManager(HardwareMap hardwareMap) {
        octo = hardwareMap.get(OctoQuad.class, "octoquad");

        octo.setSingleEncoderDirection(PAR_0,  OctoQuad.EncoderDirection.REVERSE);
        octo.setSingleEncoderDirection(PAR_1, OctoQuad.EncoderDirection.FORWARD);
        octo.setSingleEncoderDirection(PAR_2,  OctoQuad.EncoderDirection.FORWARD);

        octo.saveParametersToFlash();
        octo.resetAllPositions();
    }

    public static void createInstance(HardwareMap hardwareMap) {
        instance = new OctoquadManager(hardwareMap);
    }

    public static OctoquadManager getInstance() {
        return instance;
    }


    @Override
    public void periodic() {
        readEncoders();
    }

    private void readEncoders() {
        int[] positions = octo.readAllPositions();
        short[] velocities = octo.readAllVelocities();

        par0Pos = positions[0];
        par1Pos = positions[1];
        par2Pos = positions[2];
        pivotRemotePos = positions[3];

        par0Vel = velocities[0];
        par1Vel = velocities[1];
        par2Vel = velocities[2];
    }

    public int getPosition(int index) {
        switch (index) {
            case 0:
                return par0Pos;
            case 1:
                return par1Pos;
            case 2:
                return par2Pos;
            case 3:
                return pivotRemotePos;
            default:
                return 0;
        }
    }

    public short getVelocity(int index) {
        switch (index) {
            case 0:
                return par0Vel;
            case 1:
                return par1Vel;
            case 2:
                return par2Vel;
            default:
                return 0;
        }
    }

    public PositionVelocityPair getPosVelPair(int index) {
        return new PositionVelocityPair(getPosition(index), getVelocity(index), getPosition(index), getVelocity(index));
    }

}
