package org.firstinspires.ftc.teamcode.opmode;

import org.firstinspires.ftc.teamcode.util.OpModeContainer;

public class MainTeleopOpMode extends OpModeContainer {

    @Override
    public void initialize() {
        initHardware(OpModeType.TELEOP);
    }

    @Override
    public void run() {
        super.run();

        mecanumSubsystem.setTeleOpMovementVectors(-driverController.getLeftY(), -driverController.getLeftX(), -driverController.getRightX(), true);
    }
}
