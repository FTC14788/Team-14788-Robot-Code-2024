package org.firstinspires.ftc.teamcode.localization;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.constants.LocalizerConstants;
import org.firstinspires.ftc.teamcode.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.subsystems.OctoquadManager;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@Config
public final class ThreeDeadWheelLocalizer extends Localizer {
    public static LocalizerConstants localizerConstants = new LocalizerConstants();

    public double inPerTick = localizerConstants.INCHES_PER_TICK; //0.002934; //24.0 / 8163.0;;

    private int lastPar0Pos, lastPar1Pos, lastPerpPos;

    private double lastRawHeadingVel, headingVelOffset;

    public double imuYawHeading = 0.0;

    public double imuHeadingVelo = 0.0;
    private ExecutorService imuExecutor = Executors.newSingleThreadExecutor();
    private Rotation2d lastHeading;
    private long lastHeadingTime = System.currentTimeMillis();

//    private final Object imuLock = new Object();
//    @GuardedBy("imuLock")
    public IMU imu;

    private boolean initialized;

//    private ElapsedTime imuTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//    public static double IMU_INTERVAL = 100.0;

    public Pose2d pose = new Pose2d(0.0,0.0,0.0);

    public ThreeDeadWheelLocalizer(HardwareMap hardwareMap) {

        LazyImu lazyImu;
        lazyImu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu = lazyImu.get();
        imu.resetYaw();

        lastPar0Pos = OctoquadManager.getInstance().getPosition(0);
        lastPar1Pos = OctoquadManager.getInstance().getPosition(1);
        lastPerpPos = OctoquadManager.getInstance().getPosition(2);
    }

    public void startIMUThread(LinearOpMode opMode) {
//        imuExecutor.submit(new Runnable() {
//            @Override
//            public void run() {
//                while (!opMode.isStopRequested() && imuTimer.milliseconds() > IMU_INTERVAL) {
//                    synchronized (imuLock) {
//                        imuYawHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//                       // imuHeadingVelo = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
//                    }
//
//                    imuTimer.reset();
//                }
//            }
//        });
    }

    private int headingCounter = 0;

    public void update() {

        ElapsedTime loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        PositionVelocityPair par0PosVel = OctoquadManager.getInstance().getPosVelPair(0);
        PositionVelocityPair par1PosVel = OctoquadManager.getInstance().getPosVelPair(1);
        PositionVelocityPair perpPosVel = OctoquadManager.getInstance().getPosVelPair(2);

        if (!initialized) {
            initialized = true;

            lastPar0Pos = par0PosVel.position;
            lastPar1Pos = par1PosVel.position;
            lastPerpPos = perpPosVel.position;

            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

            lastHeading = heading;
        }

        int par0PosDelta = par0PosVel.position - lastPar0Pos;
        int par1PosDelta = par1PosVel.position - lastPar1Pos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;

        double headingDelta0 = (par0PosDelta - par1PosDelta) / (localizerConstants.LEFT_DEAD_WHEEL_Y_TICKS - localizerConstants.RIGHT_DEAD_WHEEL_Y_TICKS);
        double headingDelta = headingDelta0;

        if(headingCounter++ > 5) {
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));
//            Log.d("ThreeDeadWheelLocalizer_logger", String.format("Calc:%3.3f, IMU:%3.3f"headingDelta,heading.minus(lastHeading)));
            headingDelta = heading.minus(lastHeading);
            lastHeading = heading;
            headingCounter = 0;

        } else {
            lastHeading = lastHeading.plus(headingDelta);
        }

//        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
//        double rawHeadingVel = angularVelocity.zRotationRate;

        double calcAngularVelo = (par0PosVel.velocity - par1PosVel.velocity) / (localizerConstants.LEFT_DEAD_WHEEL_Y_TICKS - localizerConstants.RIGHT_DEAD_WHEEL_Y_TICKS);
        double rawHeadingVel = calcAngularVelo;

        // see https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/617
        if (Math.abs(rawHeadingVel - lastRawHeadingVel) > Math.PI) {
            headingVelOffset -= Math.signum(rawHeadingVel) * 2 * Math.PI;
        }
        lastRawHeadingVel = rawHeadingVel;
        double headingVel = headingVelOffset + rawHeadingVel;

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                (localizerConstants.LEFT_DEAD_WHEEL_Y_TICKS * par1PosDelta - localizerConstants.RIGHT_DEAD_WHEEL_Y_TICKS * par0PosDelta) / (localizerConstants.LEFT_DEAD_WHEEL_Y_TICKS - localizerConstants.RIGHT_DEAD_WHEEL_Y_TICKS),
                                (localizerConstants.LEFT_DEAD_WHEEL_Y_TICKS * par1PosVel.velocity - localizerConstants.RIGHT_DEAD_WHEEL_Y_TICKS * par0PosVel.velocity) / (localizerConstants.LEFT_DEAD_WHEEL_Y_TICKS - localizerConstants.RIGHT_DEAD_WHEEL_Y_TICKS),
                        }).times(inPerTick),
                        new DualNum<Time>(new double[] {
                                (-localizerConstants.PERP_DEAD_WHEEL_X_TICKS * headingDelta + perpPosDelta*1.025),
                                (localizerConstants.PERP_DEAD_WHEEL_X_TICKS / (localizerConstants.LEFT_DEAD_WHEEL_Y_TICKS - localizerConstants.RIGHT_DEAD_WHEEL_Y_TICKS) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
                        }).times(inPerTick)
                ),
                new DualNum<>(new double[] {
                        headingDelta,
                        headingVel,
                })
        );

        lastPar0Pos = par0PosVel.position;
        lastPar1Pos = par1PosVel.position;
        lastPerpPos = perpPosVel.position;

        pose = pose.plus(twist.value());

//        Log.d("Localizer_logger", String.format("par0_delta: %d | par1_delta: %d | angle_delta: %3.3f (imu: %3.3f) | velo: %3.3f (imu: %3.3f) (cal: %3.3f)" ,
//                par0PosDelta, par1PosDelta, Math.toDegrees(calcHeading), Math.toDegrees(headingDelta), calcAngularVelo, headingVel, calHeadingVel));

//        Log.d("ThreeDeadWheelsOdometry_logger", "Processing time (ms): " + String.format("%3.2f", loopTimer.milliseconds()));
//
//        Log.d("Localizer_logger", "p0: " + lastPar0Pos +
//                " | p1: " + lastPar1Pos + " | perp: " + lastPerpPos  + " | delta_h: " + String.format("%3.3f", headingDelta)
//                + " | (delta_x: " + String.format("%3.3f", twist.value().line.x) + ", delta_y: " + String.format("%3.3f", twist.value().line.y) + ")"
//                + " | (pose_x: " + String.format("%3.3f", pose.position.x) + ", pose_y: " + String.format("%3.3f", pose.position.y) + ")"
//        );

        loopTimer.reset();
    }

    public void setPoseEstimate(Pose2d poseEstimate) {
        this.pose = poseEstimate;
    }

    public Pose2d getPoseEstimate() {
        return this.pose;
    }

    public void resetHeading(double newHeading) {
        this.pose = new Pose2d(this.pose.position.x, this.pose.position.y, newHeading);
    }

    @Override
    public Pose getPose() {
        return new Pose(pose.position.x,pose.position.y, pose.heading.toDouble());
    }

    @Override
    public Pose getVelocity() {
        return null;
    }

    @Override
    public Vector getVelocityVector() {
        return null;
    }

    @Override
    public void setStartPose(Pose setStart) {
        pose = new Pose2d(setStart.getX(), setStart.getY(), setStart.getTheta());
    }

    public void setStartPose(Pose2d setStart) {
        pose = setStart;
    }

    @Override
    public void setPose(Pose setPose) {
        pose = new Pose2d(setPose.getX(), setPose.getY(), setPose.getTheta());
    }

    @Override
    public double getTotalHeading() {
        return pose.heading.toDouble();
    }

    @Override
    public double getForwardMultiplier(){
        return localizerConstants.INCHES_PER_TICK;
    }

    @Override
    public double getLateralMultiplier(){
        return localizerConstants.INCHES_PER_TICK;
    }

    @Override
    public double getTurningMultiplier(){
        return localizerConstants.TURN_RADIANS_PER_TICK;
    }
}
