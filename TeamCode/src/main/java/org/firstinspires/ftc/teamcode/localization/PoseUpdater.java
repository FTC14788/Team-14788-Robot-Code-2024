package org.firstinspires.ftc.teamcode.localization;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Twist2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.constants.DriveConstnats;
import org.firstinspires.ftc.teamcode.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.util.Derivator;
import org.firstinspires.ftc.teamcode.util.PoseSTDs;
import org.firstinspires.ftc.teamcode.util.TimeInterpolatableBuffer;

import java.util.Map;

/**
 * This is the PoseUpdater class. This class handles getting pose data from the localizer and returning
 * the information in a useful way to the Follower.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/4/2024
 */
public class PoseUpdater {
    private HardwareMap hardwareMap;

    private IMU imu;

    private Localizer localizer;

    private Pose startingPose = new Pose(0,0,0);

    private Pose currentPose = startingPose;

    private Pose previousPose = startingPose;

    private Vector currentVelocity = new Vector();

    private Vector previousVelocity = new Vector();

    private Vector currentAcceleration = new Vector();

    private double xOffset = 0;
    private double yOffset = 0;
    private double headingOffset = 0;

    public double lastLoopTimeMs;

    private long previousPoseTime;
    private long currentPoseTime;

    private double[] kalmanGain = new double[2];
    private PoseSTDs visionMeasurementSTDs;
    private double odometryTranslationCovariance = DriveConstnats.odometryTranslationCovariance;
    private double odometryRotationCovariance = DriveConstnats.odometryRotationCovariance;

    public TimeInterpolatableBuffer<Pose> poseBuffer;
    public TimeInterpolatableBuffer<Pose> accelerationBuffer;
    /**
     * Creates a new PoseUpdater from a HardwareMap and a Localizer.
     *
     * @param hardwareMap the HardwareMap
     * @param localizer the Localizer
     */
    public PoseUpdater(HardwareMap hardwareMap, Localizer localizer) {
        this.hardwareMap = hardwareMap;

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        this.localizer = localizer;

        poseBuffer = new TimeInterpolatableBuffer<Pose>(DriveConstnats.poseBufferSpan);
        accelerationBuffer = new TimeInterpolatableBuffer<Pose>(DriveConstnats.poseBufferSpan);
    }

    /**
     * Creates a new PoseUpdater from a HardwareMap.
     *
     * @param hardwareMap the HardwareMap
     */
    public PoseUpdater(HardwareMap hardwareMap) {
        // TODO: replace the second argument with your preferred localizer
        this(hardwareMap, new ThreeDeadWheelLocalizer(hardwareMap));
    }

    /**
     * This updates the robot's pose, as well as updating the previous pose, velocity, and
     * acceleration. The cache for the current pose, velocity, and acceleration is cleared, and
     * the time stamps are updated as well.
     */
    public void update() {
        previousVelocity = getVelocity();
        previousPose = applyOffset(getRawPose());
        currentPose = null;
        currentVelocity = null;
        currentAcceleration = null;
        previousPoseTime = currentPoseTime;
        currentPoseTime = System.nanoTime();
        localizer.update();
    }

    private void addData() {
        poseBuffer.addMeasurement((double) currentPoseTime, getRawPose());

    }

    /**
     * This updates the current pose and pose history by applying a kalman filter to the difference
     * in estimated pose and the camera's pose.
     * @param visionPose Pose as recorded by the vision
     * @param timeStampSeconds Timestamp, based on the
     */
    public void addVisionMeasurement(Pose2d visionPose, double timeStampSeconds) {
        // Step 1: Take the pose sample at the timestamp that the camera clicked
        Pose poseSample = poseBuffer.getInterpolatedValue(timeStampSeconds);

        if (poseSample == null) {
            return;
        }

        // Step 2: Find the twist that translates the pose in the buffer to the camera's pose
        Twist2d twist = poseSample.toPose2d().log(visionPose);

        if (twist.dtheta > DriveConstnats.ambiguityThresholdRadians) {
            return;
        }

        // Kalman filter?
        Twist2d scaledTwist = new Twist2d(
                twist.dx * kalmanGain[0],
                twist.dy * kalmanGain[0],
                twist.dtheta * kalmanGain[1]
        ); // * a value (0-1) as determined by the kalman filter

        // Step 5: Set the pose of the odometry to the vision adjustment
        setPose(
                Pose.toPose(
                       currentPose.toPose2d().exp(scaledTwist)
                )
        );

        // Step 7: Loop through the treemap and update the odometry accordingly
        for (Map.Entry<Double, Pose> entry : poseBuffer.getRawBuffer()/*.tailMap(timeStampSeconds)*/.entrySet()){
            cacheOffset(entry.getKey(), entry.getValue(), scaledTwist);
        }
    }

    public Pose cacheOffset(double currentTimeSeconds, Pose oldValue, Twist2d offset) {
        try {
            Pose check = poseBuffer.getRawBuffer().get(currentTimeSeconds);
            if (!(check == oldValue)) {
                return null;
            }
        } catch (Exception E) {
            System.out.println("Invalid buffer data");
            return null;
        }

        poseBuffer.addMeasurement(
                currentTimeSeconds,
                Pose.toPose(oldValue.toPose2d().exp(offset))
        );
        return getEstimatedPose();
    }

    public void addVisionMeasurement(Pose2d visionPose, double timeStampSeconds, PoseSTDs diseases) {
        setVisionMeasurementSTDs(diseases);
        addVisionMeasurement(visionPose, timeStampSeconds);
    }

    public void setVisionMeasurementSTDs(PoseSTDs diseases) {
        visionMeasurementSTDs.setSTDs(diseases);

        kalmanGain[0] = odometryTranslationCovariance / (odometryTranslationCovariance + visionMeasurementSTDs.getTranslationVariance());
        kalmanGain[1] = odometryRotationCovariance / (odometryRotationCovariance + visionMeasurementSTDs.getRotationVariance());
        
        odometryTranslationCovariance = (odometryTranslationCovariance * (1 - kalmanGain[0]));
        odometryRotationCovariance = (odometryRotationCovariance * (1 - kalmanGain[1]));
    }

    /**
     * This sets the starting pose. Do not run this after moving at all.
     *
     * @param set the Pose to set the starting pose to.
     */
    public void setStartingPose(Pose set) {
        startingPose = set;
        previousPose = startingPose;
        previousPoseTime = System.nanoTime();
        currentPoseTime = System.nanoTime();
        localizer.setStartPose(set);
    }

    /**
     * This sets the current pose, using offsets. Think of using offsets as setting trim in an
     * aircraft. This can be reset as well, so beware of using the resetOffset() method.
     *
     *
     * @param set The pose to set the current pose to.
     */
    public void setCurrentPoseWithOffset(Pose set) {
        Pose currentPose = getRawPose();
        setXOffset(set.getX() - currentPose.getX());
        setYOffset(set.getY() - currentPose.getY());
        setHeadingOffset(MathFunctions.getTurnDirection(currentPose.getTheta(), set.getTheta()) * MathFunctions.getSmallestAngleDifference(currentPose.getTheta(), set.getTheta()));
    }

    /**
     * This sets the offset for only the x position.
     *
     * @param offset This sets the offset.
     */
    public void setXOffset(double offset) {
        xOffset = offset;
    }

    /**
     * This sets the offset for only the y position.
     *
     * @param offset This sets the offset.
     */
    public void setYOffset(double offset) {
        yOffset = offset;
    }

    /**
     * This sets the offset for only the heading.
     *
     * @param offset This sets the offset.
     */
    public void setHeadingOffset(double offset) {
        headingOffset = offset;
    }

    /**
     * This returns the x offset.
     *
     * @return returns the x offset.
     */
    public double getXOffset() {
        return xOffset;
    }

    /**
     * This returns the y offset.
     *
     * @return returns the y offset.
     */
    public double getYOffset() {
        return yOffset;
    }

    /**
     * This returns the heading offset.
     *
     * @return returns the heading offset.
     */
    public double getHeadingOffset() {
        return headingOffset;
    }

    /**
     * This applies the offset to a specified Pose.
     *
     * @param pose The pose to be offset.
     * @return This returns a new Pose with the offset applied.
     */
    public Pose applyOffset(Pose pose) {
        return new Pose(pose.getX()+xOffset, pose.getY()+yOffset, pose.getTheta()+headingOffset);
    }

    /**
     * This resets all offsets set to the PoseUpdater. If you have reset your pose using the
     * setCurrentPoseUsingOffset(Pose2d set) method, then your pose will be returned to what the
     * PoseUpdater thinks your pose would be, not the pose you reset to.
     */
    public void resetOffset() {
        setXOffset(0);
        setYOffset(0);
        setHeadingOffset(0);
    }

    /**
     * This returns the current pose, with offsets applied. If this is called multiple times in
     * a single update, the current pose is cached so that subsequent calls don't have to repeat
     * localizer calls or calculations.
     *
     * @return returns the current pose.
     */
    public Pose getEstimatedPose() {
        if (currentPose == null) {
            currentPose = localizer.getPose();
            return applyOffset(currentPose);
        } else {
            return applyOffset(currentPose);
        }
    }

    /**
     * This returns the current raw pose, without any offsets applied. If this is called multiple times in
     * a single update, the current pose is cached so that subsequent calls don't have to repeat
     * localizer calls or calculations.
     *
     * @return returns the raw pose.
     */
    public Pose getRawPose() {
        if (currentPose == null) {
            currentPose = localizer.getPose();
            return currentPose;
        } else {
            return currentPose;
        }
    }

    /**
     * This sets the current pose without using resettable offsets.
     *
     * @param set the pose to set the current pose to.
     */
    public void setPose(Pose set) {
        resetOffset();
        localizer.setPose(set);
    }

    /**
     * Returns the robot's pose from the previous update.
     *
     * @return returns the robot's previous pose.
     */
    public Pose getPreviousPose() {
        return previousPose;
    }

    /**
     * Returns the robot's change in pose from the previous update.
     *
     * @return returns the robot's delta pose.
     */
    public Pose getDeltaPose() {
        Pose returnPose = getEstimatedPose();
        returnPose.subtract(previousPose);
        return returnPose;
    }

    /**
     * This returns the velocity of the robot as a Vector. If this is called multiple times in
     * a single update, the velocity Vector is cached so that subsequent calls don't have to repeat
     * localizer calls or calculations.
     *
     * @return returns the velocity of the robot.
     */
    public Vector getVelocity() {
        if (currentVelocity == null) {
            currentVelocity = new Vector();
            currentVelocity.setOrthogonalComponents(getEstimatedPose().getX() - previousPose.getX(), getEstimatedPose().getY() - previousPose.getY());
            currentVelocity.setMagnitude(MathFunctions.distance(getEstimatedPose(), previousPose) / ((currentPoseTime - previousPoseTime) / Math.pow(10.0, 9)));
            return MathFunctions.copyVector(currentVelocity);
        } else {
            return MathFunctions.copyVector(currentVelocity);
        }
    }

    /**
     * This returns the angular velocity of the robot as a double.
     *
     * @return returns the angular velocity of the robot.
     */
    public double getAngularVelocity() {
        return MathFunctions.getTurnDirection(previousPose.getTheta(), getEstimatedPose().getTheta()) * MathFunctions.getSmallestAngleDifference(getEstimatedPose().getTheta(), previousPose.getTheta()) / ((currentPoseTime-previousPoseTime)/Math.pow(10.0, 9));
    }

    /**
     * This returns the acceleration of the robot as a Vector. If this is called multiple times in
     * a single update, the acceleration Vector is cached so that subsequent calls don't have to
     * repeat localizer calls or calculations.
     *
     * @return returns the acceleration of the robot.
     */
    public Vector getAcceleration() {
        if (currentAcceleration == null) {
            currentAcceleration = MathFunctions.subtractVectors(getVelocity(), previousVelocity);
            currentAcceleration.setMagnitude(currentAcceleration.getMagnitude() / ((currentPoseTime - previousPoseTime) / Math.pow(10.0, 9)));
            return MathFunctions.copyVector(currentAcceleration);
        } else {
            return MathFunctions.copyVector(currentAcceleration);
        }
    }

    /**
     * This resets the heading of the robot to the IMU's heading, using Road Runner's pose reset.
     */
    public void resetHeadingToIMU() {
        localizer.setPose(new Pose(getEstimatedPose().getX(), getEstimatedPose().getY(), getNormalizedIMUHeading() + startingPose.getTheta()));
    }

    /**
     * This resets the heading of the robot to the IMU's heading, using offsets instead of Road
     * Runner's pose reset. This way, it's faster, but this can be wiped with the resetOffsets()
     * method.
     */
    public void resetHeadingToIMUWithOffsets() {
        setCurrentPoseWithOffset(new Pose(getEstimatedPose().getX(), getEstimatedPose().getY(), getNormalizedIMUHeading() + startingPose.getTheta()));
    }

    /**
     * This returns the IMU heading normalized to be between [0, 2 PI] radians.
     *
     * @return returns the normalized IMU heading.
     */
    public double getNormalizedIMUHeading() {
        return MathFunctions.normalizeAngle(-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }

    /**
     * This returns the total number of radians the robot has turned.
     *
     * @return the total heading.
     */
    public double getTotalHeading() {
        return localizer.getTotalHeading();
    }

    /**
     * This returns the Localizer.
     *
     * @return the Localizer
     */
    public Localizer getLocalizer() {
        return localizer;
    }
}
