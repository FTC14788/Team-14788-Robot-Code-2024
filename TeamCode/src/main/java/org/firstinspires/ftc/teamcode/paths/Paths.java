package org.firstinspires.ftc.teamcode.paths;

import org.firstinspires.ftc.teamcode.localization.Pose;
import org.firstinspires.ftc.teamcode.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;

public class Paths {
    public static PathChain spikeThreePiecePush() {
        Path path0 = new Path(
            new BezierCurve(
                new Point(new Pose(-45, 10, 0)),
                new Point(new Pose(-38, 20, 0)),
                new Point(new Pose(-36, 30, 0)),
                new Point(new Pose(-36, 50, 0))
            )
        );
        path0.setConstantHeadingInterpolation(-0.5 * Math.PI);

        Path path1 = new Path(
            new BezierCurve(
                new Point(new Pose(-36, 50, 0)),
                new Point(new Pose(-36, 67, 0)),
                new Point(new Pose(-24, 70, 0)),
                new Point(new Pose(-24, 55, 0)),
                new Point(new Pose(-24, 45, 0)),
                new Point(new Pose(-24, 17, 0))

            )
        );

        path1.setConstantHeadingInterpolation(-0.5 * Math.PI);

        Path path2 = new Path(
            new BezierCurve(
                new Point(new Pose(-24, 17, 0)),
                new Point(new Pose(-24, 60, 0)),
                new Point(new Pose(-16, 60, 0))
            )
        );

        path2.setConstantHeadingInterpolation(-0.5 * Math.PI);

        Path path3 = new Path(
                new BezierCurve(
                        new Point(new Pose(-16, 60, 0)),
                        new Point(new Pose(-16, 50, 0)),
                        new Point(new Pose(-16, 15, 0))

                )
        );

        path3.setConstantHeadingInterpolation(-0.5 * Math.PI);

        Path path4 = new Path(
                new BezierCurve(
                        new Point(new Pose(-16, 15, 0)),
                        new Point(new Pose(-16, 65, 0)),
                        new Point(new Pose(-12, 65, 0)),
                        new Point(new Pose(-12, 60, 0))

                )
        );

        path4.setConstantHeadingInterpolation(-0.5 * Math.PI);

        Path path5 = new Path(
                new BezierCurve(
                        new Point(new Pose(-12, 60, 0)),
                        new Point(new Pose(-10, 45, 0)),
                        new Point(new Pose(-12, 15, 0))

                )
        );

        path5.setConstantHeadingInterpolation(-0.5 * Math.PI);

        return new PathChain(path0, path1, path2, path3, path4, path5);
    }

    public static PathChain spikeThreePieceScorePush() {
        MecanumSubsystem.getInstance().setPose(new Pose(32, 10, Math.PI / -2));

        Path pathNegative0 = new Path(
                new BezierCurve(
                        new Point(new Pose(32, 10, 0)),
                        new Point(new Pose(20, 12, 0))
                )
        );

        pathNegative0.setConstantHeadingInterpolation(-0.35 * Math.PI);

        Path path0 = new Path(
                new BezierCurve(
                        new Point(new Pose(20, 12, 0)),
                        new Point(new Pose(34, 25, 0)),
                        new Point(new Pose(37.5, 35, 0)),
                        new Point(new Pose(37.5, 50, 0))
                )
        );
        path0.setConstantHeadingInterpolation(-0.5 * Math.PI);

        Path path1 = new Path(
                new BezierCurve(
                        new Point(new Pose(36, 50, 0)),
                        new Point(new Pose(36, 65, 0)),
                        new Point(new Pose(28, 65, 0)),
                        new Point(new Pose(28, 55, 0)),
                        new Point(new Pose(27, 45, 0)),
                        new Point(new Pose(16, 24, 0))

                )
        );

        path1.setLinearHeadingInterpolation(-0.5 * Math.PI, -0.75 * Math.PI);

        Path path2 = new Path(
                new BezierCurve(
                        new Point(new Pose(24, 24, 0)),
                        new Point(new Pose(24, 60, 0)),
                        new Point(new Pose(16, 60, 0))
                )
        );

        path2.setConstantHeadingInterpolation(-0.5 * Math.PI);

        Path path3 = new Path(
                new BezierCurve(
                        new Point(new Pose(16, 60, 0)),
                        new Point(new Pose(16, 50, 0)),
                        new Point(new Pose(16, 26, 0))

                )
        );

        path1.setLinearHeadingInterpolation(-0.5 * Math.PI, -0.65 * Math.PI);

        Path path4 = new Path(
                new BezierCurve(
                        new Point(new Pose(16, 26, 0)),
                        new Point(new Pose(16, 62, 0)),
                        new Point(new Pose(9.5, 62, 0))

                )
        );

        path4.setConstantHeadingInterpolation(-0.5 * Math.PI);

        Path path5 = new Path(
                new BezierCurve(
                        new Point(new Pose(9.5, 62, 0)),
                        new Point(new Pose(9.5, 30, 0)),
                        new Point(new Pose(12, 26, 0))

                )
        );

        path5.setConstantHeadingInterpolation(-0.5 * Math.PI);

        Path path6 = new Path(
                new BezierCurve(
                        new Point(new Pose(12, 26, 0)),
                        new Point(new Pose(20, 20, 0)),
                        new Point(new Pose(125, 11, 0))

                )
        );

        path6.setConstantHeadingInterpolation(Math.PI);

        return new PathChain(pathNegative0, path0, path1, path2, path3, path4, path5, path6);
    }

    public static Path tuner() {
        Path path0 = new Path(
                new BezierCurve(
                        new Point(new Pose(-45, 10, 0)),
                        new Point(new Pose(-45, 40, 0))
                )
        );

        return path0;
    }
}
