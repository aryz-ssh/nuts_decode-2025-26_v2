package org.firstinspires.ftc.teamcode.autocode;

public class RedAuto12BallSmallTriangle {

    public static class Paths {
        public PathChain ShootPreloads;
        public PathChain ScanMotif;
        public PathChain Intake;
        public PathChain HitGate;
        public PathChain Outtake;
        public PathChain Intake;
        public PathChain Outtake;
        public PathChain Intake;
        public PathChain Outtake;

        public Paths(Follower follower) {
            ShootPreloads = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(90.000, 8.000),

                                    new Pose(83.995, 19.028)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(69))

                    .build();

            ScanMotif = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(83.995, 19.028),
                                    new Pose(92.676, 52.306),
                                    new Pose(90.809, 97.983)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(69), Math.toRadians(100))

                    .build();

            Intake = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(90.809, 97.983),
                                    new Pose(65.365, 71.609),
                                    new Pose(128.479, 83.705)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(100), Math.toRadians(0))

                    .build();

            HitGate = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(128.479, 83.705),
                                    new Pose(120.078, 77.324),
                                    new Pose(129.098, 71.942)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))

                    .build();

            Outtake = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(129.098, 71.942),
                                    new Pose(72.465, 76.417),
                                    new Pose(83.995, 19.028)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(69))

                    .build();

            Intake = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(83.995, 19.028),
                                    new Pose(93.688, 63.080),
                                    new Pose(128.826, 59.414)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(69), Math.toRadians(0))

                    .build();

            Outtake = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(128.826, 59.414),
                                    new Pose(97.462, 42.340),
                                    new Pose(83.995, 18.567)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(69))

                    .build();

            Intake = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(83.995, 18.567),
                                    new Pose(92.756, 35.226),
                                    new Pose(126.665, 35.911)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(69), Math.toRadians(0))

                    .build();

            Outtake = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(126.665, 35.911),
                                    new Pose(93.857, 28.665),
                                    new Pose(83.995, 18.567)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(69))

                    .build();
        }
    }

}
