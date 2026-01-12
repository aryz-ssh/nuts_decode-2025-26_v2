/* package org.firstinspires.ftc.teamcode.autocode;
//I Did not import anything like path chain
public class BlueAuto12BallSmallTriangle {

    public static class Paths {
        public PathChain ShootPreloads;
        public PathChain Scan;
        public PathChain Intake;
        public PathChain HitGate;
        public PathChain Outtake;
        public PathChain Intake;
        public PathChain Outtake;
        public PathChain Intake;
        public PathChain Outtake;

        public Paths(Follower follower) {
            ShootPreloads = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.000, 8.000),
                                    new Pose(62.424, 19.763),
                                    new Pose(60.374, 17.978)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110))

                    .build();

            Scan = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(60.374, 17.978),

                                    new Pose(49.445, 90.954)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(84))

                    .build();

            Intake = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(49.445, 90.954),
                                    new Pose(60.772, 76.896),
                                    new Pose(15.129, 83.448)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(84), Math.toRadians(180))

                    .build();

            HitGate = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(15.129, 83.448),
                                    new Pose(21.922, 73.659),
                                    new Pose(14.818, 72.741)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                    .build();

            Outtake = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(14.818, 72.741),
                                    new Pose(76.108, 76.559),
                                    new Pose(60.539, 18.372)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110))

                    .build();

            Intake = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(60.539, 18.372),
                                    new Pose(79.533, 66.801),
                                    new Pose(15.479, 60.207)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))

                    .build();

            Outtake = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(15.479, 60.207),
                                    new Pose(65.247, 56.351),
                                    new Pose(60.199, 18.328)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))

                    .build();

            Intake = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(60.199, 18.328),
                                    new Pose(42.604, 37.756),
                                    new Pose(17.228, 32.962)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))

                    .build();

            Outtake = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(17.228, 32.962),
                                    new Pose(59.122, 38.831),
                                    new Pose(60.199, 18.153)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))

                    .build();
        }
    }
}
*/