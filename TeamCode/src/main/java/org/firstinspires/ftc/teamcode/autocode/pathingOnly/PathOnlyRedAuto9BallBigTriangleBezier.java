package org.firstinspires.ftc.teamcode.autocode.pathingOnly;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "RED 9 Ball Big Triangle (Bezier Path Only)", group = "Autonomous")
@Configurable
public class PathOnlyRedAuto9BallBigTriangleBezier extends LinearOpMode {

    private Follower follower;

    // ---------- Paths ----------
    private PathChain ScanColors;
    private PathChain Shoot1st3;
    private PathChain Intake2nd3;
    private PathChain Shoot2nd3;
    private PathChain Intake3rd3;
    private PathChain Shoot3rd3;
    private PathChain Park;

    private int state = 0;

    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);

        // Starting pose MUST match first path pose
        follower.setStartingPose(new Pose(111.000, 135.000, Math.toRadians(90)));

        buildPaths();

        waitForStart();

        while (opModeIsActive()) {

            follower.update();

            switch (state) {

                case 0:
                    follower.followPath(ScanColors);
                    state = 1;
                    break;

                case 1:
                    if (!follower.isBusy()) {
                        follower.followPath(Shoot1st3);
                        state = 2;
                    }
                    break;

                case 2:
                    if (!follower.isBusy()) {
                        follower.followPath(Intake2nd3);
                        state = 3;
                    }
                    break;

                case 3:
                    if (!follower.isBusy()) {
                        follower.followPath(Shoot2nd3);
                        state = 4;
                    }
                    break;

                case 4:
                    if (!follower.isBusy()) {
                        follower.followPath(Intake3rd3);
                        state = 5;
                    }
                    break;

                case 5:
                    if (!follower.isBusy()) {
                        follower.followPath(Shoot3rd3);
                        state = 6;
                    }
                    break;

                case 6:
                    if (!follower.isBusy()) {
                        follower.followPath(Park);
                        state = 7;
                    }
                    break;
            }

            telemetry.addData("State", state);
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("Busy", follower.isBusy());
            telemetry.update();
        }
    }

    // ---------- Path Builder ----------
    private void buildPaths() {


        ScanColors = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(111.000, 135.000),

                                new Pose(99.115, 110.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(127))

                .build();

        Shoot1st3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(99.115, 110.000),

                                new Pose(99.115, 97.996)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(127), Math.toRadians(49))

                .build();

        Intake2nd3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(99.115, 97.996),
                                new Pose(92.371, 95.577),
                                new Pose(74.281, 80.879),
                                new Pose(128.204, 83.358)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(49), Math.toRadians(0))

                .build();

        Shoot2nd3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(128.204, 83.358),

                                new Pose(99.290, 97.761)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(49))

                .build();

        Intake3rd3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(99.290, 97.761),
                                new Pose(76.038, 56.997),
                                new Pose(128.485, 59.712)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(49), Math.toRadians(0))

                .build();

        Shoot3rd3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(128.485, 59.712),
                                new Pose(100.388, 78.355),
                                new Pose(99.290, 97.761)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(49))

                .build();

        Park = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(99.290, 97.761),

                                new Pose(123.436, 66.895)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(49), Math.toRadians(90))

                .build();
    }
}
