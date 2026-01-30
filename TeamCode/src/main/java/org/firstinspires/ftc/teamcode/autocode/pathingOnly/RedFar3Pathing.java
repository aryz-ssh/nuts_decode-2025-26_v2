package org.firstinspires.ftc.teamcode.autocode.pathingOnly;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "RedFar3_PathingOnly", group = "Autonomous")
@Config
public class RedFar3Pathing extends LinearOpMode {

    private Follower follower;
    private Paths paths;

    public static class Paths {
        public PathChain Path1, Path2, Path3;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(51.317, 8.780),
                            new Pose(51.707, 22.341)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(51.707, 22.341),
                            new Pose(65.561, 13.049)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(60))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(65.561, 13.049),
                            new Pose(37.390, 13.024)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }

    private enum AutoState { PATH1, PATH2, PATH3, DONE }

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(51.317, 8.780, Math.toRadians(90)));

        paths = new Paths(follower);

        waitForStart();

        AutoState state = AutoState.PATH1;

        boolean path1Started = false, path2Started = false, path3Started = false;

        while (opModeIsActive()) {
            follower.update();

            switch (state) {
                case PATH1:
                    if (!path1Started) {
                        follower.followPath(paths.Path1);
                        path1Started = true;
                    }
                    if (!follower.isBusy()) state = AutoState.PATH2;
                    break;

                case PATH2:
                    if (!path2Started) {
                        follower.followPath(paths.Path2);
                        path2Started = true;
                    }
                    if (!follower.isBusy()) state = AutoState.PATH3;
                    break;

                case PATH3:
                    if (!path3Started) {
                        follower.followPath(paths.Path3);
                        path3Started = true;
                    }
                    if (!follower.isBusy()) state = AutoState.DONE;
                    break;

                case DONE:
                    break;
            }
        }
    }
}
