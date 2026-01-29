package org.firstinspires.ftc.teamcode.autocode.pathingOnly;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mechanisms;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;

@Autonomous(name = "Red Close 9 Ball Bermuda Path Only", group = "Autonomous")
@Configurable
public class RedClose9BallBermudaPathOnly extends LinearOpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private RobotPaths paths;
    private Mechanisms mechanisms;

    private ArrayList<String> intakeOrder = new ArrayList<>();
    private boolean intakeOn = false;
    private long delayStart = 0;
    private final long DELAY_MS = 400;

    private String lastDetectedColor = null;



    @Override
    public void runOpMode() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(111, 136, Math.toRadians(270)));

        paths = new RobotPaths(follower);

        mechanisms = new Mechanisms();
        mechanisms.initMechanisms(hardwareMap, telemetry, true);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        waitForStart();

        int state = 0;
        while (opModeIsActive()) {

            follower.update();

                switch (state) {

                    case 0:
                        follower.followPath(paths.toAprilTag);
                        state = 1;
                        break;

                    case 1:
                        if (!follower.isBusy()) {
                            follower.followPath(paths.shootPreload);
                            state = 2;
                        }
                        break;

                    case 2:
                        if (!follower.isBusy()) {
                            follower.followPath(paths.toFirstBalls);
                            state = 3;
                        }
                        break;

                    case 3:
                        if (!follower.isBusy()) {
                            follower.followPath(paths.throughFirstBalls);
                            state = 4;
                        }
                        break;

                    case 4:
                        if (!follower.isBusy()) {
                            follower.followPath(paths.shootFirstBalls);
                            state = 5;
                        }
                        break;

                    case 5:
                        if (!follower.isBusy()) {
                            follower.followPath(paths.toSecondBalls);
                            state = 6;
                        }
                        break;

                    case 6:
                        if (!follower.isBusy()) {
                            follower.followPath(paths.throughSecondBalls);
                            state = 7;
                        }
                        break;

                    case 7:
                        if (!follower.isBusy()) {
                            follower.followPath(paths.shootSecondBalls);
                            state = 8;
                        }
                        break;

                    case 8:
                        if (!follower.isBusy()) {
                            follower.followPath(paths.toEndPosition);
                            state = 9;
                        }
                        break;
                }

            panelsTelemetry.debug("State", state);
            panelsTelemetry.debug("X", follower.getPose().getX());
            panelsTelemetry.debug("Y", follower.getPose().getY());
            panelsTelemetry.update(telemetry);
        }
    }

    // ---------------- PATH LIST ----------------

    public static class RobotPaths {
        public PathChain toAprilTag;
        public PathChain shootPreload;
        public PathChain toFirstBalls;
        public PathChain throughFirstBalls;
        public PathChain shootFirstBalls;
        public PathChain toSecondBalls;
        public PathChain throughSecondBalls;
        public PathChain shootSecondBalls;
        public PathChain toEndPosition;

        public RobotPaths(Follower follower) {
            toAprilTag = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(111.000, 136.000),

                                    new Pose(100.000, 100.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(130))

                    .build();

            shootPreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(100.000, 100.000),

                                    new Pose(100.000, 87.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(45))

                    .build();

            toFirstBalls = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(100.000, 87.000),

                                    new Pose(102.000, 81.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                    .build();

            throughFirstBalls = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(102.000, 81.000),

                                    new Pose(133.000, 81.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            shootFirstBalls = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(133.000, 81.000),

                                    new Pose(100.000, 87.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                    .build();

            toSecondBalls = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(100.000, 87.000),

                                    new Pose(102.000, 56.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                    .build();

            throughSecondBalls = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(102.000, 56.000),

                                    new Pose(133.000, 56.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            shootSecondBalls = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(133.000, 56.000),

                                    new Pose(100.000, 87.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                    .build();

            toEndPosition = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(100.000, 87.000),

                                    new Pose(124.000, 69.548)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(270))

                    .build();
        }
    }
}