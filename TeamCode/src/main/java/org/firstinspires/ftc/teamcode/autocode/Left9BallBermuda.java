package org.firstinspires.ftc.teamcode.autocode;

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

@Autonomous(name = "LeftBermuda", group = "Autonomous")
@Configurable
public class Left9BallBermuda extends LinearOpMode {

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
        follower.setStartingPose(new Pose(33, 136, Math.toRadians(270)));

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
                    follower.followPath(paths.scanAprilTag);
                    state = 1;
                    break;

                case 1:
                    follower.followPath(paths.shootPreload);
                    state = 2;
                    break;

                case 2:
                    follower.followPath(paths.toFirstBalls);
                    state = 3;
                    break;

                case 3:
                    follower.followPath(paths.throughFirstBalls);
                    state = 4;
                    break;

                case 4:
                    follower.followPath(paths.shootFirstBalls);
                    state = 5;
                    break;

                case 5:
                    follower.followPath(paths.toSecondBalls);
                    state = 6;
                    break;

                case 6:
                    follower.followPath(paths.throughSecondBalls);
                    state = 7;
                    break;

                case 7:
                    follower.followPath(paths.shootSecondBalls);
                    state = 8;
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
        public PathChain scanAprilTag;
        public PathChain shootPreload;
        public PathChain toFirstBalls;
        public PathChain throughFirstBalls;
        public PathChain shootFirstBalls;
        public PathChain toSecondBalls;
        public PathChain throughSecondBalls;
        public PathChain shootSecondBalls;

        public RobotPaths(Follower follower) {
            scanAprilTag = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(33.000, 136.000),

                                    new Pose(60.000, 110.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(80))

                    .build();

            shootPreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(60.000, 110.000),

                                    new Pose(32.000, 110.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(80), Math.toRadians(135))

                    .build();

            toFirstBalls = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(32.000, 110.000),

                                    new Pose(44.000, 87.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            throughFirstBalls = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(44.000, 87.000),

                                    new Pose(16.000, 87.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            shootFirstBalls = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(16.000, 87.000),

                                    new Pose(32.000, 110.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            toSecondBalls = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(32.000, 110.000),

                                    new Pose(44.000, 63.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            throughSecondBalls = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(44.000, 63.000),

                                    new Pose(16.000, 63.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            shootSecondBalls = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(16.000, 63.000),

                                    new Pose(32.000, 110.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();
        }
    }
}