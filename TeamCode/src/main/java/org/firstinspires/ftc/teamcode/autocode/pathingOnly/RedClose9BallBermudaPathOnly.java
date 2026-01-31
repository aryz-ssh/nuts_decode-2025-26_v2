package org.firstinspires.ftc.teamcode.autocode.pathingOnly;

import com.acmerobotics.dashboard.config.Config;
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
@Config
public class RedClose9BallBermudaPathOnly extends LinearOpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private RobotPaths paths;
    private Mechanisms mechanisms;

    private ArrayList<String> intakeOrder = new ArrayList<>();
    private boolean intakeOn = false;
    private long delayStart = 0;
    private final long DELAY_MS = 400;

    public static long PATH_DELAY_MS = 5000; // 5 seconds
    private long pathEndTime = -1;

    private String lastDetectedColor = null;

    // ================= PATH TUNING =================

    // ---------- START ----------
    public static double START_X = 111.0;
    public static double START_Y = 136.0;
    public static double START_H = 90;

    // ---------- APRIL TAG ----------
    public static double TAG_X = 100.0;
    public static double TAG_Y = 125;
    public static double TAG_H = 130;

    // ---------- PRELOAD SHOT ----------
    public static double PRELOAD_X = 105.0;
    public static double PRELOAD_Y = 99.0;
    public static double PRELOAD_H = 45;

    // ---------- FIRST BALL LINE ----------
    public static double FIRST_ENTRY_X = 102.0;
    public static double FIRST_ENTRY_Y = 81.0;

    public static double FIRST_EXIT_X = 126.0;
    public static double FIRST_EXIT_Y = 81.0;

    // ---------- SECOND BALL LINE ----------
    public static double SECOND_ENTRY_X = 102.0;
    public static double SECOND_ENTRY_Y = 56.0;

    public static double SECOND_EXIT_X = 126.0;
    public static double SECOND_EXIT_Y = 56.0;

    // ---------- END ----------
    public static double END_X = 124.0;
    public static double END_Y = 69.548;
    public static double END_H = 270;

    private boolean finishedAndWaited() {
        if (follower.isBusy()) {
            pathEndTime = -1; // reset if still moving
            return false;
        }

        if (pathEndTime < 0) {
            pathEndTime = System.currentTimeMillis();
            return false;
        }

        return System.currentTimeMillis() - pathEndTime >= PATH_DELAY_MS;
    }

    @Override
    public void runOpMode() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(
                START_X,
                START_Y,
                Math.toRadians(START_H)
        ));

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
                    if (finishedAndWaited()) {
                        follower.followPath(paths.shootPreload);
                        state = 2;
                    }
                    break;

                case 2:
                    if (finishedAndWaited()) {
                        follower.followPath(paths.toFirstBalls);
                        state = 3;
                    }
                    break;

                case 3:
                    if (finishedAndWaited()) {
                        follower.followPath(paths.throughFirstBalls);
                        state = 4;
                    }
                    break;

                case 4:
                    if (finishedAndWaited()) {
                        follower.followPath(paths.shootFirstBalls);
                        state = 5;
                    }
                    break;

                case 5:
                    if (finishedAndWaited()) {
                        follower.followPath(paths.toSecondBalls);
                        state = 6;
                    }
                    break;

                case 6:
                    if (finishedAndWaited()) {
                        follower.followPath(paths.throughSecondBalls);
                        state = 7;
                    }
                    break;

                case 7:
                    if (finishedAndWaited()) {
                        follower.followPath(paths.shootSecondBalls);
                        state = 8;
                    }
                    break;

                case 8:
                    if (finishedAndWaited()) {
                        follower.followPath(paths.toEndPosition);
                        state = 9;
                    }
                    break;

                case 9:
                    // End state — do nothing
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

            toAprilTag = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(
                                    START_X,
                                    START_Y
                            ),
                            new Pose(
                                    TAG_X,
                                    TAG_Y
                            )
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(START_H),
                            Math.toRadians(TAG_H)
                    )
                    .build();

            shootPreload = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(
                                    TAG_X,
                                    TAG_Y
                            ),
                            new Pose(
                                    PRELOAD_X,
                                    PRELOAD_Y
                            )
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(TAG_H),
                            Math.toRadians(PRELOAD_H)
                    )
                    .build();

            toFirstBalls = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(
                                    PRELOAD_X,
                                    PRELOAD_Y
                            ),
                            new Pose(
                                    FIRST_ENTRY_X,
                                    FIRST_ENTRY_Y
                            )
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(PRELOAD_H),
                            Math.toRadians(0)
                    )
                    .build();

            throughFirstBalls = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(
                                    FIRST_ENTRY_X,
                                    FIRST_ENTRY_Y
                            ),
                            new Pose(
                                    FIRST_EXIT_X,
                                    FIRST_EXIT_Y
                            )
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0)
                    )
                    .build();

            shootFirstBalls = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(
                                    FIRST_EXIT_X,
                                    FIRST_EXIT_Y
                            ),
                            new Pose(
                                    PRELOAD_X,
                                    PRELOAD_Y
                            )
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(PRELOAD_H)
                    )
                    .build();

            toSecondBalls = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(
                                    PRELOAD_X,
                                    PRELOAD_Y
                            ),
                            new Pose(
                                    SECOND_ENTRY_X,
                                    SECOND_ENTRY_Y
                            )
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(PRELOAD_H),
                            Math.toRadians(0)
                    )
                    .build();

            throughSecondBalls = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(
                                    SECOND_ENTRY_X,
                                    SECOND_ENTRY_Y
                            ),
                            new Pose(
                                    SECOND_EXIT_X,
                                    SECOND_EXIT_Y
                            )
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0)
                    )
                    .build();

            shootSecondBalls = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(
                                    SECOND_EXIT_X,
                                    SECOND_EXIT_Y
                            ),
                            new Pose(
                                    PRELOAD_X,
                                    PRELOAD_Y
                            )
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(PRELOAD_H)
                    )
                    .build();

            toEndPosition = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(
                                    PRELOAD_X,
                                    PRELOAD_Y
                            ),
                            new Pose(
                                    END_X,
                                    END_Y
                            )
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(PRELOAD_H),
                            Math.toRadians(END_H)
                    )
                    .build();
        }
    }
}