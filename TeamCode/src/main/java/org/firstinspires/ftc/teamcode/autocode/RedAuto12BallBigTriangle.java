package org.firstinspires.ftc.teamcode.autocode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mechanisms;
@Autonomous(name = "Red Big Triangle Auto - 12 Ball", group = "Autonomous")
@Configurable

public class RedAuto12BallBigTriangle extends LinearOpMode  {


    private Mechanisms mechanisms;
    public static class Paths {
        public PathChain ScanmotifR;
        public PathChain ShootPreload;
        public PathChain IntakeFirstSetof3Balls;
        public PathChain HitGate;
        public PathChain ShootFirstSetof3Balls;
        public PathChain Intake2ndSetof3Balls;
        public PathChain Shoot2ndSetof3Balls;
        public PathChain Intake3rdSetof3Balls;
        public PathChain Shoot3rdSetof3Balls;

        public Paths(Follower follower) {
            ScanmotifR = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(112.194, 135.776),

                                    new Pose(90.559, 113.380)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(120))

                    .build();

            ShootPreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(90.559, 113.380),

                                    new Pose(88.000, 100.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(43))

                    .build();

            IntakeFirstSetof3Balls = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(88.000, 100.000),
                                    new Pose(94.953, 77.258),
                                    new Pose(129.303, 83.286)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))

                    .build();

            HitGate = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(129.303, 83.286),
                                    new Pose(123.527, 72.098),
                                    new Pose(128.974, 71.783)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))

                    .build();

            ShootFirstSetof3Balls = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(128.974, 71.783),

                                    new Pose(88.000, 100.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(43))

                    .build();

            Intake2ndSetof3Balls = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(88.000, 100.000),
                                    new Pose(55.126, 50.255),
                                    new Pose(130.002, 59.275)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))

                    .build();

            Shoot2ndSetof3Balls = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(130.002, 59.275),

                                    new Pose(88.000, 100.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43))

                    .build();

            Intake3rdSetof3Balls = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(88.000, 100.000),
                                    new Pose(37.244, 30.288),
                                    new Pose(130.002, 35.386)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))

                    .build();

            Shoot3rdSetof3Balls = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(130.002, 35.386),

                                    new Pose(88.000, 100.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43))

                    .build();
        }
    }
    @Override
    public void runOpMode() {
        while (!isStarted() && !isStopRequested()) {
            mechanisms.sorterInitLoop();   // homing ONLY
            idle();
        }
        while (opModeIsActive()) {
            if (autoState != BlueAuto12BallBigTriangle.AutoState.WAIT_FOR_HOME) {
                follower.update();
            }
        }

    }
