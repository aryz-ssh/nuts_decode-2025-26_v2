package org.firstinspires.ftc.teamcode.autocode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mechanisms;
@Autonomous(name = "Big Triangle Auto - 12 Ball", group = "Autonomous")
@Configurable

public class BallAutoPathBlue12 extends LinearOpMode  {


    private Mechanisms mechanisms;
    public static class Paths {
    public PathChain Scanmotif;
    public PathChain First3BallsintoGoal;
    public PathChain Intake;
    public PathChain HitGate;
    public PathChain Intake2ndSetof3Balls;
    public PathChain Shoot2ndSetof3Balls;
    public PathChain Intake3rdSetof3Balls;
    public PathChain Shoot3rdSetof3Balls;


    public Paths(Follower follower) {
        Scanmotif = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(32.194410692588086, 135.77642770352375),

                                new Pose(56.559, 113.380)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-300))

                .build();

        First3BallsintoGoal = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.559, 113.380),

                                new Pose(44.000, 105.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-300), Math.toRadians(-225))

                .build();

        Intake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(44.000, 105.000),
                                new Pose(55.000, 80.000),
                                new Pose(15.000, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-225), Math.toRadians(-180))

                .build();

        HitGate = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.000, 84.000),
                                new Pose(24.000, 72.000),
                                new Pose(15.000, 72.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-270))

                .build();

        Shoot2ndSetof3Balls = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.000, 72.000),
                                new Pose(48.000, 96.000),
                                new Pose(44.000, 105.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-270), Math.toRadians(-225))

                .build();

        Intake2ndSetof3Balls = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(44.000, 105.000),
                                new Pose(55.000, 45.000),
                                new Pose(15.000, 60.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-225), Math.toRadians(-180))

                .build();

        Shoot2ndSetof3Balls = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.000, 60.000),
                                new Pose(48.000, 72.000),
                                new Pose(44.000, 105.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-225))

                .build();

        Intake3rdSetof3Balls = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(44.000, 105.000),
                                new Pose(72.000, 24.000),
                                new Pose(15.000, 35.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-225), Math.toRadians(-180))

                .build();

        Shoot3rdSetof3Balls = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.000, 35.000),
                                new Pose(28.500, 67.000),
                                new Pose(44.000, 105.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-225))

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
            if (autoState != BallAutoPathBlue12.AutoState.WAIT_FOR_HOME) {
                follower.update();
            }
        }

}
