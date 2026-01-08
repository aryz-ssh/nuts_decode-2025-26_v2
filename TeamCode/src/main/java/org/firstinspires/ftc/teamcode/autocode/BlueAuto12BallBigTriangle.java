package org.firstinspires.ftc.teamcode.autocode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mechanisms;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Big Triangle Auto - 12 Ball", group = "Autonomous")
@Configurable

public class BlueAuto12BallBigTriangle extends LinearOpMode {
    private Mechanisms mechanisms;

    Follower follower = Constants.createFollower(hardwareMap);

    private enum AutoState {
        START_TO_SHOOT,
        WAIT_TO_SHOOT,

        START_OUTTAKE,
        RAMP_UP,
        WAIT_FOR_HOME,
        SORTER_SETTLE,

        MOVE_SORTER,
        WAIT_SORTER,
        KICK_1,
        WAIT_1,
        KICK_2,
        WAIT_2,
        NEXT_POCKET,

        START_MOVE_AWAY,
        WAIT_MOVE_AWAY,

        RETURN_TO_INTAKE,
        WAIT_RETURN_TO_INTAKE,

        DONE
    }


    @Override
    public void runOpMode() {

        while (!isStarted() && !isStopRequested()) {
            mechanisms.sorterInitLoop();   // homing ONLY
            idle();
        }
        waitForStart();


        Scanmotif(follower);
        follower.followPath(Scanmotif);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        First3BallsintoGoal(follower);
        follower.followPath(First3BallsintoGoal);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        Intake(follower);
        follower.followPath(Intake);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        HitGate(follower);
        follower.followPath(HitGate);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        Shoot1stSetof3Balls(follower);
        follower.followPath(Shoot1stSetof3Balls);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        Intake2ndSetof3Balls(follower);
        follower.followPath(Intake2ndSetof3Balls);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        Shoot2ndSetof3Balls(follower);
        follower.followPath(Shoot2ndSetof3Balls);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        Intake3rdSetof3Balls(follower);
        follower.followPath(Intake3rdSetof3Balls);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        Shoot3rdSetof3Balls(follower);
        follower.followPath(Shoot3rdSetof3Balls);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
    }

    public PathChain Scanmotif;

    private void Scanmotif(Follower follower) {
        Scanmotif = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(32.194410692588086, 135.77642770352375),

                                new Pose(56.559, 113.380)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-300))

                .build();
    }
    //
    public PathChain First3BallsintoGoal;

    private void First3BallsintoGoal(Follower follower) {
        First3BallsintoGoal = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.559, 113.380),

                                new Pose(44.000, 105.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-300), Math.toRadians(-225))

                .build();
    }

    public PathChain Intake;
    private void Intake(Follower follower) {
        Intake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(44.000, 105.000),
                                new Pose(55.000, 80.000),
                                new Pose(15.000, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-225), Math.toRadians(-180))

                .build();
    }
    public PathChain HitGate;
    private void HitGate (Follower follower){
        HitGate = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.000, 84.000),
                                new Pose(24.000, 72.000),
                                new Pose(15.000, 72.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-270))

                .build();
    }
    public PathChain Shoot1stSetof3Balls;
    private void Shoot1stSetof3Balls (Follower follower) {
        Shoot1stSetof3Balls = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.000, 72.000),
                                new Pose(48.000, 96.000),
                                new Pose(44.000, 105.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-270), Math.toRadians(-225))

                .build();
    }
    public PathChain Intake2ndSetof3Balls;
    private void Intake2ndSetof3Balls (Follower follower){
        Intake2ndSetof3Balls = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(44.000, 105.000),
                                new Pose(55.000, 45.000),
                                new Pose(15.000, 60.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-225), Math.toRadians(-180))

                .build();
    }
    public PathChain Shoot2ndSetof3Balls;
    private void Shoot2ndSetof3Balls (Follower follower){
        Shoot2ndSetof3Balls = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.000, 60.000),
                                new Pose(48.000, 72.000),
                                new Pose(44.000, 105.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-225))

                .build();
    }
    public PathChain Intake3rdSetof3Balls;
    private void Intake3rdSetof3Balls (Follower follower){
        Intake3rdSetof3Balls = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(44.000, 105.000),
                                new Pose(72.000, 24.000),
                                new Pose(15.000, 35.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-225), Math.toRadians(-180))

                .build();
    }
    public PathChain Shoot3rdSetof3Balls;
    private void Shoot3rdSetof3Balls (Follower follower){
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
