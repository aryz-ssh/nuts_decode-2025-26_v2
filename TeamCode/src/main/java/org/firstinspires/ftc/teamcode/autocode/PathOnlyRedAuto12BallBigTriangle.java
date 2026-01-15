package org.firstinspires.ftc.teamcode.autocode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "TEST - 12 Ball RED Pathing ONLY", group = "TEST")
public class PathOnlyRedAuto12BallBigTriangle extends LinearOpMode {

    private Follower follower;

    // Paths
    private PathChain scanMotif;
    private PathChain first3;
    private PathChain intake1;
    private PathChain hitGate;
    private PathChain shoot1;
    private PathChain intake2;
    private PathChain shoot2;
    private PathChain intake3;
    private PathChain shoot3;
    private PathChain end;

    private enum PathStage {
        SCAN_MOTIF,
        FIRST3,
        INTAKE1,
        HIT_GATE,
        SHOOT1,
        INTAKE2,
        SHOOT2,
        INTAKE3,
        SHOOT3,
        END,
        DONE
    }

    private PathStage stage = PathStage.SCAN_MOTIF;

    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(112.194, 135.776, Math.toRadians(90)));

        buildPaths();

        telemetry.addLine("12-Ball Pathing Test Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        followCurrentPath();

        while (opModeIsActive()) {

            follower.update();

            if (!follower.isBusy()) {
                advanceStage();
            }

            Pose pose = follower.getPose();
            telemetry.addData("Stage", stage);
            telemetry.addData("X", pose.getX());
            telemetry.addData("Y", pose.getY());
            telemetry.addData("Heading", Math.toDegrees(pose.getHeading()));
            telemetry.update();
        }
    }

    private void followCurrentPath() {
        follower.followPath(getCurrentPath());
    }

    private void advanceStage() {
        switch (stage) {
            case SCAN_MOTIF: stage = PathStage.FIRST3; break;
            case FIRST3:     stage = PathStage.INTAKE1; break;
            case INTAKE1:    stage = PathStage.HIT_GATE; break;
            case HIT_GATE:   stage = PathStage.SHOOT1; break;
            case SHOOT1:     stage = PathStage.INTAKE2; break;
            case INTAKE2:    stage = PathStage.SHOOT2; break;
            case SHOOT2:     stage = PathStage.INTAKE3; break;
            case INTAKE3:    stage = PathStage.SHOOT3; break;
            case SHOOT3:     stage = PathStage.END; break;
            case END:        stage = PathStage.DONE; break;
            case DONE:       return;
        }

        if (stage != PathStage.DONE) {
            followCurrentPath();
        }
    }

    private PathChain getCurrentPath() {
        switch (stage) {
            case SCAN_MOTIF: return scanMotif;
            case FIRST3:     return first3;
            case INTAKE1:    return intake1;
            case HIT_GATE:   return hitGate;
            case SHOOT1:     return shoot1;
            case INTAKE2:    return intake2;
            case SHOOT2:     return shoot2;
            case INTAKE3:    return intake3;
            case SHOOT3:     return shoot3;
            case END:        return end;
            default:         return null;
        }
    }

    // ================= PATH DEFINITIONS =================

    private void buildPaths() {

        scanMotif = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(112.194, 135.776),
                        new Pose(90.559, 113.380)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(120))
                .build();

        first3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(90.559, 113.380),
                        new Pose(100.000, 105.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(43))
                .build();

        intake1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(100.000, 105.000),
                        new Pose(93.537, 80.327),
                        new Pose(129.303, 83.286)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))
                .build();

        hitGate = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(129.303, 83.286),
                        new Pose(123.527, 72.098),
                        new Pose(128.974, 71.783)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(128.974, 71.783),
                        new Pose(100.000, 105.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(43))
                .build();

        intake2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(100.000, 105.000),
                        new Pose(85.578, 52.852),
                        new Pose(130.002, 59.275)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))
                .build();

        shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(130.002, 59.275),
                        new Pose(100.000, 105.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43))
                .build();

        intake3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(100.000, 105.000),
                        new Pose(68.877, 26.747),
                        new Pose(130.002, 35.386)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))
                .build();

        shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(130.002, 35.386),
                        new Pose(100.000, 105.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43))
                .build();

        end = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(100.000, 105.000),
                        new Pose(100.000, 135.180)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(270))
                .build();
    }
}
