package org.firstinspires.ftc.teamcode.autocode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "TEST - 12 Ball BLUE Pathing ONLY", group = "Test")
@Configurable
public class PathOnlyBlueAuto12BallBigTriangle extends LinearOpMode {

    private Follower follower;

    // ---------------- Paths ----------------
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

    private enum Stage {
        SCAN,
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

    private Stage stage = Stage.SCAN;

    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(32.194, 135.776, Math.toRadians(90)));

        buildPaths();

        telemetry.addLine("Blue 12-Ball PATH-ONLY Ready");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        follower.followPath(scanMotif);

        while (opModeIsActive()) {

            follower.update();

            if (!follower.isBusy()) {
                switch (stage) {

                    case SCAN:
                        follower.followPath(first3);
                        stage = Stage.FIRST3;
                        break;

                    case FIRST3:
                        follower.followPath(intake1);
                        stage = Stage.INTAKE1;
                        break;

                    case INTAKE1:
                        follower.followPath(hitGate);
                        stage = Stage.HIT_GATE;
                        break;

                    case HIT_GATE:
                        follower.followPath(shoot1);
                        stage = Stage.SHOOT1;
                        break;

                    case SHOOT1:
                        follower.followPath(intake2);
                        stage = Stage.INTAKE2;
                        break;

                    case INTAKE2:
                        follower.followPath(shoot2);
                        stage = Stage.SHOOT2;
                        break;

                    case SHOOT2:
                        follower.followPath(intake3);
                        stage = Stage.INTAKE3;
                        break;

                    case INTAKE3:
                        follower.followPath(shoot3);
                        stage = Stage.SHOOT3;
                        break;

                    case SHOOT3:
                        follower.followPath(end);
                        stage = Stage.END;
                        break;

                    case END:
                        stage = Stage.DONE;
                        break;

                    case DONE:
                        // do nothing
                        break;
                }
            }

            telemetry.addData("Stage", stage);
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }

    // ---------------- BUILD PATHS ----------------
    private void buildPaths() {

        scanMotif = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(32.194, 135.776),

                                new Pose(56.559, 113.380)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-300))

                .build();

        first3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.559, 113.380),

                                new Pose(44.000, 105.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-300), Math.toRadians(-225))

                .build();

        intake1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(44.000, 105.000),
                                new Pose(61.937, 79.043),
                                new Pose(15.000, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-225), Math.toRadians(-180))

                .build();

        hitGate = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.000, 84.000),
                                new Pose(26.597, 79.790),
                                new Pose(15.000, 76.721)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-270))

                .build();

        shoot1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.000, 77.254),
                                new Pose(48.000, 96.000),
                                new Pose(44.000, 105.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-270), Math.toRadians(-225))

                .build();

        intake2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(44.000, 105.000),
                                new Pose(75.332, 51.219),
                                new Pose(15.000, 60.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-225), Math.toRadians(-180))

                .build();

        shoot2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.000, 60.000),
                                new Pose(48.000, 72.000),
                                new Pose(44.000, 105.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-225))

                .build();

        intake3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(44.000, 105.000),
                                new Pose(86.635, 21.261),
                                new Pose(15.000, 35.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-225), Math.toRadians(-180))

                .build();

        shoot3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(14.522, 37.388),
                                new Pose(28.500, 67.000),
                                new Pose(44.000, 105.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-225))

                .build();

        end = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(44.000, 105.000),

                                new Pose(44.000, 135.776)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-225), Math.toRadians(-90))

                .build();
    }
}

