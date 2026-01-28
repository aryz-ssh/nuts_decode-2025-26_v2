package org.firstinspires.ftc.teamcode.autocode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mechanisms;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Intake in a straight line test", group = "Autonomous")
@Configurable
public class INTAKING_TEST extends LinearOpMode {

    private Follower follower;
    private INTAKING_TEST.Paths paths;
    private Mechanisms mechanisms;

    private enum IntakeTestState {
        START_PATH,
        WAIT_PATH,
        DONE
    }

    private IntakeTestState state = IntakeTestState.START_PATH;

    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(42.4, 84.4, Math.toRadians(180)));

        paths = new INTAKING_TEST.Paths(follower);

        mechanisms = new Mechanisms();
        mechanisms.initMechanisms(hardwareMap, telemetry, true);

        telemetry.addLine("INTAKE TEST READY");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            follower.update();
            mechanisms.updateMechanisms(); // REQUIRED

            switch (state) {

                case START_PATH:
                    // Turn intake ON before moving
                    mechanisms.engageIntake(1.0, false);

                    follower.followPath(paths.Path1);
                    state = IntakeTestState.WAIT_PATH;
                    break;

                case WAIT_PATH:
                    if (!follower.isBusy()) {
                        mechanisms.disengageIntake();
                        state = IntakeTestState.DONE;
                    }
                    break;

                case DONE:
                    // Hold everything off
                    mechanisms.disengageIntake();
                    break;
            }

            telemetry.addData("State", state);
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.update();
        }
    }

    // ---------------- PATHS ----------------
    public static class Paths {

        public PathChain Path1;

        public Paths(Follower follower) {

            Path1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(42.400, 84.400),
                                    new Pose(18.800, 84.492)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(180)
                    )
                    .build();
        }
    }
}
