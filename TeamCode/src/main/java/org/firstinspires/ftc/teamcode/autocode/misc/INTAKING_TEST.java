/*
package org.firstinspires.ftc.teamcode.autocode.misc;

import com.acmerobotics.dashboard.config.Config;
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
@Config
public class INTAKING_TEST extends LinearOpMode {

    private Follower follower;
    private Paths paths;
    private Mechanisms mechanisms;

    private IntakeTestState state = IntakeTestState.START_SWEEP;
    private long settleStartMs = 0;

    public static double INTAKE_POWER = 1.0;
    public static double INTAKE_SPEED_LIMIT = 0.35;
    public static long INTAKE_SETTLE_MS = 1000;

    private enum IntakeTestState {
        START_SWEEP,
        WAIT_SWEEP,
        SETTLE,
        DONE
    }

    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(42.4, 84.4, Math.toRadians(180)));

        paths = new Paths(follower);

        mechanisms = new Mechanisms();
        mechanisms.initMechanisms(hardwareMap, telemetry, true);

        telemetry.addLine("INTAKE AUTO TEST READY");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            follower.update();
            mechanisms.updateMechanisms(); // REQUIRED

            switch (state) {

                case START_SWEEP:
                    // EXACTLY what autos do
                    follower.setMaxPower(INTAKE_SPEED_LIMIT);
                    follower.followPath(paths.Path1);

                    mechanisms.engageIntake(INTAKE_POWER, false);
                    mechanisms.sorter.setAutoMode(true);

                    state = IntakeTestState.WAIT_SWEEP;
                    break;

                case WAIT_SWEEP:
                    if (!follower.isBusy()) {
                        follower.setMaxPower(Constants.driveConstants.maxPower);
                        settleStartMs = System.currentTimeMillis();
                        state = IntakeTestState.SETTLE;
                    }
                    break;

                case SETTLE:
                    // Let balls finish seating
                    if (System.currentTimeMillis() - settleStartMs > INTAKE_SETTLE_MS) {
                        mechanisms.disengageIntake();
                        mechanisms.sorter.setAutoMode(false);
                        state = IntakeTestState.DONE;
                    }
                    break;

                case DONE:
                    mechanisms.disengageIntake();
                    break;
            }

            telemetry.addData("State", state);
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.update();
        }
    }

    // ---------------- PATH ----------------
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

*/
