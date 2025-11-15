package org.firstinspires.ftc.teamcode.pedroPathing;

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

import java.util.ArrayList;

@Autonomous(name = "AutoCode", group = "Autonomous")
@Configurable
public class AutoCode extends LinearOpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private Paths paths;
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
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower);

        mechanisms = new Mechanisms();
        mechanisms.initMechanisms(hardwareMap, telemetry);

        // initial dummy balls
        intakeOrder.add("green");
        intakeOrder.add("purple");
        intakeOrder.add("green");

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        waitForStart();

        int state = 0;
        while (opModeIsActive()) {

            follower.update();

            switch (state) {

                case 0:
                    follower.followPath(paths.P1);
                    state = 1;
                    break;

                case 1:
                    if (!follower.isBusy()) {
                        shootAll();
                        startDelay();
                        state = 2;
                    }
                    break;

                case 2:
                    if (delayDone()) {
                        follower.followPath(paths.P2);
                        state = 3;
                    }
                    break;

                case 3:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.P3);
                        state = 4;
                    }
                    break;

                case 4:
                    follower.followPath(paths.P4);
                    intakeOn = true;
                    state = 5;
                    break;

                case 5:
                    if (!follower.isBusy()) {
                        intakeOn = false;
                        follower.followPath(paths.P5);
                        state = 6;
                    }
                    break;

                case 6:
                    if (!follower.isBusy()) {
                        shootAll();
                        startDelay();
                        state = 7;
                    }
                    break;

                case 7:
                    if (delayDone()) {
                        follower.followPath(paths.P6);
                        state = 8;
                    }
                    break;

                case 8:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.P7);
                        intakeOn = true;
                        state = 9;
                    }
                    break;

                case 9:
                    if (!follower.isBusy()) {
                        intakeOn = false;
                        follower.followPath(paths.P8);
                        state = 10;
                    }
                    break;

                case 10:
                    if (!follower.isBusy()) {
                        shootAll();
                        startDelay();
                        state = 11;
                    }
                    break;

                case 11:
                    if (delayDone()) {
                        follower.followPath(paths.P9);
                        state = 12;
                    }
                    break;

                case 12:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.P10);
                        intakeOn = true;
                        state = 13;
                    }
                    break;

                case 13:
                    if (!follower.isBusy()) {
                        intakeOn = false;
                        follower.followPath(paths.P11);
                        state = 14;
                    }
                    break;

                case 14:
                    if (!follower.isBusy()) {
                        shootAll();
                        state = 15;
                    }
                    break;

                case 15:
                    // done
                    intakeOn = false;
                    break;
            }

            handleIntakeSorting();

            panelsTelemetry.debug("State", state);
            panelsTelemetry.debug("X", follower.getPose().getX());
            panelsTelemetry.debug("Y", follower.getPose().getY());
            panelsTelemetry.update(telemetry);
        }
    }

    // ---------------- PATH LIST ----------------
    public static class Paths {

        public PathChain P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11;

        public Paths(Follower follower) {

            P1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(61.341, 12.181), new Pose(61.341, 95.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(141))
                    .build();

            P2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(61.341, 95.00), new Pose(61.341, 85.20)))
                    .setLinearHeadingInterpolation(Math.toRadians(141), Math.toRadians(90))
                    .build();

            P3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(61.341, 85.20), new Pose(41.20, 35.674)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            P4 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(41.20, 35.674), new Pose(14.40, 35.674)))
                    .setReversed()
                    .build();

            P5 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(14.40, 35.674), new Pose(58.700, 9.200)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120))
                    .build();

            P6 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(58.700, 9.200), new Pose(50.429, 60.036)))
                    .build();

            P7 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(50.429, 60.036), new Pose(12.800, 60.036)))
                    .setReversed()
                    .build();

            P8 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(12.800, 60.036), new Pose(60.471, 83.529)))
                    .setReversed()
                    .build();

            P9 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(60.471, 83.529), new Pose(41.547, 83.529)))
                    .build();

            P10 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(41.547, 83.529), new Pose(15.444, 83.529)))
                    .build();

            P11 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(15.444, 83.529), new Pose(60.254, 83.529)))
                    .build();
        }
    }

    // ---------------- INTAKE + COLOR SORTING ----------------
    private void handleIntakeSorting() {

        if (intakeOn) {

            mechanisms.engageIntake(1.0, false);

            String detectedColor = mechanisms.getBottomBallColor();
            if (detectedColor != null && !detectedColor.equals(lastDetectedColor) && !detectedColor.equals("unknown")) {
                intakeOrder.add(detectedColor);
                lastDetectedColor = detectedColor;
            }

        } else {
            mechanisms.disengageIntake();
            lastDetectedColor = null;
        }
    }

    // ---------------- SHOOTING ----------------
    private void shootAll() {

        while (!intakeOrder.isEmpty()) {

            // Remove top ball using Mechanisms


            // Start outtake
            mechanisms.engageOuttake(1.0);

            // Short delay to allow shooting
            long shootStart = System.currentTimeMillis();
            while (opModeIsActive() && System.currentTimeMillis() - shootStart < 300) {
                // just wait, follower still updates in LinearOpMode loop
            }

            mechanisms.disengageOuttake();

            // Remove the ball from the queue after shooting
            intakeOrder.remove(0);
        }
    }

    // ---------------- DELAYS ----------------
    private void startDelay() {
        delayStart = System.currentTimeMillis();
    }

    private boolean delayDone() {
        return System.currentTimeMillis() - delayStart >= DELAY_MS;
    }
}
