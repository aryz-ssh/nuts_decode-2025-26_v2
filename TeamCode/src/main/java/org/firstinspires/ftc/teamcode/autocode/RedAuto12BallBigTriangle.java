package org.firstinspires.ftc.teamcode.autocode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Mechanisms;
import org.firstinspires.ftc.teamcode.SorterLogicColor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Big Triangle Auto - 12 Ball", group = "Autonomous")
@Configurable
public class RedAuto12BallBigTriangle extends LinearOpMode {

    private Mechanisms mechanisms;

    /* ===================== PATHS ===================== */

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

    private Follower follower;

    public static long INTAKE_SETTLE_MS = 750; // dashboard-tunable
    public static double INTAKE_PATH_SPEED = 0.75;
    public static int MOTIF_PIPELINE = 2;
    public static int RED_PIPELINE = 0;
    public static long MOTIF_SCAN_TIMEOUT_MS = 1500;
    public static double OUTTAKE_POWER = 0.7;
    public static double RAMP_POSITION = 0.7;
    private boolean isPreloadPhase = true;

    private int detectedMotifId = -1;

    @Override
    public void runOpMode() {

        mechanisms = new Mechanisms();
        mechanisms.initMechanisms(hardwareMap, telemetry);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();

        telemetry.addLine("Initialized (Mechanisms + Follower)");
        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) return;

        // ---------- HOME ----------
        homeAllMechanisms();
        mechanisms.setRampAngle(RAMP_POSITION);

        // ---------- LIMELIGHT ----------
        scanMotifWithLimelight();

        // ---------- BEGINNING ----------
        mechanisms.engageOuttake(OUTTAKE_POWER);
        runPath(scanMotif, 1.0);

        // ----- SHOOT PRELOAD -----
        runPath(first3, 1.0);
        // correctPoseFromLimelight();
        shootAllPockets();
        mechanisms.disengageOuttake();
        isPreloadPhase = false;

        // ----- INTAKE 1 -----
        startIntake();
        runPath(intake1, INTAKE_PATH_SPEED);

        // settle AFTER intake path
        long t0 = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - t0 < INTAKE_SETTLE_MS) {
            mechanisms.updateMechanisms();
            sleep(10);
        }

        // stopIntake();

        // ----- HIT GATE -----
        runPath(hitGate, 1.0);

        // ----- SHOOT SECOND SET -----
        mechanisms.engageOuttake(OUTTAKE_POWER);
        runPath(shoot1, 1.0);

        stopIntake();

        // correctPoseFromLimelight();
        shootAllPockets();
        mechanisms.disengageOuttake();

        // ----- INTAKE 2 -----
        startIntake();
        runPath(intake2, INTAKE_PATH_SPEED);

        // settle AFTER intake path
        t0 = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - t0 < INTAKE_SETTLE_MS) {
            mechanisms.updateMechanisms();
            sleep(10);
        }

        // stopIntake();

        // ----- SHOOT THIRD SET -----
        mechanisms.engageOuttake(OUTTAKE_POWER);
        runPath(shoot2,1.0);

        stopIntake();

        // correctPoseFromLimelight();
        shootAllPockets();
        mechanisms.disengageOuttake();

        // ----- INTAKE 3 -----
        startIntake();
        runPath(intake3, INTAKE_PATH_SPEED);

        // settle AFTER intake path
        t0 = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - t0 < INTAKE_SETTLE_MS) {
            mechanisms.updateMechanisms();
            sleep(10);
        }

        //stopIntake();

        // ----- SHOOT FOURTH SET -----
        mechanisms.engageOuttake(OUTTAKE_POWER);
        runPath(shoot3,1.0);

        stopIntake();

        // correctPoseFromLimelight();
        shootAllPockets();
        mechanisms.disengageOuttake();

        mechanisms.sorterGoToIntake(1);
        mechanisms.setRampAngle(Mechanisms.RAMP_ANGLE_MIN_POS);
        runPath(end, 1.0);
    }

    private void homeAllMechanisms() {

        telemetry.addLine("Homing sorter...");
        telemetry.update();

        // Run sorter init loop until homed
        while (opModeIsActive() && !mechanisms.isSorterHomed()) {
            mechanisms.sorterInitLoop();
            mechanisms.updateMechanisms();
            sleep(10);
        }

        telemetry.addLine("Sorter homed");
        telemetry.update();
    }

    private void scanMotifWithLimelight() {

        telemetry.addLine("Scanning motif...");
        telemetry.update();

        mechanisms.limelight.pipelineSwitch(MOTIF_PIPELINE);

        long start = System.currentTimeMillis();
        boolean found = false;

        while (opModeIsActive()
                && !found
                && System.currentTimeMillis() - start < MOTIF_SCAN_TIMEOUT_MS) {

            LLResult result = mechanisms.limelight.getLatestResult();

            if (result != null) {

                // FTC-SAFE fiducial access
                java.util.List<LLResultTypes.FiducialResult> fiducials =
                        result.getFiducialResults();

                if (fiducials != null && !fiducials.isEmpty()) {
                    found = true;

                    detectedMotifId = fiducials.get(0).getFiducialId();
                    telemetry.addData("Motif detected", detectedMotifId);
                }
            }

            telemetry.update();
            sleep(20);
        }


        if (!found) {
            detectedMotifId = 21; // default GPP (or whatever your safe default is)
            telemetry.addData("Motif defaulted", detectedMotifId);
        }

        telemetry.update();
        mechanisms.limelight.pipelineSwitch(RED_PIPELINE);
    }

    private void correctPoseFromLimelight() {

        LLResult result = mechanisms.limelight.getLatestResult();
        if (result == null || !result.isValid()) return;
        if (result.getBotposeTagCount() < 1) return;

        Pose3D pose3d = result.getBotpose_MT2();
        if (pose3d == null) return;

        Position pos = pose3d.getPosition();
        YawPitchRollAngles rot = pose3d.getOrientation();

        // Limelight botpose is METERS (per SDK)
        double xInches = pos.x * 39.3701;
        double yInches = pos.y * 39.3701;

        // SDK uses yaw = firstAngle (degrees)
        double headingRad = Math.toRadians(rot.getYaw(AngleUnit.DEGREES));
        // double headingRad = Math.toRadians(rot.getYaw(AngleUnit.DEGREES)) + Math.PI;

        Pose correctedPose = new Pose(xInches, yInches, headingRad);

        follower.setPose(correctedPose);
    }



    private void shootAllPockets() {
        for (int step = 0; step < 3; step++) {

            int pocket;

            // ---------- PRELOAD ----------
            if (isPreloadPhase) {
                pocket = step + 1;
            }
            // ---------- SORTED PHASE ----------
            else {
                SorterLogicColor.BallColor required =
                        getRequiredColorForStep(step);

                if (required == SorterLogicColor.BallColor.UNKNOWN)
                    continue;

                Integer foundPocket =
                        mechanisms.sorterLogic.getPocketWithColor(required);

                if (foundPocket == null)
                    continue; // required color not present

                pocket = foundPocket;
            }

            // 1) Rotate sorter to selected pocket
            mechanisms.sorterGoToOuttake(pocket);

            // 2) Wait until sorter reaches target
            while (opModeIsActive() && mechanisms.isSorterMoving()) {
                mechanisms.updateMechanisms();
                sleep(10);
            }

            // Small settle delay
            sleep(50);

            // 3) Fire exactly once (THIS clears the pocket)
            mechanisms.setShotPocket(pocket);
            mechanisms.ejectBall();

            // 4) Allow kicker + spacing time
            long kickStart = System.currentTimeMillis();
            while (opModeIsActive()
                    && System.currentTimeMillis() - kickStart < 500) {

                mechanisms.updateMechanisms();
                sleep(10);
            }
        }
    }

    private void startIntake() {
        mechanisms.sorterLogic.autoAdvanceEnabled = true;

        // reset intake sequence to pocket 1 so detection/storage is deterministic
        mechanisms.sorterGoToIntake(1);

        // wait briefly for the sorter to get there (or at least stop moving)
        long t0 = System.currentTimeMillis();
        while (opModeIsActive()
                && mechanisms.isSorterMoving()
                && System.currentTimeMillis() - t0 < 500) {
            mechanisms.updateMechanisms();
            sleep(10);
        }

        mechanisms.engageIntake(1.0, true);
    }

    private void stopIntake() {
        mechanisms.disengageIntake();
    }


    private SorterLogicColor.BallColor getRequiredColorForStep(int step) {
        switch (detectedMotifId) {

            case 21: // GPP
                if (step == 0) return SorterLogicColor.BallColor.GREEN;
                if (step == 1) return SorterLogicColor.BallColor.PURPLE;
                if (step == 2) return SorterLogicColor.BallColor.PURPLE;
                break;

            case 22: // PGP
                if (step == 0) return SorterLogicColor.BallColor.PURPLE;
                if (step == 1) return SorterLogicColor.BallColor.GREEN;
                if (step == 2) return SorterLogicColor.BallColor.PURPLE;
                break;

            case 23: // PPG
                if (step == 0) return SorterLogicColor.BallColor.PURPLE;
                if (step == 1) return SorterLogicColor.BallColor.PURPLE;
                if (step == 2) return SorterLogicColor.BallColor.GREEN;
                break;
        }

        return SorterLogicColor.BallColor.UNKNOWN;
    }

    private String getMotifName(int id) {
        switch (id) {
            case 21: return "GPP";
            case 22: return "PGP";
            case 23: return "PPG";
            default: return "UNKNOWN";
        }
    }

    /* ===================== PATH EXEC ===================== */

    private void runPath(PathChain path, double speedScale) {
        follower.setMaxPower(speedScale);
        follower.followPath(path);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            mechanisms.updateMechanisms();
        }

        follower.setMaxPower(1.0); // restore default
    }

    /* ===================== PATH BUILD ===================== */

    private void buildPaths() {

        scanMotif = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(112.194, 135.776),
                        new Pose(90.559, 113.380)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(90),
                        Math.toRadians(120)
                )
                .build();

        first3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(90.559, 113.380),
                        new Pose(100.000, 105.000)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(120),
                        Math.toRadians(43)
                )
                .build();

        intake1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(100.000, 105.000),
                        new Pose(93.537, 80.327),
                        new Pose(129.303, 83.286)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(43),
                        Math.toRadians(0)
                )
                .build();

        hitGate = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(129.303, 83.286),
                        new Pose(123.527, 72.098),
                        new Pose(128.974, 71.783)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(90)
                )
                .build();

        shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(128.974, 71.783),
                        new Pose(100.000, 105.000)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(90),
                        Math.toRadians(43)
                )
                .build();

        intake2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(100.000, 105.000),
                        new Pose(85.578, 52.852),
                        new Pose(130.002, 59.275)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(43),
                        Math.toRadians(0)
                )
                .build();

        shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(130.002, 59.275),
                        new Pose(100.000, 105.000)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(43)
                )
                .build();

        intake3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(100.000, 105.000),
                        new Pose(68.877, 26.747),
                        new Pose(130.002, 35.386)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(43),
                        Math.toRadians(0)
                )
                .build();

        shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(130.002, 35.386),
                        new Pose(100.000, 105.000)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(43)
                )
                .build();

        end = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(100.000, 105.000),
                        new Pose(100.000, 135.180)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(43),
                        Math.toRadians(270)
                )
                .build();
    }
}
