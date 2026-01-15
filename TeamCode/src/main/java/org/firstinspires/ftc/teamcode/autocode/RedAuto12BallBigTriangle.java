package org.firstinspires.ftc.teamcode.autocode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AprilTagLimelight;
import org.firstinspires.ftc.teamcode.Mechanisms;
import org.firstinspires.ftc.teamcode.SorterLogicColor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private AprilTagLimelight limelight;
    enum AutoState {
        START_PATH,
        WAIT_PATH,

        START_INTAKE,
        WAIT_INTAKE,
        INTAKE_SETTLE,

        START_SHOOT,
        WAIT_RAMP,
        MOVE_SORTER,
        WAIT_SORTER,
        SORTER_SETTLE,
        KICK_1,
        WAIT_1,
        NEXT_POCKET,

        DONE
    }

    private enum DriveStage {
        SCAN_MOTIF,
        TO_SHOOT_PRELOAD,
        INTAKE_1,
        HIT_GATE,
        SHOOT_1,
        INTAKE_2,
        SHOOT_2,
        INTAKE_3,
        SHOOT_3,
        END,
        DONE
    }


    private DriveStage driveStage = DriveStage.SCAN_MOTIF;
    public static double RAMP_UP_TIME = 0.1;        // seconds
    public static double SORTER_SETTLE_TIME = 0.35;  // seconds
    public static double FIRST_KICK_DELAY = 0.50;    // seconds

    AutoState autoState = AutoState.START_PATH;

    int currentPocket = 1;
    int activeShotPocket = 1;
    boolean sorterCommanded = false;
    boolean intakeRunning = false;

    ElapsedTime stateTimer = new ElapsedTime();

    public static long INTAKE_SETTLE_MS = 750; // dashboard-tunable
    public static double INTAKE_PATH_SPEED = 0.75;
    public static int MOTIF_PIPELINE = 2;
    public static int RED_PIPELINE = 0;
    public static long MOTIF_SCAN_TIMEOUT_MS = 1500;
    public static double OUTTAKE_POWER = 0.7;
    public static double RAMP_POSITION = 0.7;
    private boolean isPreloadPhase = true;
    private boolean shooterSpinning = false;

    private int detectedMotifId = -1;

    @Override
    public void runOpMode() {

        mechanisms = new Mechanisms();
        mechanisms.initMechanisms(hardwareMap, telemetry);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(112.194, 135.776, Math.toRadians(90)));
        buildPaths();

        telemetry.addLine("Initialized (Mechanisms + Follower)");
        telemetry.update();

        // --- PRE-START HOMING LOOP (like your 3-ball) ---
        while (!isStarted() && !isStopRequested()) {
            mechanisms.sorterInitLoop();   // homing only
            mechanisms.updateMechanisms();
            telemetry.addLine("Homing...");
            telemetry.update();
            idle();
        }

        waitForStart();
        if (!opModeIsActive()) return;

//        scanMotifWithLimelight();

        // safe defaults at start
        mechanisms.disengageOuttake();
        shooterSpinning = false;
        mechanisms.disengageIntake();
        mechanisms.setRampAngle(Mechanisms.RAMP_ANGLE_MIN_POS);

        autoState = AutoState.START_PATH;
        driveStage = DriveStage.SCAN_MOTIF;     // see section #2 below
        currentPocket = 1;
        sorterCommanded = false;

        while (opModeIsActive()) {

            follower.update();
            mechanisms.updateMechanisms();

            switch (autoState) {

                // =============================
                // DRIVE (start a path)
                // =============================
                case START_PATH: {
                    PathChain p = getCurrentPath();
                    double spd = getCurrentSpeedScale();

                    // 🔥 PRE-SPIN ONLY when driving TO a shoot position
                    if (isShootPath(driveStage) && !shooterSpinning) {
                        mechanisms.engageOuttake(OUTTAKE_POWER);
                        shooterSpinning = true;
                    }

                    follower.setMaxPower(spd);
                    follower.followPath(p);

                    autoState = AutoState.WAIT_PATH;
                    break;
                }

                // =============================
                // DRIVE (wait for path end)
                // =============================
                case WAIT_PATH: {
                    if (!follower.isBusy()) {
                        follower.setMaxPower(1.0);

                        // After certain paths, we do intake settle or shooting.
                        if (isIntakePathJustFinished()) {
                            stateTimer.reset();
                            autoState = AutoState.INTAKE_SETTLE;
                        } else if (isShootPathJustFinished()) {
                            autoState = AutoState.START_SHOOT;
                        } else {
                            // scanMotif, hitGate, end, first3 transitions
                            advanceAfterNonIntakeNonShootPath();
                        }
                    }
                    break;
                }

                // =============================
                // INTAKE (start before intake path)
                // =============================
                case START_INTAKE: {
                    // start intake + auto-advance BEFORE driving intake path
                    mechanisms.sorterLogic.autoAdvanceEnabled = true;
                    mechanisms.sorterGoToIntake(1);
                    mechanisms.engageIntake(1.0, true);
                    intakeRunning = true;

                    autoState = AutoState.START_PATH; // will run the intake path next
                    break;
                }

                // =============================
                // INTAKE (settle after intake path)
                // =============================
                case INTAKE_SETTLE: {
                    if (stateTimer.milliseconds() >= INTAKE_SETTLE_MS) {
                        // stop intake after settle
                        mechanisms.disengageIntake();
                        mechanisms.sorterLogic.autoAdvanceEnabled = false;
                        intakeRunning = false;

                        // after intake1 -> run hitGate
                        // after intake2/intake3 -> go straight to shoot path
                        advanceAfterIntakeSettled();
                    }
                    break;
                }

                // =============================
                // SHOOT (start spool/ramp)
                // =============================
                case START_SHOOT: {
                    mechanisms.setRampAngle(RAMP_POSITION);
                    stateTimer.reset();
                    autoState = AutoState.WAIT_RAMP;
                    break;
                }


                case WAIT_RAMP: {
                    if (stateTimer.seconds() >= RAMP_UP_TIME) {
                        currentPocket = 1;
                        sorterCommanded = false;
                        autoState = AutoState.MOVE_SORTER;
                    }
                    break;
                }

                case MOVE_SORTER: {
                    if (!sorterCommanded) {

                        // preload phase is deterministic
                        if (isPreloadPhase) {
                            activeShotPocket = currentPocket;   // preload is deterministic
                            mechanisms.sorterGoToOuttake(activeShotPocket);
                        } else {
                            // ---------- NO MOTIF → NO SORTING ----------
                            if (!shouldUseSorting()) {
                                activeShotPocket = currentPocket;
                                mechanisms.sorterGoToOuttake(activeShotPocket);
                            }
                            // ---------- MOTIF PRESENT → SORTING ----------
                            else {
                                SorterLogicColor.BallColor required =
                                        getRequiredColorForStep(currentPocket - 1);

                                Integer pocket =
                                        mechanisms.sorterLogic.getPocketWithColor(required);

                                if (pocket == null || required == SorterLogicColor.BallColor.UNKNOWN) {
                                    currentPocket++;
                                    sorterCommanded = false;

                                    if (currentPocket > 3) {
                                        mechanisms.disengageOuttake();
                                        shooterSpinning = false;
                                        advanceAfterShotSet();
                                    }
                                    break;
                                }

                                activeShotPocket = pocket;
                                mechanisms.sorterGoToOuttake(pocket);
                            }
                        }

                        sorterCommanded = true;
                    }
                    autoState = AutoState.WAIT_SORTER;
                    break;
                }

                case WAIT_SORTER: {
                    if (!mechanisms.isSorterMoving()) {
                        stateTimer.reset();
                        autoState = AutoState.SORTER_SETTLE;
                    }
                    break;
                }

                case SORTER_SETTLE: {
                    if (stateTimer.seconds() >= SORTER_SETTLE_TIME) {
                        autoState = AutoState.KICK_1;
                    }
                    break;
                }

                case KICK_1: {
                    // IMPORTANT: pocket to shoot must match what you commanded.
                    // For preload it's currentPocket.
                    // For sorted you used mechanisms.setShotPocket(pocket) in your old code,
                    // but your Mechanisms likely tracks selected pocket internally.
                    mechanisms.setShotPocket(activeShotPocket);
                    mechanisms.ejectBall();
                    stateTimer.reset();
                    autoState = AutoState.WAIT_1;
                    break;
                }

                case WAIT_1: {
                    if (stateTimer.seconds() >= FIRST_KICK_DELAY) {
                        autoState = AutoState.NEXT_POCKET;
                    }
                    break;
                }

                case NEXT_POCKET: {
                    currentPocket++;
                    sorterCommanded = false;

                    if (currentPocket > 3) {
                        mechanisms.disengageOuttake();
                        shooterSpinning = false;

                        // after preload shot, switch to sorted phase
                        if (isPreloadPhase) isPreloadPhase = false;

                        advanceAfterShotSet(); // decides next drive stage (intake1/2/3/end)
                    } else {
                        autoState = AutoState.MOVE_SORTER;
                    }
                    break;
                }

                case DONE: {
                    // park safe
                    mechanisms.disengageIntake();
                    mechanisms.disengageOuttake();
                    shooterSpinning = false;
                    mechanisms.setRampAngle(Mechanisms.RAMP_ANGLE_MIN_POS);
                    break;
                }
            }

            telemetry.addData("State", autoState);
            telemetry.addData("DriveStage", driveStage);
            telemetry.addData("Pocket", currentPocket);
            telemetry.addData("Motif", detectedMotifId == -1 ? "NONE" : getMotifName(detectedMotifId));
            telemetry.addData("Sorting", shouldUseSorting());
            telemetry.update();
        }
    }

//    private void scanMotifWithLimelight() {
//
//        telemetry.addLine("Scanning motif...");
//        telemetry.update();
//
//        mechanisms.limelight.pipelineSwitch(MOTIF_PIPELINE);
//
//        long start = System.currentTimeMillis();
//        boolean found = false;
//
//        while (opModeIsActive()
//                && !found
//                && System.currentTimeMillis() - start < MOTIF_SCAN_TIMEOUT_MS) {
//
//            LLResult result = mechanisms.limelight.getLatestResult();
//
//            if (result != null) {
//
//                // FTC-SAFE fiducial access
//                java.util.List<LLResultTypes.FiducialResult> fiducials =
//                        result.getFiducialResults();
//
//                if (fiducials != null && !fiducials.isEmpty()) {
//                    found = true;
//
//                    detectedMotifId = fiducials.get(0).getFiducialId();
//                    telemetry.addData("Motif detected", detectedMotifId);
//                }
//            }
//
//            telemetry.update();
//            sleep(20);
//        }
//
//
//        if (!found) {
//            detectedMotifId = -1; // NO MOTIF
//            telemetry.addData("Motif defaulted", detectedMotifId);
//        }
//
//        telemetry.update();
//        mechanisms.limelight.pipelineSwitch(RED_PIPELINE);
//    }

    private boolean shouldUseSorting() {
        return detectedMotifId == 21
                || detectedMotifId == 22
                || detectedMotifId == 23;
    }

    private PathChain getCurrentPath() {
        switch (driveStage) {
            case SCAN_MOTIF:         return scanMotif;
            case TO_SHOOT_PRELOAD:   return first3;
            case INTAKE_1:           return intake1;
            case HIT_GATE:           return hitGate;
            case SHOOT_1:            return shoot1;
            case INTAKE_2:           return intake2;
            case SHOOT_2:            return shoot2;
            case INTAKE_3:           return intake3;
            case SHOOT_3:            return shoot3;
            case END:                return end;
            default:                 return end;
        }
    }

    private boolean isShootPath(DriveStage stage) {
        return stage == DriveStage.TO_SHOOT_PRELOAD
                || stage == DriveStage.SHOOT_1
                || stage == DriveStage.SHOOT_2
                || stage == DriveStage.SHOOT_3;
    }

    private double getCurrentSpeedScale() {
        // only slow intake paths
        switch (driveStage) {
            case INTAKE_1:
            case INTAKE_2:
            case INTAKE_3:
                return INTAKE_PATH_SPEED;
            default:
                return 1.0;
        }
    }

    private boolean isIntakePathJustFinished() {
        return driveStage == DriveStage.INTAKE_1
                || driveStage == DriveStage.INTAKE_2
                || driveStage == DriveStage.INTAKE_3;
    }

    private boolean isShootPathJustFinished() {
        return driveStage == DriveStage.TO_SHOOT_PRELOAD
                || driveStage == DriveStage.SHOOT_1
                || driveStage == DriveStage.SHOOT_2
                || driveStage == DriveStage.SHOOT_3;
    }

    private void advanceAfterNonIntakeNonShootPath() {
        switch (driveStage) {
            case SCAN_MOTIF:
                // you can scan here, then drive to preload shoot
                // (scanMotifWithLimelight can be called before running SCAN_MOTIF if you want)
                driveStage = DriveStage.TO_SHOOT_PRELOAD;
                autoState = AutoState.START_PATH;
                break;

            case TO_SHOOT_PRELOAD:
                autoState = AutoState.START_SHOOT;
                break;

            case HIT_GATE:
                driveStage = DriveStage.SHOOT_1;
                autoState = AutoState.START_PATH;
                break;

            case END:
                driveStage = DriveStage.DONE;
                autoState = AutoState.DONE;
                break;

            default:
                // should not happen
                autoState = AutoState.DONE;
                break;
        }
    }

    private void advanceAfterIntakeSettled() {
        switch (driveStage) {
            case INTAKE_1:
                // go to hit gate after intake1
                driveStage = DriveStage.HIT_GATE;
                autoState = AutoState.START_PATH;
                break;

            case INTAKE_2:
                driveStage = DriveStage.SHOOT_2; // shoot2 path returns to shoot pos
                autoState = AutoState.START_PATH;
                break;

            case INTAKE_3:
                driveStage = DriveStage.SHOOT_3;
                autoState = AutoState.START_PATH;
                break;

            default:
                autoState = AutoState.DONE;
                break;
        }
    }

    private void advanceAfterShotSet() {
        // decides what we do after finishing 3 shots at shoot position
        if (driveStage == DriveStage.TO_SHOOT_PRELOAD) {
            // after preload shots, start first intake cycle
            driveStage = DriveStage.INTAKE_1;
            autoState = AutoState.START_INTAKE;
            return;
        }

        if (driveStage == DriveStage.SHOOT_1) {
            driveStage = DriveStage.INTAKE_2;
            autoState = AutoState.START_INTAKE;
            return;
        }

        if (driveStage == DriveStage.SHOOT_2) {
            driveStage = DriveStage.INTAKE_3;
            autoState = AutoState.START_INTAKE;
            return;
        }

        if (driveStage == DriveStage.SHOOT_3) {
            // park
            mechanisms.sorterGoToIntake(1);
            mechanisms.setRampAngle(Mechanisms.RAMP_ANGLE_MIN_POS);
            driveStage = DriveStage.END;
            autoState = AutoState.START_PATH;
            return;
        }

        // fallback
        autoState = AutoState.DONE;
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
