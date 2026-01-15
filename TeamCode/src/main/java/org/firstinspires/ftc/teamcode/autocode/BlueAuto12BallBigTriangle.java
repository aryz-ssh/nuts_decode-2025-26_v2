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
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.AprilTagLimelight;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Big Triangle Auto - 12 Ball", group = "Autonomous")
@Configurable
public class BlueAuto12BallBigTriangle extends LinearOpMode {

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
    private AprilTagLimelight limelight;
    private Follower follower;

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


    private BlueAuto12BallBigTriangle.DriveStage driveStage = BlueAuto12BallBigTriangle.DriveStage.SCAN_MOTIF;
    public static double RAMP_UP_TIME = 0.1;        // seconds
    public static double SORTER_SETTLE_TIME = 0.35;  // seconds
    public static double FIRST_KICK_DELAY = 0.50;    // seconds

    BlueAuto12BallBigTriangle.AutoState autoState = BlueAuto12BallBigTriangle.AutoState.START_PATH;

    int currentPocket = 1;
    int activeShotPocket = 1;
    boolean sorterCommanded = false;
    boolean intakeRunning = false;

    ElapsedTime stateTimer = new ElapsedTime();

    public static long INTAKE_SETTLE_MS = 750; // dashboard-tunable
    public static double INTAKE_PATH_SPEED = 0.75;
    public static int MOTIF_PIPELINE = 2;
    public static int BLUE_PIPELINE = 0;
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
        follower.setStartingPose(new Pose(32.194, 135.776, Math.toRadians(90)));
        buildPaths();

        telemetry.addLine("Initialized (Mechanisms + Follower)");
        telemetry.addData("Busy", follower.isBusy());
        telemetry.addData("DriveStage", driveStage);
        telemetry.addData("AutoState", autoState);
        telemetry.addData("Pose", follower.getPose());

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
        follower.setPose(new Pose(32.194, 135.776, Math.toRadians(90)));
        if (!opModeIsActive()) return;

       // scanMotifWithLimelight();

        // safe defaults at start
        mechanisms.disengageOuttake();
        shooterSpinning = false;
        mechanisms.disengageIntake();
        mechanisms.setRampAngle(Mechanisms.RAMP_ANGLE_MIN_POS);

        autoState = BlueAuto12BallBigTriangle.AutoState.START_PATH;
        driveStage = BlueAuto12BallBigTriangle.DriveStage.SCAN_MOTIF;     // see section #2 below
        currentPocket = 1;
        sorterCommanded = false;

        while (opModeIsActive()) {

            if (follower.isBusy()) {
                follower.update();
            }
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

                    autoState = BlueAuto12BallBigTriangle.AutoState.WAIT_PATH;
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
                            autoState = BlueAuto12BallBigTriangle.AutoState.INTAKE_SETTLE;
                        } else if (isShootPathJustFinished()) {
                            autoState = BlueAuto12BallBigTriangle.AutoState.START_SHOOT;
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

                    autoState = BlueAuto12BallBigTriangle.AutoState.START_PATH; // will run the intake path next
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
                    autoState = BlueAuto12BallBigTriangle.AutoState.WAIT_RAMP;
                    break;
                }


                case WAIT_RAMP: {
                    if (stateTimer.seconds() >= RAMP_UP_TIME) {
                        currentPocket = 1;
                        sorterCommanded = false;
                        autoState = BlueAuto12BallBigTriangle.AutoState.MOVE_SORTER;
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
                    autoState = BlueAuto12BallBigTriangle.AutoState.WAIT_SORTER;
                    break;
                }

                case WAIT_SORTER: {
                    if (!mechanisms.isSorterMoving()) {
                        stateTimer.reset();
                        autoState = BlueAuto12BallBigTriangle.AutoState.SORTER_SETTLE;
                    }
                    break;
                }

                case SORTER_SETTLE: {
                    if (stateTimer.seconds() >= SORTER_SETTLE_TIME) {
                        autoState = BlueAuto12BallBigTriangle.AutoState.KICK_1;
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
                    autoState = BlueAuto12BallBigTriangle.AutoState.WAIT_1;
                    break;
                }

                case WAIT_1: {
                    if (stateTimer.seconds() >= FIRST_KICK_DELAY) {
                        autoState = BlueAuto12BallBigTriangle.AutoState.NEXT_POCKET;
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
                        autoState = BlueAuto12BallBigTriangle.AutoState.MOVE_SORTER;
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
//
//       while (opModeIsActive()
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
//       // mechanisms.limelight.pipelineSwitch(BLUE_PIPELINE);
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

    private boolean isShootPath(BlueAuto12BallBigTriangle.DriveStage stage) {
        return stage == BlueAuto12BallBigTriangle.DriveStage.TO_SHOOT_PRELOAD
                || stage == BlueAuto12BallBigTriangle.DriveStage.SHOOT_1
                || stage == BlueAuto12BallBigTriangle.DriveStage.SHOOT_2
                || stage == BlueAuto12BallBigTriangle.DriveStage.SHOOT_3;
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
        return driveStage == BlueAuto12BallBigTriangle.DriveStage.INTAKE_1
                || driveStage == BlueAuto12BallBigTriangle.DriveStage.INTAKE_2
                || driveStage == BlueAuto12BallBigTriangle.DriveStage.INTAKE_3;
    }

    private boolean isShootPathJustFinished() {
        return driveStage == BlueAuto12BallBigTriangle.DriveStage.TO_SHOOT_PRELOAD
                || driveStage == BlueAuto12BallBigTriangle.DriveStage.SHOOT_1
                || driveStage == BlueAuto12BallBigTriangle.DriveStage.SHOOT_2
                || driveStage == BlueAuto12BallBigTriangle.DriveStage.SHOOT_3;
    }

    private void advanceAfterNonIntakeNonShootPath() {
        switch (driveStage) {
            case SCAN_MOTIF:
                // you can scan here, then drive to preload shoot
                // (scanMotifWithLimelight can be called before running SCAN_MOTIF if you want)
                driveStage = BlueAuto12BallBigTriangle.DriveStage.TO_SHOOT_PRELOAD;
                autoState = BlueAuto12BallBigTriangle.AutoState.START_PATH;
                break;

            case TO_SHOOT_PRELOAD:
                autoState = BlueAuto12BallBigTriangle.AutoState.START_SHOOT;
                break;

            case HIT_GATE:
                driveStage = BlueAuto12BallBigTriangle.DriveStage.SHOOT_1;
                autoState = BlueAuto12BallBigTriangle.AutoState.START_PATH;
                break;

            case END:
                driveStage = BlueAuto12BallBigTriangle.DriveStage.DONE;
                autoState = BlueAuto12BallBigTriangle.AutoState.DONE;
                break;

            default:
                // should not happen
                autoState = BlueAuto12BallBigTriangle.AutoState.DONE;
                break;
        }
    }

    private void advanceAfterIntakeSettled() {
        switch (driveStage) {
            case INTAKE_1:
                // go to hit gate after intake1
                driveStage = BlueAuto12BallBigTriangle.DriveStage.HIT_GATE;
                autoState = BlueAuto12BallBigTriangle.AutoState.START_PATH;
                break;

            case INTAKE_2:
                driveStage = BlueAuto12BallBigTriangle.DriveStage.SHOOT_2; // shoot2 path returns to shoot pos
                autoState = BlueAuto12BallBigTriangle.AutoState.START_PATH;
                break;

            case INTAKE_3:
                driveStage = BlueAuto12BallBigTriangle.DriveStage.SHOOT_3;
                autoState = BlueAuto12BallBigTriangle.AutoState.START_PATH;
                break;

            default:
                autoState = BlueAuto12BallBigTriangle.AutoState.DONE;
                break;
        }
    }

    private void advanceAfterShotSet() {
        // decides what we do after finishing 3 shots at shoot position
        if (driveStage == BlueAuto12BallBigTriangle.DriveStage.TO_SHOOT_PRELOAD) {
            // after preload shots, start first intake cycle
            driveStage = BlueAuto12BallBigTriangle.DriveStage.INTAKE_1;
            autoState = BlueAuto12BallBigTriangle.AutoState.START_INTAKE;
            return;
        }

        if (driveStage == BlueAuto12BallBigTriangle.DriveStage.SHOOT_1) {
            driveStage = BlueAuto12BallBigTriangle.DriveStage.INTAKE_2;
            autoState = BlueAuto12BallBigTriangle.AutoState.START_INTAKE;
            return;
        }

        if (driveStage == BlueAuto12BallBigTriangle.DriveStage.SHOOT_2) {
            driveStage = BlueAuto12BallBigTriangle.DriveStage.INTAKE_3;
            autoState = BlueAuto12BallBigTriangle.AutoState.START_INTAKE;
            return;
        }

        if (driveStage == BlueAuto12BallBigTriangle.DriveStage.SHOOT_3) {
            // park
            mechanisms.sorterGoToIntake(1);
            mechanisms.setRampAngle(Mechanisms.RAMP_ANGLE_MIN_POS);
            driveStage = BlueAuto12BallBigTriangle.DriveStage.END;
            autoState = BlueAuto12BallBigTriangle.AutoState.START_PATH;
            return;
        }

        // fallback
        autoState = BlueAuto12BallBigTriangle.AutoState.DONE;
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

        // ================= SCAN MOTIF =================
        scanMotif = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(32.194, 135.776, Math.toRadians(90)),   // START = robot start
                        new Pose(56.559, 113.380, Math.toRadians(60))    // -300° → 60°
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(90),
                        Math.toRadians(60)
                )
                .build();

        // ================= PRELOAD SHOOT =================
        first3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(56.559, 113.380, Math.toRadians(60)),
                        new Pose(44.000, 105.000, Math.toRadians(-135)) // -225°
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(60),
                        Math.toRadians(-135)
                )
                .build();

        // ================= INTAKE 1 =================
        intake1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(44.000, 105.000, Math.toRadians(-135)),
                        new Pose(61.937, 79.043),
                        new Pose(15.000, 84.000, Math.toRadians(-180))
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(-135),
                        Math.toRadians(-180)
                )
                .build();

        // ================= HIT GATE =================
        hitGate = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(15.000, 84.000, Math.toRadians(-180)),
                        new Pose(24.000, 72.000),
                        new Pose(15.000, 72.000, Math.toRadians(90))     // -270° → 90°
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(-180),
                        Math.toRadians(90)
                )
                .build();

        // ================= SHOOT 1 =================
        shoot1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(15.000, 72.000, Math.toRadians(90)),
                        new Pose(48.000, 96.000),
                        new Pose(44.000, 105.000, Math.toRadians(-135))
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(90),
                        Math.toRadians(-135)
                )
                .build();

        // ================= INTAKE 2 =================
        intake2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(44.000, 105.000, Math.toRadians(-135)),
                        new Pose(75.332, 51.219),
                        new Pose(15.000, 60.000, Math.toRadians(-180))
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(-135),
                        Math.toRadians(-180)
                )
                .build();

        // ================= SHOOT 2 =================
        shoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(15.000, 60.000, Math.toRadians(-180)),
                        new Pose(48.000, 72.000),
                        new Pose(44.000, 105.000, Math.toRadians(-135))
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(-180),
                        Math.toRadians(-135)
                )
                .build();

        // ================= INTAKE 3 =================
        intake3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(44.000, 105.000, Math.toRadians(-135)),
                        new Pose(89.940, 31.176),
                        new Pose(15.000, 35.000, Math.toRadians(-180))
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(-135),
                        Math.toRadians(-180)
                )
                .build();

        // ================= SHOOT 3 =================
        shoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(15.000, 35.000, Math.toRadians(-180)),
                        new Pose(28.500, 67.000),
                        new Pose(44.000, 105.000, Math.toRadians(-135))
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(-180),
                        Math.toRadians(-135)
                )
                .build();

        // ================= PARK =================
        end = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(44.000, 105.000, Math.toRadians(-135)),
                        new Pose(44.000, 135.776, Math.toRadians(-90))
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(-135),
                        Math.toRadians(-90)
                )
                .build();
    }

}
