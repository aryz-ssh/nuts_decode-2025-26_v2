package org.firstinspires.ftc.teamcode.autocode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AprilTagLimelight;
import org.firstinspires.ftc.teamcode.FinalSorter;
import org.firstinspires.ftc.teamcode.Mechanisms;
import org.firstinspires.ftc.teamcode.autocode.pathingOnly.BlueClose9BallBermudaPathOnly;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BlueClose", group = "Autonomous")
@Configurable
@Config
public class BlueClose9BallBermuda extends LinearOpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private BlueClose9BallBermudaPathOnly.RobotPaths paths;
    private Mechanisms mechanisms;
    private AprilTagLimelight aprilTagLimelight;
    private long delayStart = 0;

    // ================= SHOOTING CONFIG =================

    // Global shooter settings (used at every shooting point)
    public static double OUTTAKE_POWER = 0.6;   // 60%
    public static double RAMP_ANGLE = 0.70;
    public static int SHOT_SPACING_MS = 600;
    public static long PRE_SHOOT_DELAY_MS = 1000; // tune this (250–500)
    private long preshootDelayStart = -1;
    private long outtakeSpinupStart = -1;
    public static long OUTTAKE_SPINUP_MS = 400; // tune this
    private long sorterNotBusySince = -1;
    public static long SORTER_POST_BUSY_MS = 400; // tune 150–250

    // Timing
    private long shootStartTimeMs = 0;
    private static final long KICK_DELAY_MS = 200;
    private int shotsFired = 0;
    private boolean shotInProgress = false;
    private boolean shootPreloadPathStarted = false;
    private boolean outtakeSpinning = false;
    private boolean pocketAligned = false;
    private boolean transitionArmed = false;

    public static long SETTLE_DELAY_MS = 1000;
    // private long settleStartTime = -1;

    // ================= GLOBAL TIMING =================
    public static long POST_PATH_DELAY_MS = 5000;   // delay after EVERY path
    private long postShotDelayStart = -1;

    // ================= INTAKE CONFIG =================

    // Intake behavior during collection paths
    private static double INTAKE_POWER = 1.0;
    public static double INTAKE_SPEED_LIMIT = 0.35;   // path slowdown
    public static long INTAKE_SETTLE_MS = 1200;
    private boolean firstSweepPowerSet = false;
    private boolean secondSweepPowerSet = false;
    private boolean sorterPreAligned = false;


    // ================= MOTIF =================
    // Scan motif ONLY when stopped at tag
    public static long TAG_SCAN_WINDOW_MS = 500;
    private long tagScanStartMs = -1;
    private boolean tagScanArmed = false;
    private String motif = null;
    private long motifStartTimeMs = 0;
    private static final long MOTIF_TIMEOUT_MS = 600;
    private boolean motifLocked = false;
    private int motifIndex = 0;

    // ================= TELEMETRY / DASH THROTTLING =================
    public static boolean LOG_ENABLED = true;
    public static boolean DASH_POSE_ENABLED = false;

    public static long LOG_PERIOD_MS = 100;      // telemetry + panels
    public static long DASH_PERIOD_MS = 75;     // dashboard pose

    private long lastLogMs = 0;
    private long lastDashMs = 0;

    private enum AutoState {

        DRIVE_TO_TAG,          // Path: toAprilTag
        WAIT_AFTER_TAG,

        DRIVE_TO_SHOOT_POS,    // Path: shootPreload (global shooting spot)
        WAIT_AFTER_PRESHOT_POS,

        SHOOT_PRELOADS,      // <--- ADD THIS

        DRIVE_TO_FIRST_SET,    // Path: toFirstBalls
        COLLECT_FIRST_SET,     // Path: throughFirstBalls
        INTAKE_DELAY_1,

        DRIVE_TO_SHOOT_1,      // Path: shootFirstBalls
        SHOOT_SET_1,           // Shoot balls based on motif

        DRIVE_TO_SECOND_SET,   // Path: toSecondBalls
        COLLECT_SECOND_SET,    // Path: throughSecondBalls
        INTAKE_DELAY_2,

        DRIVE_TO_SHOOT_2,      // Path: shootSecondBalls
        SHOOT_SET_2,           // Shoot remaining balls

        DRIVE_TO_END,          // Path: toEndPosition

        DONE                   // Idle / safety
    }

    private AutoState lastState = null;
    private long stateEntryTime = 0;

    private boolean driveToTagStarted = false;

    private boolean toFirstSetStarted = false;
    private boolean toSecondSetStarted = false;

    private boolean firstSweepStarted = false;
    private boolean secondSweepStarted = false;

    private boolean driveToShoot1Started = false;
    private boolean driveToShoot2Started = false;
    private boolean driveToEndStarted = false;

    private boolean shootNextMotifBall(long delayMs) {

        if (motifIndex >= motif.length()) {

            // wait AFTER last shot
            if (postShotDelayStart < 0) {
                postShotDelayStart = System.currentTimeMillis();
                return false;
            }

            if (System.currentTimeMillis() - postShotDelayStart >= delayMs) {
                postShotDelayStart = -1;
                return true;
            }

            return false;
        }

        char target = motif.charAt(motifIndex);
        FinalSorter.BallColor color =
                (target == 'G')
                        ? FinalSorter.BallColor.GREEN
                        : FinalSorter.BallColor.PURPLE;

        // STEP 1: find correct pocket
        int pocket = mechanisms.sorter.getPocketWithColor(color);

        // FALLBACK: requested color not available
        if (pocket == -1) {
            pocket = mechanisms.sorter.getPocketWithAnyBall();
            if (pocket == -1) {
                // No balls left at all → motif step is effectively done
                motifIndex++;
                return false;
            }
        }

        // STEP 2: command move once
        if (!pocketAligned && !mechanisms.isSorterBusy()) {
            sorterNotBusySince = -1; // reset settle timer
            mechanisms.sorter.movePocketToOuttake(pocket);
            pocketAligned = true;
            return false;
        }

        // STEP 3: wait for move to finish, then kick
// Track when sorter becomes NOT busy
        if (pocketAligned && !mechanisms.isSorterBusy()) {
            if (sorterNotBusySince < 0) {
                sorterNotBusySince = System.currentTimeMillis();
                return false;
            }
        } else {
            sorterNotBusySince = -1; // reset if sorter moves again
        }

// Fire ONLY after sorter has been stable for 200ms
        if (pocketAligned &&
                !shotInProgress &&
                sorterNotBusySince > 0 &&
                System.currentTimeMillis() - sorterNotBusySince >= SORTER_POST_BUSY_MS) {

            mechanisms.ejectBall();
            shotInProgress = true;
            shootStartTimeMs = System.currentTimeMillis();
            sorterNotBusySince = -1;
            return false;
        }

        // STEP 4: delay + manual clear
        if (shotInProgress &&
                System.currentTimeMillis() - shootStartTimeMs > delayMs) {

            mechanisms.sorter.onBallEjected(); // clear PREVIOUS shot
            shotInProgress = false;
            pocketAligned = false;
            sorterNotBusySince = -1;
            motifIndex++;
        }

        // FAILSAFE: force eject if aligned too long
//        if (pocketAligned && !shotInProgress &&
//                System.currentTimeMillis() - shootStartTimeMs > 500) {
//
//            mechanisms.ejectBall();
//            shotInProgress = true;
//            shootStartTimeMs = System.currentTimeMillis();
//        }

        return false;
    }

    private void preAlignFirstMotifBall() {

        if (motifIndex >= motif.length()) return;

        char target = motif.charAt(motifIndex);
        FinalSorter.BallColor color =
                (target == 'G')
                        ? FinalSorter.BallColor.GREEN
                        : FinalSorter.BallColor.PURPLE;

        int pocket = mechanisms.sorter.getPocketWithColor(color);
        if (pocket == -1) return;

        if (!mechanisms.isSorterBusy()) {
            mechanisms.sorter.movePocketToOuttake(pocket);
        }
    }

    private void preAlignSorterAtInit() {
        // First motif character determines preload
        char target = motif.charAt(0);

        FinalSorter.BallColor color =
                (target == 'G')
                        ? FinalSorter.BallColor.GREEN
                        : FinalSorter.BallColor.PURPLE;

        int pocket = mechanisms.sorter.getPocketWithColor(color);

        if (pocket != -1) {
            mechanisms.sorter.movePocketToOuttake(pocket);
        }
    }

    private void log(String caption, Object value) {
        panelsTelemetry.debug(caption, value);
        telemetry.addData(caption, value);
    }

    @Override
    public void runOpMode() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(111, 136, Math.toRadians(90)));

        paths = new BlueClose9BallBermudaPathOnly.RobotPaths(follower);

        mechanisms = new Mechanisms();
        mechanisms.initMechanisms(hardwareMap, telemetry, true);

        // Declare known preload layout:
        // pocket 0 = GREEN
        // pocket 1 = PURPLE
        // pocket 2 = PURPLE
        mechanisms.sorter.forceSetSlotColors(new FinalSorter.BallColor[] {
                FinalSorter.BallColor.GREEN,
                FinalSorter.BallColor.PURPLE,
                FinalSorter.BallColor.PURPLE
        });

        FinalSorter.DASH_ENABLED = false;

        mechanisms.setRampAngle(RAMP_ANGLE);

        aprilTagLimelight = new AprilTagLimelight(hardwareMap);
        aprilTagLimelight.enableMotifScan();

        motif = "GPP"; // DEFAULT fallback (or whatever you choose)
        preAlignSorterAtInit();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        waitForStart();

        AutoState state = AutoState.DRIVE_TO_TAG;

        motifStartTimeMs = System.currentTimeMillis();

        while (opModeIsActive()) {

            follower.update();
            mechanisms.updateMechanisms();

            if (state != lastState) {
                stateEntryTime = System.currentTimeMillis();
                lastState = state;
            }

            switch (state) {

                /* ===================== DRIVE TO TAG ===================== */

                case DRIVE_TO_TAG:

                    if (!sorterPreAligned) {
                        preAlignSorterAtInit();
                        sorterPreAligned = true;
                    }

                    // Start the path once
                    if (!driveToTagStarted) {
                        follower.followPath(paths.toAprilTag);
                        driveToTagStarted = true;

                        // Reset scan state for this run
                        tagScanStartMs = -1;
                        tagScanArmed = false;
                        motifLocked = false;              // optional: if you want fresh scan each run
                        motif = "GPP";                    // optional default/fallback
                    }

                    // While Pedro is moving: NEVER touch Limelight
                    if (follower.isBusy()) {
                        break;
                    }

                    // We are stopped at tag. Arm a 500ms scan window.
                    if (!tagScanArmed) {
                        tagScanArmed = true;
                        tagScanStartMs = System.currentTimeMillis();
                    }

                    // Scan for up to 500ms total
                    if (!motifLocked) {
                        long elapsed = System.currentTimeMillis() - tagScanStartMs;

                        if (elapsed <= TAG_SCAN_WINDOW_MS) {
                            String m = aprilTagLimelight.getMotif(); // only called while stopped
                            if (!m.equals("UNKNOWN")) {
                                motif = m;
                                motifLocked = true;
                                mechanisms.sorter.triggerMotifLockedFlash();
                            }
                        } else {
                            // Window expired -> lock whatever we have (fallback stays)
                            motifLocked = true;
                        }
                    }

                    // After scan window (or early lock), move on
                    if (motifLocked) {
                        driveToTagStarted = false;
                        state = AutoState.DRIVE_TO_SHOOT_POS;
                    }

                    break;

//                case WAIT_AFTER_TAG:
//
//                    // Safety: outtake MUST be off
//                    mechanisms.disengageOuttake();
//
//                    if (follower.isBusy()) {
//                        settleStartTime = -1; // reset if Pedro corrects
//                        break;
//                    }
//
//                    if (settleStartTime < 0) {
//                        settleStartTime = System.currentTimeMillis();
//                        break;
//                    }
//
//                    if (System.currentTimeMillis() - settleStartTime >= SETTLE_DELAY_MS) {
//                        settleStartTime = -1;
//                        state = AutoState.DRIVE_TO_SHOOT_POS;
//                    }
//                    break;


                /* ===================== DRIVE TO PRELOAD SHOOT ===================== */

                case DRIVE_TO_SHOOT_POS:
                    if (!shootPreloadPathStarted) {
                        follower.followPath(paths.shootPreload);
                        shootPreloadPathStarted = true;
                    }

                    if (!follower.isBusy()) {
                        shootPreloadPathStarted = false;
                        state = AutoState.SHOOT_PRELOADS;
                    }
                    break;

//                case WAIT_AFTER_PRESHOT_POS:
//
//                    // Still NO outtake here
//                    mechanisms.disengageOuttake();
//
//                    if (follower.isBusy()) {
//                        settleStartTime = -1;
//                        break;
//                    }
//
//                    if (settleStartTime < 0) {
//                        settleStartTime = System.currentTimeMillis();
//                        break;
//                    }
//
//                    if (System.currentTimeMillis() - settleStartTime >= SETTLE_DELAY_MS) {
//
//                        // NOW it is safe to spin shooter
//                        mechanisms.engageOuttake(OUTTAKE_POWER);
//
//                        motifIndex = 0;
//                        shotInProgress = false;
//
//                        settleStartTime = -1;
//                        state = AutoState.SHOOT_PRELOADS;
//                    }
//                    break;


                /* ===================== SHOOT PRELOADS ===================== */

                case SHOOT_PRELOADS:
                    // --- Spin-up gate ---
                    if (!outtakeSpinning) {
                        if (outtakeSpinupStart < 0) {
                            mechanisms.engageOuttake(OUTTAKE_POWER);
                            outtakeSpinupStart = System.currentTimeMillis();
                            break;
                        }

                        if (System.currentTimeMillis() - outtakeSpinupStart < OUTTAKE_SPINUP_MS) {
                            break; // wait for RPM
                        }

                        outtakeSpinning = true;
                        outtakeSpinupStart = -1;
                    }

                    if (shootNextMotifBall(SHOT_SPACING_MS)) {
                        mechanisms.disengageOuttake();
                        outtakeSpinning = false;
                        outtakeSpinupStart = -1;

                        motifIndex = 0;
                        state = AutoState.DRIVE_TO_FIRST_SET;
                    }
                    break;

                /* ===================== DRIVE TO FIRST SET ===================== */

                case DRIVE_TO_FIRST_SET:
                    if (!toFirstSetStarted) {
                        mechanisms.sorter.movePocketToIntake(0); // force intake pocket
                        follower.followPath(paths.toFirstBalls);
                        toFirstSetStarted = true;
                    }

                    if (!follower.isBusy()) {
                        toFirstSetStarted = false;
                        state = AutoState.COLLECT_FIRST_SET;
                    }
                    break;

                /* ===================== COLLECT FIRST SET ===================== */

                case COLLECT_FIRST_SET:
                    if (!firstSweepStarted) {
                        follower.followPath(paths.throughFirstBalls);
                        mechanisms.engageIntake(INTAKE_POWER, false);
                        mechanisms.sorter.setAutoMode(true);
                        firstSweepStarted = true;
                        firstSweepPowerSet = false;
                    }

                    // Apply power change ONE LOOP AFTER start
                    if (firstSweepStarted && !firstSweepPowerSet) {
                        follower.setMaxPower(INTAKE_SPEED_LIMIT);
                        firstSweepPowerSet = true;
                    }

                    if (!follower.isBusy()) {
                        follower.setMaxPower(Constants.driveConstants.maxPower);
                        firstSweepStarted = false;
                        delayStart = System.currentTimeMillis();
                        state = AutoState.INTAKE_DELAY_1;
                    }
                    break;

                case INTAKE_DELAY_1:
                    // keep intake + sorter running
                    if (System.currentTimeMillis() - delayStart < INTAKE_SETTLE_MS) {
                        break;
                    }

                    mechanisms.disengageIntake();
                    mechanisms.sorter.setAutoMode(false);
                    state = AutoState.DRIVE_TO_SHOOT_1;
                    break;

                /* ===================== DRIVE TO SHOOT 1 ===================== */

                case DRIVE_TO_SHOOT_1:
                    if (!driveToShoot1Started) {
                        follower.followPath(paths.shootFirstBalls);
                        driveToShoot1Started = true;
                    }

                    if (!follower.isBusy()) {
                        driveToShoot1Started = false;
                        motifIndex = 0;
                        shotInProgress = false;
                        state = AutoState.SHOOT_SET_1;
                    }
                    break;

                /* ===================== SHOOT SET 1 ===================== */

                case SHOOT_SET_1:
                    // --- Spin-up gate ---
                    if (!outtakeSpinning) {
                        if (outtakeSpinupStart < 0) {
                            mechanisms.engageOuttake(OUTTAKE_POWER);
                            outtakeSpinupStart = System.currentTimeMillis();
                            break;
                        }

                        if (System.currentTimeMillis() - outtakeSpinupStart < OUTTAKE_SPINUP_MS) {
                            break; // wait for RPM
                        }

                        outtakeSpinning = true;
                        outtakeSpinupStart = -1;
                    }

                    if (shootNextMotifBall(SHOT_SPACING_MS)) {
                        mechanisms.disengageOuttake();
                        outtakeSpinning = false;
                        outtakeSpinupStart = -1;

                        motifIndex = 0;
                        state = AutoState.DRIVE_TO_SECOND_SET;
                    }
                    break;

                /* ===================== DRIVE TO SECOND SET ===================== */

                case DRIVE_TO_SECOND_SET:
                    if (!toSecondSetStarted) {
                        mechanisms.sorter.movePocketToIntake(0); // force intake pocket
                        follower.followPath(paths.toSecondBalls);
                        toSecondSetStarted = true;
                    }

                    if (!follower.isBusy()) {
                        toSecondSetStarted = false;
                        state = AutoState.COLLECT_SECOND_SET;
                    }
                    break;

                /* ===================== COLLECT SECOND SET ===================== */

                case COLLECT_SECOND_SET:
                    if (!secondSweepStarted) {
                        follower.followPath(paths.throughSecondBalls);
                        mechanisms.engageIntake(INTAKE_POWER, false);
                        mechanisms.sorter.setAutoMode(true);
                        secondSweepStarted = true;
                        secondSweepPowerSet = false;
                    }

                    // Apply power change ONE LOOP AFTER start
                    if (secondSweepStarted && !secondSweepPowerSet) {
                        follower.setMaxPower(INTAKE_SPEED_LIMIT);
                        secondSweepPowerSet = true;
                    }

                    if (!follower.isBusy()) {
                        follower.setMaxPower(Constants.driveConstants.maxPower);
                        mechanisms.sorter.setAutoMode(false);
                        secondSweepStarted = false;
                        delayStart = System.currentTimeMillis();
                        state = AutoState.INTAKE_DELAY_2;
                    }
                    break;

                case INTAKE_DELAY_2:
                    // keep intake + sorter running
                    if (System.currentTimeMillis() - delayStart < INTAKE_SETTLE_MS) {
                        break;
                    }

                    mechanisms.disengageIntake();
                    mechanisms.sorter.setAutoMode(false);
                    state = AutoState.DRIVE_TO_SHOOT_2;
                    break;

                /* ===================== DRIVE TO SHOOT 2 ===================== */

                case DRIVE_TO_SHOOT_2:
                    if (!driveToShoot2Started) {
                        follower.followPath(paths.shootSecondBalls);
                        driveToShoot2Started = true;
                    }

                    if (!follower.isBusy()) {
                        driveToShoot2Started = false;
                        motifIndex = 0;
                        shotInProgress = false;
                        state = AutoState.SHOOT_SET_2;
                    }
                    break;

                /* ===================== SHOOT SET 2 ===================== */

                case SHOOT_SET_2:
                    // --- Spin-up gate ---
                    if (!outtakeSpinning) {
                        if (outtakeSpinupStart < 0) {
                            mechanisms.engageOuttake(OUTTAKE_POWER);
                            outtakeSpinupStart = System.currentTimeMillis();
                            break;
                        }

                        if (System.currentTimeMillis() - outtakeSpinupStart < OUTTAKE_SPINUP_MS) {
                            break; // wait for RPM
                        }

                        outtakeSpinning = true;
                        outtakeSpinupStart = -1;
                    }

                    if (shootNextMotifBall(SHOT_SPACING_MS)) {
                        mechanisms.disengageOuttake();
                        outtakeSpinning = false;
                        outtakeSpinupStart = -1;
                        motifIndex = 0;
                        state = AutoState.DRIVE_TO_END;
                    }
                    break;

                /* ===================== DRIVE TO END ===================== */

                case DRIVE_TO_END:
                    if (!driveToEndStarted) {
                        follower.followPath(paths.toEndPosition);
                        driveToEndStarted = true;
                    }

                    if (!follower.isBusy()) {
                        driveToEndStarted = false;
                        state = AutoState.DONE;
                    }
                    break;

                /* ===================== DONE ===================== */

                case DONE:
                    mechanisms.disengageIntake();
                    mechanisms.disengageOuttake();
                    break;
            }

            long now = System.currentTimeMillis();
            boolean doLog = LOG_ENABLED && (now - lastLogMs >= LOG_PERIOD_MS);

            if (doLog) {
                telemetry.clearAll();
                log("State", state);
                log("State Time (ms)", System.currentTimeMillis() - stateEntryTime);
                log("X", follower.getPose().getX());
                log("Y", follower.getPose().getY());
                log("Follower Busy", follower.isBusy());
                log("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
                log("Motif", motif);
                log("Motif Index", motifIndex);
                log("Pocket Aligned", pocketAligned);
                log("Shot In Progress", shotInProgress);
                log("Sorter Busy", mechanisms.isSorterBusy());
                telemetry.update();
                panelsTelemetry.update(telemetry);
                lastLogMs = now;
            }


            now = System.currentTimeMillis();
            if (DASH_POSE_ENABLED && now - lastDashMs >= DASH_PERIOD_MS) {
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("x", follower.getPose().getX());
                packet.put("y", follower.getPose().getY());
                packet.put("heading", Math.toDegrees(follower.getPose().getHeading()));
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
                lastDashMs = now;
            }
        }
    }
}
