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
    public static final double OUTTAKE_POWER = 0.6;   // 60%
    public static final double RAMP_ANGLE = 0.70;
    public static final int SHOT_SPACING_MS = 300;


    // Timing
    private long shootStartTimeMs = 0;
    private static final long KICK_DELAY_MS = 200;
    private int shotsFired = 0;
    private boolean shotInProgress = false;
    private boolean shootPreloadPathStarted = false;
    private boolean outtakeSpinning = false;
    private boolean pocketAligned = false;

    // ================= INTAKE CONFIG =================

    // Intake behavior during collection paths
    private static final double INTAKE_POWER = 1.0;
    public static final double INTAKE_SPEED_LIMIT = 0.5;   // path slowdown
    public static final long INTAKE_SETTLE_MS = 250;


    // ================= MOTIF =================

    private String motif = null;
    private long motifStartTimeMs = 0;
    private static final long MOTIF_TIMEOUT_MS = 600;
    private boolean motifLocked = false;
    private int motifIndex = 0;

    private enum AutoState {

        DRIVE_TO_TAG,          // Path: toAprilTag

        DRIVE_TO_SHOOT_POS,    // Path: shootPreload (global shooting spot)
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

    private boolean shootNextMotifBall(long delayMs) {

        if (motifIndex >= motif.length()) return true; // done

        char target = motif.charAt(motifIndex);
        FinalSorter.BallColor color =
                (target == 'G')
                        ? FinalSorter.BallColor.GREEN
                        : FinalSorter.BallColor.PURPLE;

        // STEP 1: find correct pocket
        int pocket = mechanisms.sorter.getPocketWithColor(color);
        if (pocket == -1) {
            // No such ball left → skip
            motifIndex++;
            return false;
        }

        // STEP 2: command move once
        if (!pocketAligned && !mechanisms.isSorterBusy()) {
            mechanisms.sorter.movePocketToOuttake(pocket);
            pocketAligned = true;
            return false;
        }

        // STEP 3: wait for move to finish, then kick
        if (pocketAligned && !shotInProgress && !mechanisms.isSorterBusy()) {
            mechanisms.ejectBall();
            shotInProgress = true;
            shootStartTimeMs = System.currentTimeMillis();
            return false;
        }

        // STEP 4: delay + manual clear
        if (shotInProgress &&
                System.currentTimeMillis() - shootStartTimeMs > delayMs) {

            mechanisms.sorter.onBallEjected();
            shotInProgress = false;
            pocketAligned = false;
            motifIndex++;
        }

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

    private void log(String caption, Object value) {
        panelsTelemetry.debug(caption, value);
        telemetry.addData(caption, value);
    }

    @Override
    public void runOpMode() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(33, 136, Math.toRadians(90)));

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

        mechanisms.setRampAngle(RAMP_ANGLE);

        aprilTagLimelight = new AprilTagLimelight(hardwareMap);
        aprilTagLimelight.enableMotifScan();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        waitForStart();

        AutoState state = AutoState.DRIVE_TO_TAG;

        motifStartTimeMs = System.currentTimeMillis();
        motif = "GPP"; // DEFAULT fallback (or whatever you choose)

        while (opModeIsActive()) {

            follower.update();
            mechanisms.updateMechanisms();

            if (state != lastState) {
                stateEntryTime = System.currentTimeMillis();
                lastState = state;
            }

            switch (state) {

                case DRIVE_TO_TAG:

//                    if (!follower.isBusy()) {
//                        follower.followPath(paths.toAprilTag);
//                    }
                    if (!driveToTagStarted) {
                        follower.followPath(paths.toAprilTag);
                        driveToTagStarted = true;
                    }

                    if (!motifLocked) {
                        String m = aprilTagLimelight.getMotif();

                        if (!m.equals("UNKNOWN")) {
                            motif = m;
                            motifLocked = true;
                            mechanisms.sorter.triggerMotifLockedFlash();

                        } else if (System.currentTimeMillis() - motifStartTimeMs > MOTIF_TIMEOUT_MS) {
                            motifLocked = true;
                        }
                    }

                    if (!follower.isBusy()) {
                        state = AutoState.DRIVE_TO_SHOOT_POS;
                        driveToTagStarted = false;
                    }
                    break;

                case DRIVE_TO_SHOOT_POS:

                    // 1) Start path ONCE
                    if (!shootPreloadPathStarted) {
                        follower.followPath(paths.shootPreload);
                        mechanisms.engageOuttake(OUTTAKE_POWER); // spin up EARLY
                        shootPreloadPathStarted = true;
                    }

                    // 2) Wait for path to finish
                    if (shootPreloadPathStarted && !follower.isBusy()) {
                        shootPreloadPathStarted = false;
                        shotsFired = 0;
                        shotInProgress = false;
                        shootStartTimeMs = System.currentTimeMillis();

                        state = AutoState.SHOOT_PRELOADS;
                    }
                    break;


                case SHOOT_PRELOADS:

                    if (shootNextMotifBall(SHOT_SPACING_MS)) {
                        // finished motif (3 balls)
                        motifIndex = 0; // reset for later sets

                        follower.followPath(paths.toFirstBalls);
                        mechanisms.disengageOuttake();
                        state = AutoState.DRIVE_TO_FIRST_SET;
                    }

                    break;

                case DRIVE_TO_FIRST_SET:
                    if (!toFirstSetStarted) {
                        follower.followPath(paths.toFirstBalls);
                        toFirstSetStarted = true;
                    }
                    if (!follower.isBusy()) {
                        toFirstSetStarted = false;
                        state = AutoState.COLLECT_FIRST_SET;
                    }
                    break;

                case COLLECT_FIRST_SET:
                    // Slow ONLY the intake sweep path
                    if (!firstSweepStarted) {
                        follower.setMaxPower(INTAKE_SPEED_LIMIT);
                        follower.followPath(paths.throughFirstBalls);

                        mechanisms.engageIntake(INTAKE_POWER, false);
                        mechanisms.sorter.setAutoMode(true);
                        firstSweepStarted = true;
                    }

                    if (!follower.isBusy()) {
                        firstSweepStarted = false;
                        follower.setMaxPower(Constants.driveConstants.maxPower);

                        // KEEP intake + sorter running during settle
                        delayStart = System.currentTimeMillis();
                        state = AutoState.INTAKE_DELAY_1;
                    }
                    break;

                case INTAKE_DELAY_1:
                    if (System.currentTimeMillis() - delayStart > INTAKE_SETTLE_MS) {
                        mechanisms.disengageIntake();
                        mechanisms.sorter.setAutoMode(false);
                        preAlignFirstMotifBall();

                        follower.followPath(paths.shootFirstBalls);
                        state = AutoState.DRIVE_TO_SHOOT_1;
                    }
                    break;

                case DRIVE_TO_SHOOT_1:
                    // Start spin-up ONCE while driving
                    if (!outtakeSpinning) {
                        mechanisms.engageOuttake(OUTTAKE_POWER);
                        outtakeSpinning = true;
                    }

                    // Wait until path finishes
                    if (!follower.isBusy()) {
                        shootStartTimeMs = System.currentTimeMillis();
                        motifIndex = 0;           // reset motif sequence
                        shotInProgress = false;  // safety
                        state = AutoState.SHOOT_SET_1;
                    }
                    break;

                case SHOOT_SET_1:
                    // Keep outtake spinning — DO NOT disengage yet
                    if (shootNextMotifBall(SHOT_SPACING_MS)) {

                        // Finished full motif (GPP etc.)
                        motifIndex = 0;
                        outtakeSpinning = false;

                        mechanisms.disengageOuttake();   // now safe to stop
                        follower.followPath(paths.toSecondBalls);
                        state = AutoState.DRIVE_TO_SECOND_SET;
                    }

                    break;

                case DRIVE_TO_SECOND_SET:
                    if (!toSecondSetStarted) {
                        follower.followPath(paths.toSecondBalls);
                        toSecondSetStarted = true;
                    }
                    if (!follower.isBusy()) {
                        toSecondSetStarted = false;
                        state = AutoState.COLLECT_SECOND_SET;
                    }
                    break;

                case COLLECT_SECOND_SET:
                    // Slow ONLY the intake sweep path
                    if (!secondSweepStarted) {
                        follower.setMaxPower(INTAKE_SPEED_LIMIT);
                        follower.followPath(paths.throughSecondBalls);

                        mechanisms.engageIntake(INTAKE_POWER, false);
                        mechanisms.sorter.setAutoMode(true);
                        secondSweepStarted = true;
                    }

                    if (!follower.isBusy()) {
                        firstSweepStarted = false;
                        follower.setMaxPower(Constants.driveConstants.maxPower);

                        // KEEP intake + sorter running during settle
                        delayStart = System.currentTimeMillis();
                        state = AutoState.INTAKE_DELAY_2;
                    }
                    break;

                case INTAKE_DELAY_2:
                    if (System.currentTimeMillis() - delayStart > INTAKE_SETTLE_MS) {
                        mechanisms.disengageIntake();
                        mechanisms.sorter.setAutoMode(false);
                        preAlignFirstMotifBall();

                        follower.followPath(paths.shootSecondBalls);
                        state = AutoState.DRIVE_TO_SHOOT_2;
                    }
                    break;

                case DRIVE_TO_SHOOT_2:
                    if (!outtakeSpinning) {
                        mechanisms.engageOuttake(OUTTAKE_POWER);
                        outtakeSpinning = true;
                    }

                    if (!follower.isBusy()) {
                        shootStartTimeMs = System.currentTimeMillis();
                        motifIndex = 0;
                        shotInProgress = false;
                        state = AutoState.SHOOT_SET_2;
                    }
                    break;

                case SHOOT_SET_2:
                    if (shootNextMotifBall(SHOT_SPACING_MS)) {
                        motifIndex = 0;
                        outtakeSpinning = false;

                        mechanisms.disengageOuttake();
                        follower.followPath(paths.toEndPosition);
                        state = AutoState.DRIVE_TO_END;
                    }

                    break;

                case DRIVE_TO_END:
                    if (!follower.isBusy()) {
                        state = AutoState.DONE;
                    }
                    break;

                case DONE:
                    mechanisms.disengageIntake();
                    mechanisms.disengageOuttake();
                    break;
            }

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

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("x", follower.getPose().getX());
            packet.put("y", follower.getPose().getY());
            packet.put("heading", Math.toDegrees(follower.getPose().getHeading()));
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
