/*
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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BlueFar9", group = "Autonomous")
@Configurable
@Config
public class BlueFar9GPT extends LinearOpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private RobotPaths paths;
    private Mechanisms mechanisms;
    private AprilTagLimelight aprilTagLimelight;

    */
/* ================= SHOOT CONFIG ================= *//*

    public static double OUTTAKE_POWER = 0.6;
    public static double RAMP_ANGLE = 0.70;
    public static int SHOT_SPACING_MS = 600;
    public static long OUTTAKE_SPINUP_MS = 400;

    */
/* ================= INTAKE CONFIG ================= *//*

    public static double INTAKE_POWER = 1.0;
    public static double INTAKE_SPEED_LIMIT = 0.35;
    public static long INTAKE_SETTLE_MS = 1200;

    */
/* ================= MOTIF ================= *//*

    public static long TAG_SCAN_WINDOW_MS = 500;
    private String motif = "GPP";
    private int motifIndex = 0;

    private boolean outtakeSpinning = false;
    private long outtakeSpinupStart = -1;
    private boolean pocketAligned = false;
    private boolean shotInProgress = false;
    private long shotStartMs = 0;
    private long sorterNotBusySince = -1;

    */
/* ================= STATE ================= *//*

    private enum AutoState {
        DRIVE_TO_TAG,
        DRIVE_TO_SHOOT_PRELOAD,
        SHOOT_PRELOAD,

        DRIVE_TO_FIRST_SET,
        COLLECT_FIRST_SET,
        INTAKE_DELAY_1,
        DRIVE_TO_SHOOT_1,
        SHOOT_SET_1,

        DRIVE_TO_SECOND_SET,
        COLLECT_SECOND_SET,
        INTAKE_DELAY_2,
        DRIVE_TO_SHOOT_2,
        SHOOT_SET_2,

        DRIVE_TO_END,
        DONE
    }

    private AutoState state = AutoState.DRIVE_TO_TAG;
    private long delayStart = 0;

    */
/* ================= SHOOT LOGIC (UNCHANGED) ================= *//*


    private boolean shootNextMotifBal
    (long delayMs) {

        if (motifIndex >= motif.length()) return true;

        char target = motif.charAt(motifIndex);
        FinalSorter.BallColor color =
                (target == 'G') ? FinalSorter.BallColor.GREEN : FinalSorter.BallColor.PURPLE;

        int pocket = mechanisms.sorter.getPocketWithColor(color);
        if (pocket == -1) pocket = mechanisms.sorter.getPocketWithAnyBall();
        if (pocket == -1) {
            motifIndex++;
            return false;
        }

        if (!pocketAligned && !mechanisms.isSorterBusy()) {
            mechanisms.sorter.movePocketToOuttake(pocket);
            pocketAligned = true;
            sorterNotBusySince = -1;
            return false;
        }

        if (pocketAligned && !mechanisms.isSorterBusy()) {
            if (sorterNotBusySince < 0) {
                sorterNotBusySince = System.currentTimeMillis();
                return false;
            }
        }

        if (!shotInProgress &&
                sorterNotBusySince > 0 &&
                System.currentTimeMillis() - sorterNotBusySince > 250) {

            mechanisms.ejectBall();
            shotStartMs = System.currentTimeMillis();
            shotInProgress = true;
            return false;
        }

        if (shotInProgress &&
                System.currentTimeMillis() - shotStartMs > delayMs) {

            mechanisms.sorter.onBallEjected();
            motifIndex++;
            pocketAligned = false;
            shotInProgress = false;
        }

        return false;
    }

    */
/* ================= RUN ================= *//*


    @Override
    public void runOpMode() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(90))); // BLUE FAR

        paths = new RobotPaths(follower);

        mechanisms = new Mechanisms();
        mechanisms.initMechanisms(hardwareMap, telemetry, true);
        mechanisms.setRampAngle(RAMP_ANGLE);

        mechanisms.sorter.forceSetSlotColors(new FinalSorter.BallColor[]{
                FinalSorter.BallColor.GREEN,
                FinalSorter.BallColor.PURPLE,
                FinalSorter.BallColor.PURPLE
        });

        aprilTagLimelight = new AprilTagLimelight(hardwareMap);
        aprilTagLimelight.enableMotifScan();

        waitForStart();

        while (opModeIsActive()) {

            follower.update();
            mechanisms.updateMechanisms();

            switch (state) {

                case DRIVE_TO_TAG:
                    follower.followPath(paths.limelight);
                    if (!follower.isBusy()) {
                        motif = aprilTagLimelight.getMotif();
                        state = AutoState.DRIVE_TO_SHOOT_PRELOAD;
                    }
                    break;

                case DRIVE_TO_SHOOT_PRELOAD:
                    follower.followPath(paths.shootPreLoad);
                    if (!follower.isBusy()) {
                        motifIndex = 0;
                        state = AutoState.SHOOT_PRELOAD;
                    }
                    break;

                case SHOOT_PRELOAD:
                    if (!outtakeSpinning) {
                        mechanisms.engageOuttake(OUTTAKE_POWER);
                        outtakeSpinupStart = System.currentTimeMillis();
                        outtakeSpinning = true;
                        break;
                    }
                    if (shootNextMotifBall(SHOT_SPACING_MS)) {
                        mechanisms.disengageOuttake();
                        motifIndex = 0;
                        state = AutoState.DRIVE_TO_FIRST_SET;
                    }
                    break;

                case DRIVE_TO_FIRST_SET:
                    follower.followPath(paths.toFirstBalls);
                    mechanisms.engageIntake(INTAKE_POWER, false);
                    state = AutoState.COLLECT_FIRST_SET;
                    break;

                case COLLECT_FIRST_SET:
                    follower.followPath(paths.throughFirstBalls);
                    if (!follower.isBusy()) {
                        delayStart = System.currentTimeMillis();
                        state = AutoState.INTAKE_DELAY_1;
                    }
                    break;

                case INTAKE_DELAY_1:
                    if (System.currentTimeMillis() - delayStart > INTAKE_SETTLE_MS) {
                        mechanisms.disengageIntake();
                        state = AutoState.DRIVE_TO_SHOOT_1;
                    }
                    break;

                case DRIVE_TO_SHOOT_1:
                    follower.followPath(paths.shootFirstBalls);
                    if (!follower.isBusy()) {
                        motifIndex = 0;
                        state = AutoState.SHOOT_SET_1;
                    }
                    break;

                case SHOOT_SET_1:
                    if (shootNextMotifBall(SHOT_SPACING_MS)) {
                        mechanisms.disengageOuttake();
                        motifIndex = 0;
                        state = AutoState.DRIVE_TO_SECOND_SET;
                    }
                    break;

                case DRIVE_TO_SECOND_SET:
                    follower.followPath(paths.toSecondBalls);
                    mechanisms.engageIntake(INTAKE_POWER, false);
                    state = AutoState.COLLECT_SECOND_SET;
                    break;

                case COLLECT_SECOND_SET:
                    follower.followPath(paths.throughSecondBalls);
                    if (!follower.isBusy()) {
                        delayStart = System.currentTimeMillis();
                        state = AutoState.INTAKE_DELAY_2;
                    }
                    break;

                case INTAKE_DELAY_2:
                    if (System.currentTimeMillis() - delayStart > INTAKE_SETTLE_MS) {
                        mechanisms.disengageIntake();
                        state = AutoState.DRIVE_TO_SHOOT_2;
                    }
                    break;

                case DRIVE_TO_SHOOT_2:
                    follower.followPath(paths.shootSecondBalls);
                    if (!follower.isBusy()) {
                        motifIndex = 0;
                        state = AutoState.SHOOT_SET_2;
                    }
                    break;

                case SHOOT_SET_2:
                    if (shootNextMotifBall(SHOT_SPACING_MS)) {
                        mechanisms.disengageOuttake();
                        state = AutoState.DRIVE_TO_END;
                    }
                    break;

                case DRIVE_TO_END:
                    follower.followPath(paths.Path9);
                    if (!follower.isBusy()) state = AutoState.DONE;
                    break;

                case DONE:
                    mechanisms.disengageIntake();
                    mechanisms.disengageOuttake();
                    break;
            }
        }
    }

    */
/* ================= PATHS (VISUALIZER) ================= *//*


    public static class RobotPaths {

        public PathChain limelight, shootPreLoad;
        public PathChain toFirstBalls, throughFirstBalls, shootFirstBalls;
        public PathChain toSecondBalls, throughSecondBalls, shootSecondBalls;
        public PathChain Path9;

        public RobotPaths(Follower follower) {

            limelight = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(56, 8), new Pose(60, 12))
            ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(85)).build();

            shootPreLoad = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(60, 12), new Pose(60, 22))
            ).setLinearHeadingInterpolation(Math.toRadians(85), Math.toRadians(113)).build();

            toFirstBalls = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(60, 22), new Pose(44, 38))
            ).setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(180)).build();

            throughFirstBalls = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(44, 38), new Pose(16, 38))
            ).setTangentHeadingInterpolation().build();

            shootFirstBalls = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(16, 38), new Pose(60, 22))
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(113)).build();

            toSecondBalls = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(60, 22), new Pose(44.951, 62.854))
            ).setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(180)).build();

            throughSecondBalls = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(44.951, 62.854), new Pose(17.976, 62.463))
            ).setTangentHeadingInterpolation().build();

            shootSecondBalls = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(17.976, 62.463), new Pose(60, 22))
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(113)).build();

            Path9 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(60, 22), new Pose(60, 38))
            ).setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(90)).build();
        }
    }
}
*/
