package org.firstinspires.ftc.teamcode.autocode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AprilTagLimelight;
import org.firstinspires.ftc.teamcode.Mechanisms;
import org.firstinspires.ftc.teamcode.SorterLogicColor;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue 9 Ball Big Triangle", group = "Auto")
public class BlueAuto9BallBigTriangle extends LinearOpMode {

    // ---------------- Paths ----------------
    public PathChain scanMotif, preloadShoot;
    public PathChain intake1Prep, intake1Through, shoot1;
    public PathChain intake2Prep, intake2Through, shoot2;
    public PathChain path9;

    // ---------------- Enums ----------------
    private enum AutoState {
        START,
        DRIVE_SCAN_MOTIF,
        SCAN_MOTIF,
        PRELOAD_SHOOT,
        INTAKE1_PREP,
        INTAKE1_THROUGH,
        SHOOT1,
        INTAKE2_PREP,
        INTAKE2_THROUGH,
        SHOOT2,
        PARK,
        IDLE
    }

    public enum Motif {
        GPP, PGP, PPG, UNKNOWN
    }

    // ---------- SHOOTING CONTROL ----------
    private enum ShootState {
        IDLE,
        MOVE_SORTER,
        SETTLE,
        KICK,
        WAIT_BETWEEN,
        DONE
    }

    private ShootState shootState = ShootState.IDLE;
    private final ElapsedTime shootTimer = new ElapsedTime();

    // Order of pockets to shoot (example)
    private int[] shootOrder = {0, 1, 2};
    private int shootIndex = 0;

    // Timing constants
    private static final double SORTER_SETTLE_SEC = 0.2;
    private static final double BETWEEN_SHOTS_SEC = 0.6;
    private static final double OUTTAKE_SPEED = 0.8;
    private static final double RAMP_ANGLE = 0.5;

    // ---------------- State ----------------
    private AutoState state = AutoState.START;
    private Motif detectedMotif = Motif.GPP;   // DEFAULT

    private Follower follower;
    private Mechanisms mechanisms;
    private AprilTagLimelight limelight;

    private final ElapsedTime motifTimer = new ElapsedTime();

    private static final int MOTIF_PIPELINE = 2;
    private static final double MOTIF_TIMEOUT_SEC = 1.5;

    // ================= RUN OPMODE =================
    @Override
    public void runOpMode() {

        // -------- INIT HARDWARE --------
        mechanisms = new Mechanisms();
        mechanisms.initMechanisms(hardwareMap, telemetry, true);

        limelight = new AprilTagLimelight(hardwareMap);

        follower = Constants.createFollower(hardwareMap);

        // ✅ REQUIRED: SET STARTING POSE
        follower.setStartingPose(
                new Pose(
                        32.630,
                        135.003,
                        Math.toRadians(90)
                )
        );

        // Build paths AFTER pose is set
        buildPaths();

        telemetry.addLine("Initialized. Homing sorter...");
        telemetry.update();

        // -------- PRE-START HOMING --------
        while (!isStarted() && !isStopRequested()) {
            mechanisms.sorterInitLoop();
            mechanisms.updateMechanisms();
            telemetry.addData("Sorter Homed", mechanisms.isSorterHomed());
            telemetry.update();
            idle();
        }

        waitForStart();
        if (!opModeIsActive()) return;

        // -------- PRELOAD ASSUMPTION --------
        applyPreloadPocketAssumption();

        // -------- MAIN LOOP --------
        while (opModeIsActive()) {
            follower.update();              // 🚨 REQUIRED
            mechanisms.updateMechanisms();
            updateAuto();

            telemetry.addData("State", state);
            telemetry.addData("Motif", detectedMotif);
            telemetry.update();
        }
    }

    // ================= PRELOAD ASSUMPTION =================
    private void applyPreloadPocketAssumption() {
        mechanisms.sorterLogic.pocketColors[0] = SorterLogicColor.BallColor.GREEN;
        mechanisms.sorterLogic.pocketColors[1] = SorterLogicColor.BallColor.PURPLE;
        mechanisms.sorterLogic.pocketColors[2] = SorterLogicColor.BallColor.PURPLE;

        mechanisms.sorterLogic.pocketReady[0] = false;
        mechanisms.sorterLogic.pocketReady[1] = false;
        mechanisms.sorterLogic.pocketReady[2] = false;
    }

    // ================= AUTO STATE MACHINE =================
    private void updateAuto() {

        switch (state) {

            case START:
                follower.followPath(scanMotif);
                state = AutoState.DRIVE_SCAN_MOTIF;
                break;

            case DRIVE_SCAN_MOTIF:
                if (!follower.isBusy()) {
                    limelight.setPipeline(MOTIF_PIPELINE);
                    motifTimer.reset();
                    state = AutoState.SCAN_MOTIF;
                }
                break;

            case SCAN_MOTIF:
                if (motifTimer.seconds() >= MOTIF_TIMEOUT_SEC) {
                    detectedMotif = Motif.GPP; // placeholder

                    // mechanisms.engageOuttake(OUTTAKE_SPEED);

                    follower.followPath(preloadShoot);
                    state = AutoState.PRELOAD_SHOOT;
                }
                break;

            case PRELOAD_SHOOT:
                if (!follower.isBusy()) {
                    follower.followPath(intake1Prep);
                    state = AutoState.INTAKE1_PREP;
                }
                break;

            case INTAKE1_PREP:
                if (!follower.isBusy()) {
                    follower.followPath(intake1Through);
                    state = AutoState.INTAKE1_THROUGH;
                }
                break;

            case INTAKE1_THROUGH:
                if (!follower.isBusy()) {
                    follower.followPath(shoot1);
                    state = AutoState.SHOOT1;
                }
                break;

            case SHOOT1:
                if (!follower.isBusy()) {
                    follower.followPath(intake2Prep);
                    state = AutoState.INTAKE2_PREP;
                }
                break;

            case INTAKE2_PREP:
                if (!follower.isBusy()) {
                    follower.followPath(intake2Through);
                    state = AutoState.INTAKE2_THROUGH;
                }
                break;

            case INTAKE2_THROUGH:
                if (!follower.isBusy()) {
                    follower.followPath(shoot2);
                    state = AutoState.SHOOT2;
                }
                break;

            case SHOOT2:
                if (!follower.isBusy()) {
                    follower.followPath(path9);
                    state = AutoState.PARK;
                }
                break;

            case PARK:
                if (!follower.isBusy()) {
                    state = AutoState.IDLE;
                }
                break;

            case IDLE:
                break;
        }
    }

    // ================= PATH BUILD =================
    private void buildPaths() {

        scanMotif = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(32.630, 135.003),

                                new Pose(53.875, 117.207)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(60))

                .build();

        preloadShoot = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(53.875, 117.207),

                                new Pose(54.092, 99.574)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(137))

                .build();

        intake1Prep = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(54.092, 99.574),

                                new Pose(44.993, 90.531)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))

                .build();

        intake1Through = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(44.993, 90.531),

                                new Pose(14.636, 90.177)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        shoot1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(14.636, 90.177),

                                new Pose(54.092, 99.574)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))

                .build();

        intake2Prep = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(54.092, 99.574),

                                new Pose(45.702, 66.734)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))

                .build();

        intake2Through = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(45.702, 66.734),

                                new Pose(18.433, 66.374)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        shoot2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(18.433, 66.374),

                                new Pose(54.092, 99.574)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))

                .build();

        path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(54.092, 99.574),

                                new Pose(54.564, 129.600)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(270))

                .build();
    }
}
