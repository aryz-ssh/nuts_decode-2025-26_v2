package org.firstinspires.ftc.teamcode.autocode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FinalSorter;
import org.firstinspires.ftc.teamcode.Mechanisms;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "RedFar3", group = "Autonomous")
@Config
public class RedFar3 extends LinearOpMode {

    /* ================= CORE ================= */

    private Follower follower;
    private Paths paths;
    private Mechanisms mechanisms;

    /* ================= SHOOTING CONFIG (FROM RedClose) ================= */

    public static double OUTTAKE_POWER = 1.0;
    public static long OUTTAKE_SPINUP_MS = 400;
    public static long SHOT_SPACING_MS = 600;
    public static long SORTER_POST_BUSY_MS = 400;

    private boolean outtakeSpinning = false;
    private long outtakeSpinupStart = -1;

    private boolean shotInProgress = false;
    private long shootStartTimeMs = 0;

    private boolean pocketAligned = false;
    private long sorterNotBusySince = -1;

    private int shotsFired = 0;

    /* ================= AUTO STATES ================= */

    private enum AutoState {
        DRIVE_FORWARD,
        DRIVE_INTO_TRIANGLE,
        SHOOT_3,
        PARK,
        DONE
    }

    /* ================= SORTER-BASED SHOOT (EXTRACTED FROM RedClose) ================= */

    private boolean shootNextBall() {

        if (shotsFired >= 3) {
            return true;
        }

        int pocket = mechanisms.sorter.getPocketWithAnyBall();
        if (pocket == -1) {
            shotsFired = 3;
            return true;
        }

        // Align pocket
        if (!pocketAligned && !mechanisms.isSorterBusy()) {
            sorterNotBusySince = -1;
            mechanisms.sorter.movePocketToOuttake(pocket);
            pocketAligned = true;
            return false;
        }

        // Wait for sorter to settle
        if (pocketAligned && !mechanisms.isSorterBusy()) {
            if (sorterNotBusySince < 0) {
                sorterNotBusySince = System.currentTimeMillis();
                return false;
            }
        } else {
            sorterNotBusySince = -1;
        }

        // Fire
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

        // Spacing + cleanup
        if (shotInProgress &&
                System.currentTimeMillis() - shootStartTimeMs >= SHOT_SPACING_MS) {

            mechanisms.sorter.onBallEjected();
            shotInProgress = false;
            pocketAligned = false;
            shotsFired++;
        }

        return false;
    }

    /* ================= PATHS (FROM VISUALIZER, FIXED) ================= */

    public static class Paths {

        public PathChain Path1; // forward
        public PathChain Path2; // into triangle
        public PathChain Path3; // park

        public Paths(Follower follower) {

            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(51.317, 8.780),
                            new Pose(51.707, 22.341)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(51.707, 22.341),
                            new Pose(65.561, 13.049)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(90),
                            Math.toRadians(85)
                    )
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(65.561, 13.049),
                            new Pose(37.390, 13.024)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }

    /* ================= MAIN ================= */

    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(51.317, 8.780, Math.toRadians(90)));


        paths = new Paths(follower);

        mechanisms = new Mechanisms();
        mechanisms.initMechanisms(hardwareMap, telemetry, true);

        // Declare preload layout (same assumption as RedClose)
        mechanisms.sorter.forceSetSlotColors(new FinalSorter.BallColor[]{
                FinalSorter.BallColor.GREEN,
                FinalSorter.BallColor.PURPLE,
                FinalSorter.BallColor.PURPLE
        });

        waitForStart();

        AutoState state = AutoState.DRIVE_FORWARD;

        boolean forwardStarted = false;
        boolean triangleStarted = false;
        boolean parkStarted = false;

        while (opModeIsActive()) {

            follower.update();
            mechanisms.updateMechanisms();

            switch (state) {

                case DRIVE_FORWARD:
                    if (!forwardStarted) {
                        follower.followPath(paths.Path1);
                        forwardStarted = true;
                    }
                    if (!follower.isBusy()) {
                        forwardStarted = false;
                        state = AutoState.DRIVE_INTO_TRIANGLE;
                    }
                    break;

                case DRIVE_INTO_TRIANGLE:
                    if (!triangleStarted) {
                        follower.followPath(paths.Path2);
                        triangleStarted = true;
                    }
                    if (!follower.isBusy()) {
                        triangleStarted = false;
                        shotsFired = 0;
                        state = AutoState.SHOOT_3;
                    }
                    break;

                case SHOOT_3:
                    if (!outtakeSpinning) {
                        if (outtakeSpinupStart < 0) {
                            mechanisms.engageOuttake(OUTTAKE_POWER);
                            outtakeSpinupStart = System.currentTimeMillis();
                            break;
                        }
                        if (System.currentTimeMillis() - outtakeSpinupStart < OUTTAKE_SPINUP_MS) {
                            break;
                        }
                        outtakeSpinning = true;
                        outtakeSpinupStart = -1;
                    }

                    if (shootNextBall()) {
                        mechanisms.disengageOuttake();
                        outtakeSpinning = false;
                        state = AutoState.PARK;
                    }
                    break;

                case PARK:
                    if (!parkStarted) {
                        follower.followPath(paths.Path3);
                        parkStarted = true;
                    }
                    if (!follower.isBusy()) {
                        state = AutoState.DONE;
                    }
                    break;

                case DONE:
                    mechanisms.disengageIntake();
                    mechanisms.disengageOuttake();
                    break;
            }
        }
    }
}
