package org.firstinspires.ftc.teamcode.autocode;

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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.util.ArrayList;

@Autonomous(name = "Close Red Auto - 3 ball", group = "Autonomous")
@Configurable
public class RedAuto3BallBigTriangle extends LinearOpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private Paths paths;
  //  private Mechanisms mechanisms;
    private Mechanisms mechanisms;

    private ArrayList<String> intakeOrder = new ArrayList<>();
    private boolean intakeOn = false;
    private long delayStart = 0;
    private final long DELAY_MS = 400;

    private String lastDetectedColor = null;

    private enum AutoState {
        START_TO_SHOOT,
        WAIT_TO_SHOOT,

        START_OUTTAKE,
        RAMP_UP,
        WAIT_FOR_HOME,
        SORTER_SETTLE,

        MOVE_SORTER,
        WAIT_SORTER,
        KICK_1,
        WAIT_1,
        KICK_2,
        WAIT_2,
        NEXT_POCKET,

        START_MOVE_AWAY,
        WAIT_MOVE_AWAY,

        RETURN_TO_INTAKE,
        WAIT_RETURN_TO_INTAKE,

        DONE
    }

    boolean outtakeStarted = false;

    AutoState autoState = AutoState.WAIT_FOR_HOME;

    int currentPocket = 1;
    boolean sorterCommanded = false;

    boolean toShootStarted = false;
    boolean moveAwayStarted = false;
    boolean returnCommanded = false;


    // ================= DASHBOARD TUNABLES =================

    // Shooter / outtake
    public static double OUTTAKE_POWER = 0.55;
    public static double RAMP_ANGLE_TARGET = 0.99;
    public static double RAMP_UP_TIME = 1.5;

    // Sorter timing
    public static double SORTER_SETTLE_TIME = 0.4;

    // Kicker timing
    public static double FIRST_KICK_DELAY = 0.30;
    public static double SECOND_KICK_DELAY = 0.45;

    ElapsedTime stateTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(117, 131.5, Math.toRadians(36)));

        mechanisms = new Mechanisms();
        mechanisms.initMechanisms(hardwareMap, telemetry);

        paths = new Paths(follower);

        //mechanisms = new Mechanisms();
        //mechanisms.initMechanisms(hardwareMap, telemetry);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        while (!isStarted() && !isStopRequested()) {
            mechanisms.sorterInitLoop();   // homing ONLY
            idle();
        }

        waitForStart();

        autoState = AutoState.START_TO_SHOOT;

        while (opModeIsActive()) {
            if (autoState != AutoState.WAIT_FOR_HOME) {
                follower.update();
            }
            mechanisms.updateMechanisms();

            switch (autoState) {
                case START_TO_SHOOT:
                    if (!toShootStarted) {
                        follower.followPath(paths.ToShoot);
                        toShootStarted = true;
                    }
                    autoState = AutoState.WAIT_TO_SHOOT;
                    break;

                // ===============================
                // DRIVE TO SHOOT POSITION
                // ===============================
                case START_OUTTAKE:
                    if (!outtakeStarted) {
                        mechanisms.engageOuttake(OUTTAKE_POWER);
                        mechanisms.setRampAngle(RAMP_ANGLE_TARGET);
                        stateTimer.reset();
                        outtakeStarted = true;
                    }
                    autoState = AutoState.RAMP_UP;
                    break;

                case WAIT_TO_SHOOT:
                    if (!follower.isBusy()) {
                        autoState = AutoState.START_OUTTAKE;
                    }
                    break;

                // ===============================
                // WAIT FOR FULL RPM + RAMP
                // ===============================
                case RAMP_UP:
                    if (stateTimer.seconds() >= RAMP_UP_TIME) {
                        sorterCommanded = false;
                        autoState = AutoState.MOVE_SORTER;
                    }
                    break;

                // ===============================
                // MOVE SORTER TO POCKET
                // ===============================
                case MOVE_SORTER:
                    if (!sorterCommanded) {
                        mechanisms.sorterGoToOuttake(currentPocket);
                        sorterCommanded = true;
                    }
                    autoState = AutoState.WAIT_SORTER;
                    break;

                case WAIT_SORTER:
                    if (!isSorterMoving()) {
                        stateTimer.reset();
                        autoState = AutoState.SORTER_SETTLE;
                    }
                    break;

                case SORTER_SETTLE:
                    if (stateTimer.seconds() >= SORTER_SETTLE_TIME) {   // ← tune: 0.35–0.5
                        autoState = AutoState.KICK_1;
                    }
                    break;

                // ===============================
                // FIRST KICK
                // ===============================
                case KICK_1:
                    mechanisms.setShotPocket(currentPocket);
                    mechanisms.ejectBall();
                    stateTimer.reset();
                    autoState = AutoState.WAIT_1;
                    break;

                // ===============================
                // WAIT AFTER FIRST KICK
                // ===============================
                case WAIT_1:
                    if (stateTimer.seconds() >= FIRST_KICK_DELAY) {
                        autoState = AutoState.KICK_2;
                    }
                    break;

                // ===============================
                // SECOND FAILSAFE KICK
                // ===============================
                case KICK_2:
                    mechanisms.ejectBall();
                    stateTimer.reset();
                    autoState = AutoState.WAIT_2;
                    break;

                // ===============================
                // WAIT AFTER SECOND KICK
                // ===============================
                case WAIT_2:
                    if (stateTimer.seconds() >= SECOND_KICK_DELAY) {
                        autoState = AutoState.NEXT_POCKET;
                    }
                    break;

                // ===============================
                // ADVANCE TO NEXT POCKET
                // ===============================
                case NEXT_POCKET:
                    currentPocket++;
                    sorterCommanded = false;

                    if (currentPocket > 3) {
                        mechanisms.disengageOuttake();
                        autoState = AutoState.START_MOVE_AWAY;
                    } else {
                        autoState = AutoState.MOVE_SORTER;
                    }
                    break;

                // ===============================
                // DRIVE AWAY
                // ===============================
                case START_MOVE_AWAY:
                    if (!moveAwayStarted) {
                        follower.followPath(paths.moveAway);
                        moveAwayStarted = true;
                    }
                    autoState = AutoState.WAIT_MOVE_AWAY;
                    break;

                case WAIT_MOVE_AWAY:
                    if (!follower.isBusy()) {
                        autoState = AutoState.RETURN_TO_INTAKE;
                    }
                    break;

                case RETURN_TO_INTAKE:
                    if (!returnCommanded) {
                        mechanisms.disengageOuttake();
                        mechanisms.setRampAngle(Mechanisms.RAMP_ANGLE_MIN_POS);
                        mechanisms.sorterGoToIntake(1);
                        returnCommanded = true;
                    }
                    autoState = AutoState.WAIT_RETURN_TO_INTAKE;
                    break;

                case WAIT_RETURN_TO_INTAKE:
                    if (!isSorterMoving()) {
                        autoState = AutoState.DONE;
                    }
                    break;


                // ===============================
                // FINISHED
                // ===============================
                case DONE:
                    // Robot parked. Mechanisms are safe. Do nothing.
                    break;
            }

            panelsTelemetry.debug("AutoState", autoState);
            panelsTelemetry.debug("Pocket", currentPocket);
            panelsTelemetry.debug("X", follower.getPose().getX());
            panelsTelemetry.debug("Y", follower.getPose().getY());
            panelsTelemetry.update(telemetry);
        }

    }

    public boolean isSorterMoving() {
        return mechanisms.isSorterMoving();
    }

    // ---------------- PATH LIST ----------------
    public static class Paths {

        public PathChain ToShoot;
        public PathChain moveAway;

        public Paths(Follower follower) {
            ToShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(117.000, 131.500), new Pose(98.60508083140877, 115.56581986143188))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(36))
                    .build();

            moveAway = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(98.60508083140877, 115.56581986143188), new Pose(96.31475409836065, 132.6688524590164))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(270))
                    .build();
        }


    }
}
