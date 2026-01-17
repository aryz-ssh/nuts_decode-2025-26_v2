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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;

@Autonomous(name = "New Red 3 ball Auto - 3 ball", group = "Autonomous")
@Configurable
public class NewRedAuto3BallBigTriangle extends LinearOpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private Paths paths;
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

    NewRedAuto3BallBigTriangle.AutoState autoState =
            NewRedAuto3BallBigTriangle.AutoState.WAIT_FOR_HOME;

    int currentPocket = 1;
    boolean sorterCommanded = false;

    boolean toShootStarted = false;
    boolean moveAwayStarted = false;
    boolean returnCommanded = false;

    // ================= DASHBOARD TUNABLES =================

    public static double OUTTAKE_POWER = 0.55;
    public static double RAMP_ANGLE_TARGET = 0.99;
    public static double RAMP_UP_TIME = 1.5;

    public static double SORTER_SETTLE_TIME = 0.4;

    public static double FIRST_KICK_DELAY = 0.30;
    public static double SECOND_KICK_DELAY = 0.45;

    ElapsedTime stateTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(117, 131.5, Math.toRadians(36)));

        paths = new Paths(follower);

        mechanisms = new Mechanisms();
        mechanisms.initMechanisms(hardwareMap, telemetry, true);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        while (!isStarted() && !isStopRequested()) {
            mechanisms.sorterInitLoop();
            idle();
        }

        waitForStart();

        autoState = NewRedAuto3BallBigTriangle.AutoState.START_TO_SHOOT;

        while (opModeIsActive()) {
            if (autoState != NewRedAuto3BallBigTriangle.AutoState.WAIT_FOR_HOME) {
                follower.update();
            }
            mechanisms.updateMechanisms();

            switch (autoState) {
                case START_TO_SHOOT:
                    if (!toShootStarted) {
                        follower.followPath(paths.ToShoot);
                        toShootStarted = true;
                    }
                    autoState = NewRedAuto3BallBigTriangle.AutoState.WAIT_TO_SHOOT;
                    break;

                case START_OUTTAKE:
                    if (!outtakeStarted) {
                        mechanisms.engageOuttake(OUTTAKE_POWER);
                        mechanisms.setRampAngle(RAMP_ANGLE_TARGET);
                        stateTimer.reset();
                        outtakeStarted = true;
                    }
                    autoState = NewRedAuto3BallBigTriangle.AutoState.RAMP_UP;
                    break;

                case WAIT_TO_SHOOT:
                    if (!follower.isBusy()) {
                        autoState = NewRedAuto3BallBigTriangle.AutoState.START_OUTTAKE;
                    }
                    break;

                case RAMP_UP:
                    if (stateTimer.seconds() >= RAMP_UP_TIME) {
                        sorterCommanded = false;
                        autoState = NewRedAuto3BallBigTriangle.AutoState.MOVE_SORTER;
                    }
                    break;

                case MOVE_SORTER:
                    if (!sorterCommanded) {
                        mechanisms.sorterGoToOuttake(currentPocket);
                        sorterCommanded = true;
                    }
                    autoState = NewRedAuto3BallBigTriangle.AutoState.WAIT_SORTER;
                    break;

                case WAIT_SORTER:
                    if (!isSorterMoving()) {
                        stateTimer.reset();
                        autoState = NewRedAuto3BallBigTriangle.AutoState.SORTER_SETTLE;
                    }
                    break;

                case SORTER_SETTLE:
                    if (stateTimer.seconds() >= SORTER_SETTLE_TIME) {
                        autoState = NewRedAuto3BallBigTriangle.AutoState.KICK_1;
                    }
                    break;

                case KICK_1:
                    mechanisms.setShotPocket(currentPocket);
                    mechanisms.ejectBall();
                    stateTimer.reset();
                    autoState = NewRedAuto3BallBigTriangle.AutoState.WAIT_1;
                    break;

                case WAIT_1:
                    if (stateTimer.seconds() >= FIRST_KICK_DELAY) {
                        autoState = NewRedAuto3BallBigTriangle.AutoState.KICK_2;
                    }
                    break;

                case KICK_2:
                    mechanisms.ejectBall();
                    stateTimer.reset();
                    autoState = NewRedAuto3BallBigTriangle.AutoState.WAIT_2;
                    break;

                case WAIT_2:
                    if (stateTimer.seconds() >= SECOND_KICK_DELAY) {
                        autoState = NewRedAuto3BallBigTriangle.AutoState.NEXT_POCKET;
                    }
                    break;

                case NEXT_POCKET:
                    currentPocket++;
                    sorterCommanded = false;

                    if (currentPocket > 3) {
                        mechanisms.disengageOuttake();
                        autoState = NewRedAuto3BallBigTriangle.AutoState.START_MOVE_AWAY;
                    } else {
                        autoState = NewRedAuto3BallBigTriangle.AutoState.MOVE_SORTER;
                    }
                    break;

                case START_MOVE_AWAY:
                    if (!moveAwayStarted) {
                        follower.followPath(paths.MoveAway);
                        moveAwayStarted = true;
                    }
                    autoState = NewRedAuto3BallBigTriangle.AutoState.WAIT_MOVE_AWAY;
                    break;

                case WAIT_MOVE_AWAY:
                    if (!follower.isBusy()) {
                        autoState = NewRedAuto3BallBigTriangle.AutoState.RETURN_TO_INTAKE;
                    }
                    break;

                case RETURN_TO_INTAKE:
                    if (!returnCommanded) {
                        mechanisms.disengageOuttake();
                        mechanisms.setRampAngle(Mechanisms.RAMP_ANGLE_MIN_POS);
                        mechanisms.sorterGoToIntake(1);
                        returnCommanded = true;
                    }
                    autoState = NewRedAuto3BallBigTriangle.AutoState.WAIT_RETURN_TO_INTAKE;
                    break;

                case WAIT_RETURN_TO_INTAKE:
                    if (!isSorterMoving()) {
                        autoState = NewRedAuto3BallBigTriangle.AutoState.DONE;
                    }
                    break;

                case DONE:
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

    public static class Paths {

        public PathChain ToShoot;
        public PathChain MoveAway;

        public Paths(Follower follower) {
            ToShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(117, 131.500),
                                    new Pose(103.39568748502913, 103.44840303168627)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(36))
                    .setReversed()
                    .build();

            MoveAway = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(103.44840303168627, 103.44840303168627),
                                    new Pose(104.20516505726476, 72.08176831028842)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(270))
                    .build();
        }
    }
}
