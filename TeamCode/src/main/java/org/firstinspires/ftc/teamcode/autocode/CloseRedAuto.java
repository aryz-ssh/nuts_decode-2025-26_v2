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
public class CloseRedAuto extends LinearOpMode {

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

        MOVE_SORTER,
        WAIT_SORTER,
        KICK_1,
        WAIT_1,
        KICK_2,
        WAIT_2,
        NEXT_POCKET,

        START_MOVE_AWAY,
        WAIT_MOVE_AWAY,

        DONE
    }

    boolean outtakeStarted = false;

    AutoState autoState = AutoState.WAIT_FOR_HOME;

    int currentPocket = 1;
    boolean sorterCommanded = false;

    boolean toShootStarted = false;
    boolean moveAwayStarted = false;

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

        // ===============================
        // SORTER HOMING INIT LOOP
        // ===============================
        while (!isStarted() && !isStopRequested()) {
            mechanisms.sorterInitLoop();      // decides homing actions
            mechanisms.updateMechanisms();    // actually runs sorterLogic.update() -> motor moves

            panelsTelemetry.debug("Sorter", "Homing...");
            panelsTelemetry.debug("SorterPos", mechanisms.getSorterCurrentPosition());
            panelsTelemetry.debug("SorterTarget", mechanisms.getSorterTargetPosition());
            panelsTelemetry.update(telemetry);

            idle();        // IMPORTANT: let hardware loop breathe
            // or sleep(10);
        }

        waitForStart();

        autoState = mechanisms.isSorterHomed() ? AutoState.START_TO_SHOOT : AutoState.WAIT_FOR_HOME;


        while (opModeIsActive()) {
            if (autoState != AutoState.WAIT_FOR_HOME) {
                follower.update();
            }
            mechanisms.updateMechanisms();

            switch (autoState) {
                case WAIT_FOR_HOME:
                    // keep homing state machine running AFTER start
                    mechanisms.sorterInitLoop();
                    mechanisms.updateMechanisms();

                    panelsTelemetry.debug("WAIT", "Homing");
                    panelsTelemetry.debug("SorterPos", mechanisms.getSorterCurrentPosition());
                    panelsTelemetry.debug("SorterHomed", mechanisms.isSorterHomed());
                    panelsTelemetry.update(telemetry);

                    if (mechanisms.isSorterHomed()) {
                        autoState = AutoState.START_TO_SHOOT;
                    }
                    break;


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
                        mechanisms.engageOuttake(0.7);
                        mechanisms.setRampAngle(0.9);
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
                    if (stateTimer.seconds() >= 1.5) {
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
                    if (sorterAtTarget()) {
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
                    if (stateTimer.seconds() >= 0.30) {
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
                    if (stateTimer.seconds() >= 0.45) {
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
                        autoState = AutoState.DONE;
                    }
                    break;




                // ===============================
                // FINISHED
                // ===============================
                case DONE:
                    // idle
                    break;
            }

            panelsTelemetry.debug("AutoState", autoState);
            panelsTelemetry.debug("Pocket", currentPocket);
            panelsTelemetry.debug("X", follower.getPose().getX());
            panelsTelemetry.debug("Y", follower.getPose().getY());
            panelsTelemetry.update(telemetry);
        }

    }

    private boolean sorterAtTarget() {
        return Math.abs(
                mechanisms.getSorterCurrentPosition()
                        - mechanisms.getSorterTargetPosition()
        ) < 20; // tune if needed
    }

    // ---------------- PATH LIST ----------------
    public static class Paths {

        public PathChain ToShoot;
        public PathChain moveAway;

        public Paths(Follower follower) {
            ToShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(117.000, 131.500), new Pose(100.09180327868853, 117.79672131147541))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(36))
                    .build();

            moveAway = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(100.09180327868853, 117.79672131147541), new Pose(96.31475409836065, 132.6688524590164))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(270))
                    .build();
        }


    }
}
