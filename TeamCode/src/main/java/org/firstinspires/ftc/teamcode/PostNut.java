package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "PostNut")
public class PostNut extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Follower follower;

    long lastTelem = 0;
    public static double DEADZONE = 0.05;

    // ---------- Mechanisms ----------
    private Mechanisms mechanisms;
    private MasterDrivetrain drivetrain;

    // ---------- Sorter Variables ----------
    private int selectedPocket = 1;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastA = false;
    private boolean lastX = false;

    // ---------- Sorter Auto-Mode ----------
    private enum SorterMode { NONE, INTAKE, OUTTAKE }
    private SorterMode sorterMode = SorterMode.INTAKE;

    // ---------- Outtake ----------
    private boolean outtakeOn = false;
    private boolean lastB = false;

    // ---------- Ramp angle debounce ----------
    private boolean lastRB2 = false;
    private boolean lastLB2 = false;

    // ---------- Outtake debounce -------------
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    // init stuff

    boolean allianceChosen = false;
    boolean driveModeChosen = false;
    boolean startAngleChosen = false;
    private boolean lastBack = false;
    private boolean lastGreenRequest = false;
    private boolean lastPurpleRequest = false;
    private boolean intakeToggle = false;
    private boolean lastIntakeTrigger = false;

    int startAngleIndex = 0;
    // 0 = forward, 1 = back-right, 2 = back-left


    @Override
    public void runOpMode() throws InterruptedException {

        mechanisms = new Mechanisms();
        mechanisms.initMechanisms(hardwareMap, telemetry);

        drivetrain = new MasterDrivetrain();
        drivetrain.init(hardwareMap);

        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);

        telemetry.addData("Status", "HOMING SORTER…");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            // 1 — DRIVE MODE SELECTION (FIRST)
            if (!driveModeChosen) {
                telemetry.addLine("Choose Drive Mode:");
                telemetry.addLine("A = Robot-Centric");
                telemetry.addLine("Y = Field-Centric");

                if (gamepad1.a) {
                    drivetrain.fieldCentricEnabled = false;
                    driveModeChosen = true;
                    allianceChosen = true;   // skip alliance
                    startAngleChosen = true; // skip angle
                }
                if (gamepad1.y) {
                    drivetrain.fieldCentricEnabled = true;
                    driveModeChosen = true;
                }
            }

            // 2 — ALLIANCE SELECT (ONLY IF FIELD CENTRIC)
            if (driveModeChosen && drivetrain.fieldCentricEnabled && !allianceChosen) {
                telemetry.addLine("Choose Alliance:");
                telemetry.addLine("X = BLUE");
                telemetry.addLine("B = RED");

                if (gamepad1.x) {
                    drivetrain.isRedAlliance = false;
                    allianceChosen = true;
                }
                if (gamepad1.b) {
                    drivetrain.isRedAlliance = true;
                    allianceChosen = true;
                }
            }

            // 3 — START ORIENTATION SELECT (ONLY IF FIELD CENTRIC)
            if (allianceChosen && drivetrain.fieldCentricEnabled && !startAngleChosen) {
                telemetry.addLine("Choose Starting Orientation:");
                telemetry.addLine("DPAD UP: Forward");
                telemetry.addLine("DPAD RIGHT: Back-Right");
                telemetry.addLine("DPAD LEFT: Back-Left");

                if (gamepad1.dpad_up) {
                    startAngleIndex = 0;
                    startAngleChosen = true;
                }
                if (gamepad1.dpad_right) {
                    startAngleIndex = 1;
                    startAngleChosen = true;
                }
                if (gamepad1.dpad_left) {
                    startAngleIndex = 2;
                    startAngleChosen = true;
                }
            }

            // APPLY ANGLE OFFSET (ONLY IF FIELD CENTRIC)
            if (startAngleChosen && drivetrain.fieldCentricEnabled) {
                switch (startAngleIndex) {
                    case 0: drivetrain.startOffsetRadians = 0; break;                     // Forward
                    case 1: drivetrain.startOffsetRadians = -Math.PI / 4; break;          // Back-right
                    case 2: drivetrain.startOffsetRadians = -3 * Math.PI / 4; break;      // Back-left
                }
            }

            // TELEMETRY
            telemetry.addData("Drive Mode", driveModeChosen ?
                    (drivetrain.fieldCentricEnabled ? "FIELD" : "ROBOT") : "CHOOSE");

            if (drivetrain.fieldCentricEnabled) {
                telemetry.addData("Alliance", allianceChosen ?
                        (drivetrain.isRedAlliance ? "RED" : "BLUE") : "CHOOSE");

                telemetry.addData("Start Angle",
                        startAngleChosen ? startAngleIndex : "CHOOSE");
            }

            telemetry.update();

            mechanisms.sorterInitLoop();
            sleep(10);
        }


        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // =============================================================
            //                GAMEPAD 1 — DRIVE AND INTAKE
            // =============================================================

            // Reset full localization heading (Pedro IMU + Odo fused)
            if (gamepad1.back && !lastBack) {

                // 1 — Read Pedro IMU heading AFTER reset
                follower.setPose(follower.getPose().withHeading(0));

                // 2 — Feed the new heading into drivetrain
                drivetrain.startOffsetRadians = 0;
                drivetrain.resetHeadingFromFollower(0);

                // 3 — Optional: reset mecanisms IMU if you want it in sync
                mechanisms.resetHeading();
            }
            lastBack = gamepad1.back;


            double y  = applyDeadband(-gamepad1.left_stick_y);
            double x  = applyDeadband(gamepad1.left_stick_x);
            double rx = applyDeadband(gamepad1.right_stick_x);

            drivetrain.brakeAssist = gamepad1.left_trigger > 0.3;

            double heading = follower.getPose().getHeading();


            if (drivetrain.fieldCentricEnabled) {
                drivetrain.driveFieldCentric(x, y, rx, heading);
            } else {
                drivetrain.driveRobotCentric(x, y, rx, heading);
            }

            // ================= INTAKE TOGGLE (GAMEPAD1 RIGHT TRIGGER) =================
            boolean intakeTriggerNow = gamepad1.right_trigger > 0.3;
            boolean reversePressed = gamepad1.left_bumper;

            if (intakeTriggerNow && !lastIntakeTrigger) {
                intakeToggle = !intakeToggle;

                if (intakeToggle) {
                    // Enable intake
                    mechanisms.engageIntake(1.0, reversePressed);
                } else {
                    // Disable intake
                    mechanisms.disengageIntake();
                }
            }
            lastIntakeTrigger = intakeTriggerNow;

            // If intake is ON, continuously update reverse direction (safe)
            if (intakeToggle) {
                mechanisms.engageIntake(1.0, reversePressed);
            }


            // pusher
            mechanisms.pushBallToSorter(gamepad1.right_bumper);

            // =============================================================
            //                       GAMEPAD 2 — MECHANISMS
            // =============================================================

            // Pocket select + auto-move to last mode
            if (gamepad2.dpad_right && !lastDpadRight) {
                selectedPocket++;
                if (selectedPocket > 3) selectedPocket = 1;
                applySorterModeToPocket();
            }
            if (gamepad2.dpad_left && !lastDpadLeft) {
                selectedPocket--;
                if (selectedPocket < 1) selectedPocket = 3;
                applySorterModeToPocket();
            }

            lastDpadRight = gamepad2.dpad_right;
            lastDpadLeft  = gamepad2.dpad_left;

            // === GREEN REQUEST — Left Trigger ===
            boolean greenReq = gamepad2.left_trigger > 0.3;

            if (greenReq && !lastGreenRequest) {

                Integer pocket = mechanisms.sorterLogic.getPocketWithColor(
                        SorterLogicColor.BallColor.GREEN
                );

                if (pocket != null) {
                    selectedPocket = pocket;
                    sorterMode = SorterMode.OUTTAKE;

                    mechanisms.sorterLogic.setCurrentIntakePocket(pocket);
                    mechanisms.sorterGoToOuttake(selectedPocket);

                    // telemetry.addData("Color Request", "GREEN → Pocket " + pocket);
                } else {
                    // telemetry.addData("Color Request", "GREEN NOT FOUND");
                }
            }
            lastGreenRequest = greenReq;


            // === PURPLE REQUEST — Right Trigger ===
            boolean purpleReq = gamepad2.right_trigger > 0.3;

            if (purpleReq && !lastPurpleRequest) {

                Integer pocket = mechanisms.sorterLogic.getPocketWithColor(
                        SorterLogicColor.BallColor.PURPLE
                );

                if (pocket != null) {
                    selectedPocket = pocket;
                    sorterMode = SorterMode.OUTTAKE;

                    mechanisms.sorterLogic.setCurrentIntakePocket(pocket);
                    mechanisms.sorterGoToOuttake(selectedPocket);

                    // telemetry.addData("Color Request", "PURPLE → Pocket " + pocket);
                } else {
                    // telemetry.addData("Color Request", "PURPLE NOT FOUND");
                }
            }
            lastPurpleRequest = purpleReq;

            // A = go to intake for current pocket, and remember that mode
            if (gamepad2.a && !lastA) {
                mechanisms.sorterLogic.markPocketReady(selectedPocket);
                mechanisms.sorterGoToIntake(selectedPocket);
                sorterMode = SorterMode.INTAKE;
            }
            lastA = gamepad2.a;

            // X = go to outtake for current pocket, and remember that mode
            if (gamepad2.x && !lastX) {
                mechanisms.sorterGoToOuttake(selectedPocket);
                sorterMode = SorterMode.OUTTAKE;
            }
            lastX = gamepad2.x;

            if (gamepad2.b && !lastB) {
                outtakeOn = !outtakeOn;

                if (outtakeOn)
                    mechanisms.engageOuttake(mechanisms.getManualOuttakeSpeed());
                else
                    mechanisms.disengageOuttake();
            }
            lastB = gamepad2.b;

            // --- OUTTAKE SPEED ADJUST WITH DEBOUNCE ---
            if (gamepad2.dpad_up && !lastDpadUp) {
                mechanisms.increaseOuttakeSpeed(0.1);
            }
            if (gamepad2.dpad_down && !lastDpadDown) {
                mechanisms.decreaseOuttakeSpeed(0.1);
            }

            lastDpadUp = gamepad2.dpad_up;
            lastDpadDown = gamepad2.dpad_down;

            if (gamepad2.y) {
                mechanisms.ejectBall();
            }

            boolean rb2 = gamepad2.right_bumper;
            boolean lb2 = gamepad2.left_bumper;

            if (rb2 && !lastRB2) {
                mechanisms.adjustOuttakeAngle(true, false);
            }

            if (lb2 && !lastLB2) {
                mechanisms.adjustOuttakeAngle(false, true);
            }

            lastRB2 = rb2;
            lastLB2 = lb2;

            mechanisms.updateMechanisms();

            // =============================================================
            //                          TELEMETRY
            // =============================================================
            if (System.currentTimeMillis() - lastTelem > 100) {

                // --- Basic sorter mode ---
                telemetry.addData("Mode", sorterMode);

                // --- Current sorter position ---
                int curPos = mechanisms.getSorterCurrentPosition();
                int tgtPos = mechanisms.getSorterTargetPosition();
                telemetry.addData("Sorter Pos", curPos);
                telemetry.addData("Target Pos", tgtPos);
                telemetry.addData("Error", tgtPos - curPos);

                // --- Pocket Selection ---
                telemetry.addData("Selected Pocket", selectedPocket);

                // --- Pocket Colors (very simple) ---
                SorterLogicColor.BallColor[] pc = mechanisms.sorterLogic.getPocketColors();
                telemetry.addData("P1", pc[0]);
                telemetry.addData("P2", pc[1]);
                telemetry.addData("P3", pc[2]);

                // --- Outtake ---
                telemetry.addData("Outtake Speed", mechanisms.getManualOuttakeSpeed());
                telemetry.addData("Ramp Angle", "%.2f / %.2f",
                        mechanisms.getRampAngleCurrent(),
                        mechanisms.getRampAngleTarget());

                telemetry.addData("Time", runtime.seconds());
                telemetry.update();

                lastTelem = System.currentTimeMillis();
            }
        }
    }

    private void applySorterModeToPocket() {
        switch (sorterMode) {
            case INTAKE:
                mechanisms.sorterGoToIntake(selectedPocket);
                break;

            case OUTTAKE:
                mechanisms.sorterGoToOuttake(selectedPocket);
                break;

            case NONE:
            default:
                // do nothing until A or X is pressed at least once
                break;
        }
    }

    private double applyDeadband(double value) {
        return (Math.abs(value) > DEADZONE) ? value : 0;
    }
}