package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "PostNut")
public class PostNut extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private long lastTelem = 0;
    // ---------- Gamepad ----------
    public static double DEADZONE = 0.05;
    public static double GAMEPAD_TRIGGER_THRESHOLD = 0.25;
    // ---------- Mechanisms ----------
    private Mechanisms mechanisms;
    private MasterDrivetrain drivetrain;
    private AprilTagLimelight limelight;
    // ---------- Intake ----------
    private boolean intakeToggle = false;
    private boolean lastIntakeTrigger = false;
    // ---------- Sorter ----------
    private int selectedPocket = 1;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastA = false;
    private boolean lastX = false;
    private enum SorterMode { NONE, INTAKE, OUTTAKE }
    private SorterMode sorterMode = SorterMode.INTAKE;
    private boolean lastGreenRequest = false;
    private boolean lastPurpleRequest = false;
    // ---------- Outtake ----------
    private boolean outtakeOn = false;
    private boolean lastB = false;
    // ---------- Ramp angle debounce ----------
    private boolean lastRB2 = false;
    private boolean lastLB2 = false;
    // ---------- Outtake speed debounce ----------
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    // ---------- Drive / Limelight ----------
    private boolean allianceChosen = false;
    private boolean pipelineSet = false;
    private boolean isRedAlliance = false;

    @Override
    public void runOpMode() throws InterruptedException {
        mechanisms = new Mechanisms();
        mechanisms.initMechanisms(hardwareMap, telemetry,true);
        drivetrain = new MasterDrivetrain();
        drivetrain.init(hardwareMap);
        limelight = new AprilTagLimelight(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // ----- Pre-start: select alliance & pipeline -----
        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("=== ROBOT-CENTRIC MODE ===");
            telemetry.addLine("Select Alliance for Limelight:");
            telemetry.addLine("X = BLUE  (Pipeline 9)");
            telemetry.addLine("B = RED   (Pipeline 8)");
            if (!allianceChosen) {
                if (gamepad1.x) {
                    isRedAlliance = false;
                    allianceChosen = true;
                }
                if (gamepad1.b) {
                    isRedAlliance = true;
                    allianceChosen = true;
                }
            }
            // Set limelight pipeline once
            if (allianceChosen && !pipelineSet) {
                int pipeline = isRedAlliance ? 8 : 9;
                limelight.setPipeline(pipeline);
                pipelineSet = true;
            }
            telemetry.addData("Alliance",
                    allianceChosen ? (isRedAlliance ? "RED" : "BLUE") : "CHOOSE");
            telemetry.addData("Limelight Pipeline",
                    pipelineSet ? (isRedAlliance ? "8 (RED)" : "9 (BLUE)") : "WAITING");
//            mechanisms.sorterInitLoop();
            telemetry.update();
            sleep(20);
        }

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // =============================================================
            //                GAMEPAD 1 — DRIVE AND INTAKE
            // =============================================================
            if (gamepad1.back) drivetrain.resetImuYaw();

            // Stick input
            double y  = applyDeadband(-gamepad1.left_stick_y);
            double x  = applyDeadband(gamepad1.left_stick_x);
            double rx = applyDeadband(gamepad1.right_stick_x);

            // ----- Auto-strafe (AprilTag) overrides x-axis -----
            boolean brake = gamepad1.left_trigger > GAMEPAD_TRIGGER_THRESHOLD;
            if (gamepad1.x) {
                double autoTurn = limelight.getAutoStrafePower(true, rx);
                drivetrain.driveRobotCentric(x, y, autoTurn, brake);
            } else {
                drivetrain.driveRobotCentric(x, y, rx, brake);
            }

            // ---------- Intake toggle (GAMEPAD 1) ----------
            boolean intakeTriggerNow = gamepad1.right_trigger > GAMEPAD_TRIGGER_THRESHOLD;
            boolean reversePressed = gamepad1.left_bumper;

            if (intakeTriggerNow && !lastIntakeTrigger) {
                intakeToggle = !intakeToggle;

                if (intakeToggle) {
                    // ---------- NEW: Force sorter to intake mode when intake is on ----------
                    sorterMode = SorterMode.INTAKE;
//                    mechanisms.sorterGoToIntake(selectedPocket);

                    mechanisms.engageIntake(1.0, reversePressed);
                } else {
                    mechanisms.disengageIntake();
                    // Optionally restore previous sorterMode or let GP2 control it
                    // (we leave it as-is for GP2 to take over)
                }
            }
            lastIntakeTrigger = intakeTriggerNow;

            // Keep intake running if toggle is active
            if (intakeToggle) mechanisms.engageIntake(1.0, reversePressed);

            // =============================================================
            //                       GAMEPAD 2 — MECHANISMS
            // =============================================================

            // Pocket select
            if (gamepad2.dpad_right && !lastDpadRight) {
                mechanisms.sorterStepForward();
            }
            if (gamepad2.dpad_left && !lastDpadLeft) {
                mechanisms.sorterStepBack();
            }
            lastDpadRight = gamepad2.dpad_right;
            lastDpadLeft = gamepad2.dpad_left;

            // Color requests
//            handleColorRequest(SorterLogicColor.BallColor.GREEN, gamepad2.left_trigger > GAMEPAD_TRIGGER_THRESHOLD);
//            handleColorRequest(SorterLogicColor.BallColor.PURPLE, gamepad2.right_trigger > GAMEPAD_TRIGGER_THRESHOLD);

            // A/X/B/Y buttons
            if (gamepad2.a && !lastA) {
//                mechanisms.sorterLogic.markPocketReady(selectedPocket);
//                mechanisms.sorterGoToIntake(selectedPocket);
                sorterMode = SorterMode.INTAKE;
            }
            lastA = gamepad2.a;

            if (gamepad2.x && !lastX) {
//                mechanisms.sorterGoToOuttake(selectedPocket);
                sorterMode = SorterMode.OUTTAKE;
            }
            lastX = gamepad2.x;

            if (gamepad2.b && !lastB) {
                outtakeOn = !outtakeOn;
                if (outtakeOn) mechanisms.engageOuttake(mechanisms.getManualOuttakeSpeed());
                else mechanisms.disengageOuttake();
            }
            lastB = gamepad2.b;

            if (gamepad2.dpad_up && !lastDpadUp) mechanisms.increaseOuttakeSpeed(0.1);
            if (gamepad2.dpad_down && !lastDpadDown) mechanisms.decreaseOuttakeSpeed(0.1);
            lastDpadUp = gamepad2.dpad_up;
            lastDpadDown = gamepad2.dpad_down;

            if (gamepad2.y) {
                mechanisms.setShotPocket(selectedPocket);
                mechanisms.ejectBall();
            }

            // Ramp angle adjustments
            boolean rb2 = gamepad2.right_bumper;
            boolean lb2 = gamepad2.left_bumper;
            if (rb2 && !lastRB2) mechanisms.adjustOuttakeAngle(true, false);
            if (lb2 && !lastLB2) mechanisms.adjustOuttakeAngle(false, true);
            lastRB2 = rb2;
            lastLB2 = lb2;

            // Update mechanisms
            mechanisms.updateMechanisms();

            // =============================================================
            //                          TELEMETRY
            // =============================================================
            telemetry.addLine("---- LIMELIGHT (AprilTag) ----");
            telemetry.addData("Auto Strafe Power", "%.2f", limelight.getAutoStrafePower(gamepad1.x, rx));
            telemetry.addData("tx (AprilTag)", limelight.getTx());
            telemetry.addData("Distance (in)", limelight.getDistance());
            if (System.currentTimeMillis() - lastTelem > 100) {
                lastTelem = System.currentTimeMillis();
                telemetry.addData("Mode", sorterMode);
//                int curPos = mechanisms.getSorterCurrentPosition();
//                int tgtPos = mechanisms.getSorterTargetPosition();
                telemetry.addData("Outtake Power", mechanisms.getManualOuttakeSpeed());
                telemetry.addData("Current Outtake Velocity", "%.0f t/s", mechanisms.outtakeMotor.getVelocity());
                telemetry.addData("Ramp Angle", "%.2f / %.2f",
                        mechanisms.getRampAngleCurrent(), mechanisms.getRampAngleTarget());
                telemetry.addData("Time", runtime.seconds());
                telemetry.update();
            } else {
                telemetry.update();
            }

            // ================= SORTER DASHBOARD TELEMETRY =================
            TelemetryPacket packet = new TelemetryPacket();

            packet.put("sorter/error_ticks",
                    mechanisms.sorterLogic.getErrorTicks());
            packet.put("sorter/current_pos",
                    mechanisms.sorterLogic.getCurrentPosition());
            packet.put("sorter/target_pos",
                    mechanisms.sorterLogic.getTargetPosition());
            packet.put("sorter/isBusy",
                    mechanisms.sorterLogic.isBusy());

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }

    // ---------- Helpers ----------
//    private void applySorterModeToPocket() {
//        switch (sorterMode) {
//            case INTAKE:
//                mechanisms.sorterGoToIntake(selectedPocket);
//                break;
//            case OUTTAKE:
//                mechanisms.sorterGoToOuttake(selectedPocket);
//                break;
//            case NONE:
//                break;
//        }
//    }

    private double applyDeadband(double value) {
        return Math.abs(value) > DEADZONE ? value : 0;
    }

//    private void handleColorRequest(SorterLogicColor.BallColor color, boolean pressed) {
//        boolean lastRequest = (color == SorterLogicColor.BallColor.GREEN) ? lastGreenRequest : lastPurpleRequest;
//        if (pressed && !lastRequest) {
//            Integer pocket = mechanisms.sorterLogic.getPocketWithColor(color);
//            if (pocket != null) {
//                selectedPocket = pocket;
//                sorterMode = SorterMode.OUTTAKE;
//                mechanisms.sorterLogic.setCurrentIntakePocket(pocket);
//                mechanisms.sorterGoToOuttake(selectedPocket);
//            }
//        }
//        if (color == SorterLogicColor.BallColor.GREEN) lastGreenRequest = pressed;
//        else lastPurpleRequest = pressed;
//    }
}
