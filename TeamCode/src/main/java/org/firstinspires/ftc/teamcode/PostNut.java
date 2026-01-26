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
    private enum SorterMode { NONE, INTAKE, OUTTAKE }
    private SorterMode sorterMode = SorterMode.INTAKE;

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

    // ---------- Gamepad2 sorter buttons ----------
    private boolean lastX = false;
    private boolean lastY = false;

    // ---------- Auto Align ----------
    private boolean autoAlignEnabled = false;
    private boolean lastX1 = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // ----- Init -----
        mechanisms = new Mechanisms();
        mechanisms.initMechanisms(hardwareMap, telemetry, true);

        drivetrain = new MasterDrivetrain();
        drivetrain.init(hardwareMap);

        limelight = new AprilTagLimelight(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // ----- Pre-start: alliance select -----
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

            if (allianceChosen && !pipelineSet) {
                int pipeline = isRedAlliance ? 8 : 9;
                limelight.setPipeline(pipeline);
                pipelineSet = true;
            }

            telemetry.addData("Alliance", allianceChosen ? (isRedAlliance ? "RED" : "BLUE") : "CHOOSE");
            telemetry.addData("Limelight Pipeline", pipelineSet ? (isRedAlliance ? "8 (RED)" : "9 (BLUE)") : "WAITING");
            telemetry.update();
            sleep(20);
        }

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // =============================================================
            // GAMEPAD 1 — DRIVE AND INTAKE
            // =============================================================
            if (gamepad1.back) drivetrain.resetImuYaw();

            double y = applyDeadband(-gamepad1.left_stick_y);
            double x = applyDeadband(gamepad1.left_stick_x);
            double rx = applyDeadband(gamepad1.right_stick_x);

            boolean brake = gamepad1.left_trigger > GAMEPAD_TRIGGER_THRESHOLD;
            // ---------- Auto Align Toggle (GP1 X) ----------
                if (gamepad1.x && !lastX1) {
                    autoAlignEnabled = !autoAlignEnabled;

                    if (autoAlignEnabled) {
                        limelight.enableAutoAlign(); // switches to pipeline 3
                    }
                }
                lastX1 = gamepad1.x;

            // ---------- DRIVE (Robot-Centric + HEADING ALIGN ONLY) ----------
            double driveX = x;
            double driveY = y;
            double driveTurn = rx;

            if (autoAlignEnabled) {
                // ONLY rotate — no strafe, no forward correction
                driveTurn += limelight.getTurnCorrection(true);
            }

            drivetrain.driveRobotCentric(driveX, driveY, driveTurn, brake);

            boolean intakeTriggerNow = gamepad1.right_trigger > GAMEPAD_TRIGGER_THRESHOLD;
            boolean reversePressed = gamepad1.left_bumper;

            if (intakeTriggerNow && !lastIntakeTrigger) {
                intakeToggle = !intakeToggle;
                if (intakeToggle) {
                    sorterMode = SorterMode.INTAKE;
                    mechanisms.engageIntake(1.0, reversePressed);
                } else {
                    mechanisms.disengageIntake();
                }
            }
            lastIntakeTrigger = intakeTriggerNow;

            if (intakeToggle) mechanisms.engageIntake(1.0, reversePressed);

            // =============================================================
            // GAMEPAD 2 — MECHANISMS / SORTER
            // =============================================================

            // Rotate closest ball to top
            if (gamepad2.x && !lastX) {
                mechanisms.moveBallToTop(true); // GREEN
            }
            lastX = gamepad2.x;

            if (gamepad2.y && !lastY) {
                mechanisms.moveBallToTop(false); // PURPLE
            }
            lastY = gamepad2.y;

            // Outtake toggle
            if (gamepad2.b && !lastB) {
                outtakeOn = !outtakeOn;
                if (outtakeOn) mechanisms.engageOuttake(mechanisms.getManualOuttakeSpeed());
                else mechanisms.disengageOuttake();
            }
            lastB = gamepad2.b;

            // Outtake speed adjustments
            if (gamepad2.dpad_up && !lastDpadUp) mechanisms.increaseOuttakeSpeed(0.1);
            if (gamepad2.dpad_down && !lastDpadDown) mechanisms.decreaseOuttakeSpeed(0.1);
            lastDpadUp = gamepad2.dpad_up;
            lastDpadDown = gamepad2.dpad_down;

            // Ramp adjustments
            boolean rb2 = gamepad2.right_bumper;
            boolean lb2 = gamepad2.left_bumper;
            if (rb2 && !lastRB2) mechanisms.adjustOuttakeAngle(true, false);
            if (lb2 && !lastLB2) mechanisms.adjustOuttakeAngle(false, true);
            lastRB2 = rb2;
            lastLB2 = lb2;

            // Update mechanisms
            mechanisms.updateMechanisms();

            // =============================================================
            // TELEMETRY
            // =============================================================
            telemetry.addLine("---- LIMELIGHT AUTO ALIGN ----");
            telemetry.addData("Auto Align", autoAlignEnabled ? "ON" : "OFF");
            telemetry.addData("Pipeline", limelight.getCurrentPipeline());
            telemetry.addData("Lateral Err (m)", limelight.getLateralErrorMeters());
            telemetry.addData("Heading Err (deg)", limelight.getHeadingErrorDeg());
            telemetry.addData("Forward Dist (m)", limelight.getForwardDistanceMeters());
            telemetry.addData("Distance", limelight.getDistance());


            if (System.currentTimeMillis() - lastTelem > 100) {
                lastTelem = System.currentTimeMillis();
                telemetry.addData("Mode", sorterMode);
                telemetry.addData("Outtake Power", mechanisms.getManualOuttakeSpeed());
                telemetry.addData("Current Outtake Velocity", "%.0f t/s", mechanisms.outtakeMotor.getVelocity());
                telemetry.addData("Ramp Angle", "%.2f / %.2f",
                        mechanisms.getRampAngleCurrent(), mechanisms.getRampAngleTarget());
                telemetry.addData("Time", runtime.seconds());
                telemetry.update();
            } else {
                telemetry.update();
            }

            // Dashboard telemetry
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("sorter/isBusy", mechanisms.isSorterBusy());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }

    // ---------- Helpers ----------
    private double applyDeadband(double value) {
        return Math.abs(value) > DEADZONE ? value : 0;
    }
}
