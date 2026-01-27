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
    private int selectedPocket = 0;

    // ---------- Outtake ----------
    private boolean outtakeOn = false;
    private boolean lastB1 = false; // gamepad1
    private boolean lastB2 = false; // gamepad2

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
    private boolean lastBack1 = false;

    // ---------- Gamepad2 sorter buttons ----------
    private boolean lastX = false;
    private boolean lastY = false;

    // ---------- Auto Align ----------
    private boolean autoAlignEnabled = false;
    private boolean lastX1 = false;

    // ---------- Sorter Manual Controls ----------
    private boolean lastDpadLeft2 = false;
    private boolean lastDpadRight2 = false;
    private boolean lastDpadUp2 = false;
    private boolean lastDpadDown2 = false;

    private enum PrestartStage {
        DRIVE_MODE,
        ALLIANCE,
        CONFIRM
    }

    private PrestartStage prestartStage = PrestartStage.DRIVE_MODE;

    private boolean fieldCentric = false;


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

            telemetry.clearAll();

            switch (prestartStage) {

                // DRIVE MODE SELECTION
                case DRIVE_MODE:
                    telemetry.addLine("=== DRIVE MODE SELECT ===");
                    telemetry.addLine("DPAD UP   = FIELD-CENTRIC");
                    telemetry.addLine("DPAD DOWN = ROBOT-CENTRIC");
                    telemetry.addLine();
                    telemetry.addData("Selected", fieldCentric ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
                    telemetry.addLine();
                    telemetry.addLine("Press A to confirm");

                    if (gamepad1.dpad_up && !lastDpadUp) {
                        fieldCentric = true;
                    }
                    if (gamepad1.dpad_down && !lastDpadDown) {
                        fieldCentric = false;
                    }

                    if (gamepad1.a) {
                        prestartStage = PrestartStage.ALLIANCE;
                    }
                    break;

                // ALLIANCE SELECTION
                case ALLIANCE:
                    telemetry.addLine("=== ALLIANCE SELECT ===");
                    telemetry.addLine("X = BLUE  (Pipeline 9)");
                    telemetry.addLine("B = RED   (Pipeline 8)");
                    telemetry.addLine();

                    if (gamepad1.x && !lastX) {
                        isRedAlliance = false;
                        allianceChosen = true;
                    }
                    if (gamepad1.b && !lastB1) {
                        isRedAlliance = true;
                        allianceChosen = true;
                    }

                    telemetry.addData("Alliance",
                            allianceChosen ? (isRedAlliance ? "RED" : "BLUE") : "CHOOSE");

                    if (allianceChosen && !pipelineSet) {
                        limelight.setPipeline(isRedAlliance ? 8 : 9);
                        drivetrain.setAlliance(isRedAlliance);
                        pipelineSet = true;
                    }

                    telemetry.addLine();
                    telemetry.addLine("Press A to confirm");

                    if (gamepad1.a && allianceChosen) {
                        prestartStage = PrestartStage.CONFIRM;
                    }
                    break;

                // CONFIRMATION / RECAP
                case CONFIRM:
                    telemetry.addLine("=== SUMMARY ===");
                    telemetry.addLine();

                    telemetry.addData("Drive Mode",
                            fieldCentric ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
                    telemetry.addData("Alliance",
                            isRedAlliance ? "RED" : "BLUE");
                    telemetry.addData("Limelight Pipeline",
                            isRedAlliance ? "8 (RED)" : "9 (BLUE)");

                    telemetry.addLine();
                    telemetry.addLine("Waiting for START...");
                    telemetry.addLine("Press BACK to reconfigure");

                    if (gamepad1.back) {
                        prestartStage = PrestartStage.DRIVE_MODE;
                        allianceChosen = false;
                        pipelineSet = false;
                    }
                    break;
            }

            telemetry.update();

            // update debounce
            lastX = gamepad1.x;
            lastB1 = gamepad1.b;
            lastDpadUp = gamepad1.dpad_up;
            lastDpadDown = gamepad1.dpad_down;

            sleep(20);
        }

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // =============================================================
            // GAMEPAD 1 — DRIVE AND INTAKE
            // =============================================================
            if (gamepad1.back && !lastBack1) {
                drivetrain.resetImuYaw();
            }
            lastBack1 = gamepad1.back;

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

            if (fieldCentric) {
                drivetrain.driveFieldCentric(driveX, driveY, driveTurn, brake);
            } else {
                drivetrain.driveRobotCentric(driveX, driveY, driveTurn, brake);
            }

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
            if (gamepad2.left_trigger > 0.1) {
                mechanisms.moveBallToTop(true); // GREEN
            }
            lastX = gamepad2.x;

            if (gamepad2.right_trigger > 0.1) {
                mechanisms.moveBallToTop(false); // PURPLE
            }

            if (gamepad2.y) {
                mechanisms.ejectBall();
            }

            boolean dpadLeft  = gamepad2.dpad_left;
            boolean dpadRight = gamepad2.dpad_right;
            boolean dpadUp    = gamepad2.dpad_up;
            boolean dpadDown  = gamepad2.dpad_down;

            // Only allow manual commands if sorter is NOT moving
            if (!mechanisms.isSorterBusy()) {

                // Cycle pocket LEFT
                if (dpadLeft && !lastDpadLeft2) {
                    selectedPocket = (selectedPocket + 2) % 3; // -1 mod 3
                }

                // Cycle pocket RIGHT
                if (dpadRight && !lastDpadRight2) {
                    selectedPocket = (selectedPocket + 1) % 3;
                }

                // Move selected pocket to INTAKE
                if (gamepad2.a) {
                    mechanisms.sorter.movePocketToIntake(selectedPocket);
                }

                // Move selected pocket to OUTTAKE
                if (gamepad2.x) {
                    mechanisms.sorter.movePocketToOuttake(selectedPocket);
                }
            }

            // Update debounce states
            lastDpadLeft2  = dpadLeft;
            lastDpadRight2 = dpadRight;
            lastDpadUp2    = dpadUp;
            lastDpadDown2  = dpadDown;

            // Outtake toggle
            if (gamepad2.b && !lastB2) {
                outtakeOn = !outtakeOn;
                if (outtakeOn) mechanisms.engageOuttake(mechanisms.getManualOuttakeSpeed());
                else mechanisms.disengageOuttake();
            }
            lastB2 = gamepad2.b;

            // Outtake speed adjustments
            if (gamepad2.dpad_up && !lastDpadUp) mechanisms.increaseOuttakeSpeed(0.1);
            if (gamepad2.dpad_down && !lastDpadDown) mechanisms.decreaseOuttakeSpeed(0.1);
            lastDpadUp = gamepad2.dpad_up;
            lastDpadDown = gamepad2.dpad_down;

            // Ramp adjustments
            boolean rb2 = gamepad2.right_bumper;
            boolean lb2 = gamepad2.left_bumper;
            if (rb2 && !lastRB2) mechanisms.adjustOuttakeAngle(true);
            if (lb2 && !lastLB2) mechanisms.adjustOuttakeAngle(false);
            lastRB2 = rb2;
            lastLB2 = lb2;

            // Update mechanisms
            mechanisms.updateMechanisms();

            // =============================================================
            // TELEMETRY
            // =============================================================
            telemetry.addLine("---- BASIC DRIVETRAIN ----");
            telemetry.addData("Heading (deg)", drivetrain.getHeadingDeg());

            telemetry.addLine("---- LIMELIGHT AUTO ALIGN ----");
            telemetry.addData("Auto Align", autoAlignEnabled ? "ON" : "OFF");
            telemetry.addData("Pipeline", limelight.getCurrentPipeline());
            telemetry.addData("Lateral Err (m)", limelight.getLateralErrorMeters());
            telemetry.addData("Heading Err (deg)", limelight.getHeadingErrorDeg());
            telemetry.addData("Forward Dist (m)", limelight.getForwardDistanceMeters());
            telemetry.addData("Distance", limelight.getDistance());

            telemetry.addLine("---- OUTTAKE STATUS ----");
            telemetry.addData("Outtake Power", mechanisms.getManualOuttakeSpeed());
            telemetry.addData("Current Outtake Velocity", "%.0f t/s", mechanisms.outtakeMotor.getVelocity());
            telemetry.addData("Ramp Angle", "%.2f / %.2f", mechanisms.getRampAngleCurrent(), mechanisms.getRampAngleTarget());
            telemetry.addData("Time", runtime.seconds());

            telemetry.addLine("---- SORTER ----");
            telemetry.addData("Sorter Busy", mechanisms.isSorterBusy());
            telemetry.addData("Selected Pocket", selectedPocket);
            telemetry.addData("Intake Pocket", mechanisms.sorter.getPocketClosestTo(FinalSorter.INTAKE_TICKS));
            telemetry.addData("Outtake Pocket", mechanisms.sorter.getPocketClosestTo(FinalSorter.OUTTAKE_TICKS));
            telemetry.addData("Pending Ball", mechanisms.sorter.getPendingBall());

            FinalSorter.BallColor[] colors = mechanisms.sorter.getSlotColors();
            telemetry.addData("Slot 0", colors[0]);
            telemetry.addData("Slot 1", colors[1]);
            telemetry.addData("Slot 2", colors[2]);

            if (System.currentTimeMillis() - lastTelem > 100) {
                lastTelem = System.currentTimeMillis();
                telemetry.update();
            }
        }
    }

    // ---------- Helpers ----------
    private double applyDeadband(double value) {
        return Math.abs(value) > DEADZONE ? value : 0;
    }
}
