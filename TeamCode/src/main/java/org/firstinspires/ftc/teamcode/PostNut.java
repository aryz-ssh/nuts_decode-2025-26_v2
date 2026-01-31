package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "PostNut", group = "0")
public class PostNut extends LinearOpMode {

    private static final double DEADZONE = 0.05;
    private static final double TRIGGER_THRESH = 0.25;

    private ElapsedTime runtime = new ElapsedTime();

    // ---------- Mechanisms ----------
    private Mechanisms mechanisms;
    private MasterDrivetrain drivetrain;
    private AprilTagLimelight limelight;

    // ---------- Intake ----------
    private boolean intakeToggle = false;
    private boolean lastIntakeTrigger = false;

    // ---------- Sorter ----------
    private enum SorterMode { INTAKE, OUTTAKE }
    private SorterMode sorterMode = SorterMode.INTAKE;
    private int selectedPocket = 0;

    // ---------- Debounce ----------
    private boolean lastBack1 = false;
    private boolean lastB2 = false;
    private boolean lastDpadLeft2 = false;
    private boolean lastDpadRight2 = false;

    @Override
    public void runOpMode() {

        // ================= INIT =================
        mechanisms = new Mechanisms();
        mechanisms.initMechanisms(hardwareMap, telemetry, true);

        drivetrain = new MasterDrivetrain();
        drivetrain.init(hardwareMap);

        limelight = new AprilTagLimelight(hardwareMap);

        // 🔒 FORCE KNOWN SORTER STATE (NO GUESSING)
        selectedPocket = 0;
        sorterMode = SorterMode.INTAKE;
        mechanisms.sorter.movePocketToIntake(0);

        telemetry.addLine("Initialized — Sorter locked to INTAKE pocket 0");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // ================= LOOP =================
        while (opModeIsActive()) {

            // ---------- DRIVE ----------
            if (gamepad1.back && !lastBack1) {
                drivetrain.resetImuYaw();
            }
            lastBack1 = gamepad1.back;

            double y = deadband(-gamepad1.left_stick_y);
            double x = deadband(gamepad1.left_stick_x);
            double rx = deadband(gamepad1.right_stick_x);

            drivetrain.driveRobotCentric(x, y, rx, false);

            // ---------- INTAKE TOGGLE ----------
            boolean intakeTriggerNow = gamepad1.right_trigger > TRIGGER_THRESH;
            boolean reverse = gamepad1.left_bumper;

            if (intakeTriggerNow && !lastIntakeTrigger) {
                intakeToggle = !intakeToggle;
                if (intakeToggle) {
                    sorterMode = SorterMode.INTAKE;
                    mechanisms.engageIntake(1.0, reverse);
                } else {
                    mechanisms.disengageIntake();
                }
            }
            lastIntakeTrigger = intakeTriggerNow;

            if (intakeToggle) {
                mechanisms.engageIntake(1.0, reverse);
            }

            // ---------- SORTER MODE ----------
            if (gamepad2.a) {
                sorterMode = SorterMode.INTAKE;
                mechanisms.sorter.movePocketToIntake(selectedPocket);
            }

            if (gamepad2.x) {
                sorterMode = SorterMode.OUTTAKE;
                mechanisms.sorter.movePocketToOuttake(selectedPocket);
            }

            // ---------- MANUAL POCKET STEP ----------
            boolean dpadLeft  = gamepad2.dpad_left;
            boolean dpadRight = gamepad2.dpad_right;

            if (!mechanisms.isSorterBusy()) {

                boolean moved = false;

                if (dpadRight && !lastDpadRight2) {
                    selectedPocket = (selectedPocket + 1) % 3;
                    moved = true;
                }

                if (dpadLeft && !lastDpadLeft2) {
                    selectedPocket = (selectedPocket + 2) % 3;
                    moved = true;
                }

                if (moved) {
                    if (sorterMode == SorterMode.INTAKE) {
                        mechanisms.sorter.movePocketToIntake(selectedPocket);
                    } else {
                        mechanisms.sorter.movePocketToOuttake(selectedPocket);
                    }
                }
            }

            lastDpadLeft2 = dpadLeft;
            lastDpadRight2 = dpadRight;

            // ---------- OUTTAKE TOGGLE ----------
            if (gamepad2.b && !lastB2) {
                mechanisms.toggleOuttake();
            }
            lastB2 = gamepad2.b;

            // ---------- UPDATE ----------
            mechanisms.updateMechanisms();

            // ---------- TELEMETRY (NO PLANE GUESSING) ----------
            telemetry.addData("Runtime", "%.1f", runtime.seconds());
            telemetry.addData("Sorter Busy", mechanisms.isSorterBusy());
            telemetry.addData("Sorter Mode", sorterMode);
            telemetry.addData("Selected Pocket", selectedPocket);

            FinalSorter.BallColor[] colors = mechanisms.sorter.getSlotColors();
            telemetry.addData("Slot 0", colors[0]);
            telemetry.addData("Slot 1", colors[1]);
            telemetry.addData("Slot 2", colors[2]);

            telemetry.update();
        }
    }

    private double deadband(double v) {
        return Math.abs(v) > DEADZONE ? v : 0.0;
    }
}
