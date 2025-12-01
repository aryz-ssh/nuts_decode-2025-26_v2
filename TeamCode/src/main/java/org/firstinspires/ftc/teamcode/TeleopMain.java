package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleopMain")
public class TeleopMain extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    long lastTelem = 0;

    // ---------- Mechanisms ----------
    private Mechanisms mechanisms;
    private TeleopDrivetrain drivetrain;

    // ---------- Intake Direction ----------
    private boolean intakeDirectionFlip = true;
    private boolean lastLeftBumper = false;

    // ---------- Sorter Variables ----------
    private int selectedPocket = 1;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastA = false;
    private boolean lastX = false;

    // ---------- Outtake ----------
    private boolean outtakeOn = false;
    private boolean lastB = false;

    // ---------- Ramp angle debounce ----------
    private boolean lastRB2 = false;
    private boolean lastLB2 = false;

    // ---------- Outtake debounce -------------
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;


    @Override
    public void runOpMode() throws InterruptedException {

        mechanisms = new Mechanisms();
        mechanisms.initMechanisms(hardwareMap, telemetry);

        drivetrain = new TeleopDrivetrain(this);

        drivetrain.initDriveTrain(hardwareMap);

        telemetry.addData("Status", "HOMING SORTER…");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            mechanisms.sorterInitLoop();
            telemetry.update();
            sleep(5);
        }

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // =============================================================
            //                GAMEPAD 1 — DRIVE AND INTAKE
            // =============================================================

            double y  = applyDeadband(-gamepad1.left_stick_y);
            double x  = applyDeadband(gamepad1.left_stick_x);
            double rx = applyDeadband(gamepad1.right_stick_x);

            // ✔ ORIGINAL ROBOT-CENTRIC MIXING
            double fl = y + x + rx;
            double fr = y - x - rx;
            double bl = y - x + rx;
            double br = y + x - rx;

            drivetrain.updateDrive(fl, fr, bl, br);

            // ---------- INTAKE ----------
            if (gamepad1.left_bumper && !lastLeftBumper) {
                intakeDirectionFlip = !intakeDirectionFlip;
            }
            lastLeftBumper = gamepad1.left_bumper;

            // --- INTAKE FULL SPEED ON ANY TRIGGER PRESS ---
            if (gamepad1.right_trigger > 0.05) {
                mechanisms.engageIntake(1.0, intakeDirectionFlip); // ← full speed
            } else {
                mechanisms.disengageIntake();
            }

            // pusher
            mechanisms.pushBallToSorter(gamepad1.right_bumper);


            // =============================================================
            //                       GAMEPAD 2 — MECHANISMS
            // =============================================================

            if (gamepad2.dpad_right && !lastDpadRight) {
                selectedPocket++;
                if (selectedPocket > 3) selectedPocket = 1;
            }
            if (gamepad2.dpad_left && !lastDpadLeft) {
                selectedPocket--;
                if (selectedPocket < 1) selectedPocket = 3;
            }

            lastDpadRight = gamepad2.dpad_right;
            lastDpadLeft  = gamepad2.dpad_left;

            if (gamepad2.a && !lastA)
                mechanisms.sorterGoToIntake(selectedPocket);
            lastA = gamepad2.a;

            if (gamepad2.x && !lastX)
                mechanisms.sorterGoToOuttake(selectedPocket);
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


            if (gamepad2.y)
                mechanisms.ejectBall();

            boolean rb2 = gamepad2.right_bumper;
            boolean lb2 = gamepad2.left_bumper;

            if (rb2 && !lastRB2)
                mechanisms.adjustOuttakeAngle(true, false);

            if (lb2 && !lastLB2)
                mechanisms.adjustOuttakeAngle(false, true);

            lastRB2 = rb2;
            lastLB2 = lb2;


            // =============================================================
            //                        UPDATE MECHANISMS
            // =============================================================
            mechanisms.updateMechanisms();


            // =============================================================
            //                          TELEMETRY
            // =============================================================
            if (System.currentTimeMillis() - lastTelem > 100) {
                telemetry.addData("Runtime", runtime.seconds());
                telemetry.addData("Outtake Speed", mechanisms.getManualOuttakeSpeed());
                telemetry.addData("Ramp Angle", "%.2f / %.2f",
                        mechanisms.getRampAngleCurrent(),
                        mechanisms.getRampAngleTarget());

                telemetry.addData("Sorter Pos", "%d → %d",
                        mechanisms.getSorterCurrentPosition(),
                        mechanisms.getSorterTargetPosition());

                telemetry.addData("Sorter Error", mechanisms.getSorterTargetPosition()
                        - mechanisms.getSorterCurrentPosition());

                telemetry.addData("Ball Present", mechanisms.sorterBallPresent());
                telemetry.addData("Ball Color", mechanisms.sorterDetectColor());

                telemetry.update();
                lastTelem = System.currentTimeMillis();
            }
        }
    }

    private double applyDeadband(double value) {
        return (Math.abs(value) > 0.05) ? value : 0;
    }
}
