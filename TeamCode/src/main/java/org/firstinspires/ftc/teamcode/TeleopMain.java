package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@TeleOp(name = "TeleopMain")
public class TeleopMain extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // ---------- Mechanisms ----------
    private Mechanisms mechanisms;
    private TeleopDrivetrain drivetrain;
    private ArrayList<String> intakeOrder = new ArrayList<>();

    // ---------- Intake Direction ----------
    private boolean intakeDirectionFlip = true;
    private boolean lastLeftBumper = false;

    // ---------- Shooter Speed ----------
    private double shooterSpeed = 0.6;
    private boolean lastRB = false;

    // ---------- SORTER DEBOUNCE ----------
    private boolean sorterTriggerPressed = false;

    // ---------- Drive Ramping ----------
    private double lastFL = 0, lastFR = 0, lastBL = 0, lastBR = 0;
    private static final double RAMP_RATE = 0.15;  // 0–1 change per loop

    @Override
    public void runOpMode() throws InterruptedException {

        mechanisms = new Mechanisms();
        mechanisms.initMechanisms(hardwareMap, telemetry);

        drivetrain = new TeleopDrivetrain(this);
        drivetrain.initDriveTrain(hardwareMap);

        intakeOrder.add("red");
        intakeOrder.add("blue");
        intakeOrder.add("green");

        telemetry.addData("Status", "READY");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // =============================================================
            //                     GAMEPAD 1 — DRIVE
            // =============================================================
            double y  = applyDeadband(gamepad1.left_stick_y);
            double x  = applyDeadband(gamepad1.left_stick_x);
            double rx = -applyDeadband(gamepad1.right_stick_x);

            double fl = y + x + rx;
            double fr = y - x - rx;
            double bl = y - x + rx;
            double br = y + x - rx;

            // normalize only if needed
            double max = Math.max(1.0, Math.max(Math.abs(fl),
                    Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
            fl /= max; fr /= max; bl /= max; br /= max;

            double speedMultiplier = 1.0 ;

            fl *= speedMultiplier;
            fr *= speedMultiplier;
            bl *= speedMultiplier;
            br *= speedMultiplier;

            // =============================================================
            //                    SMOOTH ACCELERATION (RAMP)
            // =============================================================
            fl = ramp(fl, lastFL); lastFL = fl;
            fr = ramp(fr, lastFR); lastFR = fr;
            bl = ramp(bl, lastBL); lastBL = bl;
            br = ramp(br, lastBR); lastBR = br;

            drivetrain.updateDrive(fl, fr, bl, br);

            // =============================================================
            //                       GAMEPAD 2 — MECHANISMS
            // =============================================================

            // ---------- INTAKE ----------
            if (gamepad2.left_bumper && !lastLeftBumper) {
                intakeDirectionFlip = !intakeDirectionFlip;
            }
            lastLeftBumper = gamepad2.left_bumper;

            if (gamepad2.left_trigger > 0.1) {
                mechanisms.engageIntake(gamepad2.left_trigger, intakeDirectionFlip);
            } else {
                mechanisms.disengageIntake();
            }

            // ---------- SORTER — single press rotates 120° with X button ----------
/*            if (gamepad2.x && !sorterTriggerPressed) {
                mechanisms.rotateCarouselStep(0.5);  // rotate 120°
                sorterTriggerPressed = true;
            }

            if (!gamepad2.x) {
                sorterTriggerPressed = false;
            }

            // Stop motor automatically once it reaches target
            if (mechanisms.sortingMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION &&
                    !mechanisms.sortingMotor.isBusy()) {
                mechanisms.sortingMotor.setPower(0);
                mechanisms.sortingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }*/

         //------- MANUAL OUTTAKE ----------
            if (gamepad2.right_trigger > 0.1) {
                mechanisms.manualOuttake(gamepad2.right_trigger);
            } else {
                mechanisms.manualOuttake(0);
            }

            if (gamepad2.y) {
                mechanisms.rackMove(true);
            }
            else {
                mechanisms.rackMove(false);
            }

            if (gamepad2.dpad_right) mechanisms.increaseOuttakeSpeed(0.1);
            if (gamepad2.dpad_left) mechanisms.decreaseOuttakeSpeed(0.1);

            // ---------- TELEMETRY ----------
            telemetry.addData("Runtime", runtime.seconds());
            telemetry.addData("Drive Multiplier", speedMultiplier);
            // telemetry.addData("Sorting Motor Position", mechanisms.sortingMotor.getCurrentPosition());
            telemetry.addData("Outtake Speed", mechanisms.getManualOuttakeSpeed());
            telemetry.update();
        }
    }

    // ---------------- HELPER ----------------
    private double applyDeadband(double value) {
        return (Math.abs(value) > 0.05) ? value : 0;
    }

    private double ramp(double target, double current) {
        double delta = target - current;
        if (Math.abs(delta) > RAMP_RATE) {
            return current + Math.signum(delta) * RAMP_RATE;
        }
        return target;
    }
}

//shlokk