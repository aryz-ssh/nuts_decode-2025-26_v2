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
    private double shooterSpeed = 0.6; // toggle speed
    private boolean lastRB = false;    // debounce

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize mechanisms
        mechanisms = new Mechanisms();
        mechanisms.initMechanisms(hardwareMap, telemetry);

        // Initialize drivetrain
        drivetrain = new TeleopDrivetrain(this);
        drivetrain.initDriveTrain(hardwareMap);

        // Initialize intakeOrder (example)
        intakeOrder.add("red");
        intakeOrder.add("blue");
        intakeOrder.add("green");

        telemetry.addData("Status", "READY");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // ================================================================
            //                       GAMEPAD 1: DRIVE ONLY
            // ================================================================
            double y  = applyDeadband(gamepad1.left_stick_y);
            double x  = -applyDeadband(gamepad1.left_stick_x) * 1.1;
            double rx = -applyDeadband(gamepad1.right_stick_x);

            double fl = y + x + rx;
            double fr = y - x - rx;
            double bl = y - x + rx;
            double br = y + x - rx;

            double max = Math.max(1.0, Math.max(Math.abs(fl),
                    Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

            fl /= max; fr /= max; bl /= max; br /= max;

            drivetrain.updateDrive(fl * 0.8, fr * 0.8, bl * 0.8, br * 0.8);

            // ================================================================
            //                       GAMEPAD 2: MECHANISMS
            // ================================================================

            // ---------- INTAKE ----------
            if (gamepad2.left_bumper && !lastLeftBumper) {
                intakeDirectionFlip = !intakeDirectionFlip; // toggle on press
            }
            lastLeftBumper = gamepad2.left_bumper;

            if (gamepad2.left_trigger > 0.1) {
                mechanisms.engageIntake(gamepad2.left_trigger, intakeDirectionFlip);
            } else {
                mechanisms.disengageIntake();
            }

            // ---------- SORTER (carousel) ----------
            if (gamepad2.right_trigger > 0.1) {
                mechanisms.rotateCarouselStep(0.5); // continuous while holding
            } else if (mechanisms.sortingMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                mechanisms.sortingMotor.setPower(0);
                mechanisms.sortingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }


            // ---------- SHOOTER SPEED TOGGLE ---------

            // ---------- MANUAL OUTTAKE CONTROL ----------
            if (gamepad2.y) {
                mechanisms.manualOuttake(true);
            } else {
                mechanisms.manualOuttake(false);
            }

            if (gamepad2.b) {
                mechanisms.manualOuttake(true);
            } else {
                mechanisms.manualOuttake(false);
            }

            // Increase speed with D-pad right
            if (gamepad2.dpad_right) {
                mechanisms.increaseOuttakeSpeed(0.1);
            }
            // Decrease speed with D-pad left
            if (gamepad2.dpad_left) {
                mechanisms.decreaseOuttakeSpeed(0.1);
            }

            // ================================================================
            //                       TELEMETRY
            // ================================================================
            telemetry.addData("Runtime", runtime.seconds());
            telemetry.addData("Intake Order", intakeOrder.toString());
            telemetry.addData("Bottom Ball Color", mechanisms.getBottomBallColor());
            telemetry.addData("Intake Motor Velocity", mechanisms.intakeMotor.getVelocity());
            telemetry.addData("Sorting Motor Position", mechanisms.sortingMotor.getCurrentPosition());
            telemetry.addData("Outtake Velocity", mechanisms.outtakeMotorLeft.getVelocity());
            telemetry.addData("Manual Outtake Speed", mechanisms.getManualOuttakeSpeed());
            telemetry.addData("Shooter Speed", shooterSpeed);
            telemetry.update();
        }
    }

    // ---------------- HELPER ----------------
    private double applyDeadband(double value) {
        return (Math.abs(value) > 0.05) ? value : 0;
    }
}
