package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@TeleOp(name = "TeleopMain")
public class TeleopMain extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private boolean intakeDirectionFlip = true;
    private double targetHeading = 0.0;

    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;

    // ===== SHOOTER SYSTEM ADDED =====
    private double speedX = 0.6;       // toggled speed
    private boolean lastRB = false;    // toggle debounce

    private double applyDeadband(double value) {
        return (Math.abs(value) > 0.05) ? value : 0;
    }

    // ---------- Mechanisms ----------
    private Mechanisms mechanisms;
    private TeleopDrivetrain drivetrain;

    private ArrayList<String> intakeOrder = new ArrayList<>(); // Current balls in carousel

    @Override
    public void runOpMode() throws InterruptedException {

// Initialize mechanisms
        mechanisms = new Mechanisms();
        mechanisms.initMechanisms(hardwareMap, telemetry);

// Initialize drivetrain correctly
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

            // ================================================================
            //                       DRIVE SYSTEM
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
            //                       INTAKE SYSTEM
            // ================================================================
            if(gamepad1.left_bumper) {
                intakeDirectionFlip = false;
            } else {
                intakeDirectionFlip = true;
            }

            if (gamepad1.left_trigger > 0.1) {
                mechanisms.engageIntake(gamepad1.left_trigger, intakeDirectionFlip);
            } else {
                mechanisms.disengageIntake();
            }


            // ================================================================
            //                       SORTER SYSTEM
            // ================================================================
            if (gamepad2.right_trigger > 0.1) {
                int steps = 1;
                mechanisms.rotateCarousel(steps, 0.5);

                if(gamepad2.right_bumper) {
                    mechanisms.engageOuttake(speedX);
                } else {
                    mechanisms.disengageOuttake();
                }
            }


            // ================================================================
            //         ===== SHOOTER SYSTEM ADDED (GAMEPAD 1) =====
            // ================================================================

            // ---- Toggle speedX using RB (0.6 ↔ 0.9) ----
            if (gamepad1.right_bumper && !lastRB) {
                speedX = (speedX == 0.6) ? 0.9 : 0.6;
            }
            lastRB = gamepad1.right_bumper;


            // ---- Fire launcher using RT ----
            if (gamepad1.right_trigger > 0.5) {
                mechanisms.fireLauncher(speedX);
            } else {
                mechanisms.stopLauncher();
            }


            // ================================================================
            //                       TELEMETRY
            // ================================================================
            telemetry.addData("IntakeOrder", intakeOrder.toString());
            telemetry.addData("Shooter SpeedX", speedX);
            telemetry.addData("Runtime", runtime.seconds());
            telemetry.update();
        }
    }

    private double applyDeadzone(double value, double threshold) {
        return Math.abs(value) < threshold ? 0 : value;
    }
}
