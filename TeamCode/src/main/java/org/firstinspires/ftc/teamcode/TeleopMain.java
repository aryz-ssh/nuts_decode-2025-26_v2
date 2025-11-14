package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@TeleOp(name = "TeleopMain")
public class TeleopMain extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private boolean intakeDirectionFlip = true;

    // ---------- Mechanisms ----------
    private Mechanisms mechanisms;
    private ArrayList<String> intakeOrder = new ArrayList<>(); // Current balls in carousel

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize mechanisms
        mechanisms = new Mechanisms();
        mechanisms.initMechanisms(hardwareMap, telemetry);

        // Populate intake order for testing
        intakeOrder.add("red");
        intakeOrder.add("blue");
        intakeOrder.add("green");

        telemetry.addData("Status", "READY TO NUT");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // ---------------- Drivetrain Controls ----------------
            double y = applyDeadzone(-gamepad1.left_stick_y, 0.05);   // forward/back
            double x = applyDeadzone(gamepad1.left_stick_x, 0.05);    // strafe
            double rx = applyDeadzone(gamepad1.right_stick_x, 0.05);  // rotation

            // TODO: Replace with your drivetrain code
            // drivetrain.updateDrive(targetFL, targetFR, targetBL, targetBR);

            // ---------------- Intake Controls ----------------
            intakeDirectionFlip = !gamepad2.left_bumper;
            double intakePower = gamepad2.left_trigger;
            mechanisms.engageIntake(intakePower, intakeDirectionFlip);

            // ---------------- Outtake Controls ----------------
            if (gamepad2.right_bumper) {
                mechanisms.engageOuttake(1.0);
            } else {
                mechanisms.disengageOuttake();
            }

            // ---------------- Auto-Sorter using Bottom Color Sensor ----------------
            if (gamepad2.right_trigger > 0.1 && !intakeOrder.isEmpty()) {
                String bottomColor = mechanisms.getBottomBallColor();
                int steps = mechanisms.stepsToAlign(bottomColor, intakeOrder);

                // Rotate carousel to align the desired ball
                mechanisms.rotateCarousel(steps, 0.5);

                // Remove the ball after shooting
                mechanisms.removeTopBall(intakeOrder);

                telemetry.addData("Sorter", "Shot ball: %s", bottomColor);
            }

            telemetry.addData("IntakeOrder", intakeOrder.toString());
            telemetry.addData("Runtime", runtime.seconds());
            telemetry.update();
        }
    }

    // ---------------- Helper Functions ----------------
    private double applyDeadzone(double value, double threshold) {
        return Math.abs(value) < threshold ? 0 : value;
    }
}
