package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Outtake Tester", group = "Utility")
public class ServoOuttakeTester extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Change this to match your servo name in the configuration
        Servo servo = hardwareMap.get(Servo.class, "rampAngle");

        double pos = 0.5;        // Start centered
        double step = 0.01;      // Pressing D-Pad adjusts by this much

        servo.setPosition(pos);

        telemetry.addLine("Servo Tester Loaded");
        telemetry.addLine("Controls:");
        telemetry.addLine("D-Pad Left  = - Step");
        telemetry.addLine("D-Pad Right = + Step");
        telemetry.addLine("X = 0.0   |  B = 1.0");
        telemetry.addLine("A = 0.5 (center)");
        telemetry.addLine("----------------------------------");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Manual adjustments with D-Pad
            if (gamepad1.dpad_left) {
                pos -= step;
            }
            if (gamepad1.dpad_right) {
                pos += step;
            }

            // Quick jump positions
            if (gamepad1.x) {     // Full left
                pos = 0.0;
            }
            if (gamepad1.a) {     // Center
                pos = 0.5;
            }
            if (gamepad1.b) {     // Full right
                pos = 1.0;
            }

            // Clamp to valid range
            pos = Math.max(0.0, Math.min(1.0, pos));

            // Apply to servo
            servo.setPosition(pos);

            // Telemetry
            telemetry.addData("Servo Position", pos);
            telemetry.addData("Step Size", step);
            telemetry.update();

            sleep(20); // just to keep loop clean, optional
        }
    }
}
