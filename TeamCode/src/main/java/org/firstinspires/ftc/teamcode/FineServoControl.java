package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "FineServoControl", group = "Tools")
public class FineServoControl extends LinearOpMode {

    private Servo servo;
    private double servoPos = 0.5;

    // How fine the adjustments should be (smaller = finer)
    private final double STEP = 0.1;

    // For debouncing button presses
    private boolean upWasPressed = false;
    private boolean downWasPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {

        servo = hardwareMap.get(Servo.class, "rackServo");
        servo.setPosition(servoPos);

        waitForStart();

        while (opModeIsActive()) {

            // Increase position
            if (gamepad1.dpad_up && !upWasPressed) {
                servoPos += STEP;
                upWasPressed = true;
            }
            if (!gamepad1.dpad_up) upWasPressed = false;

            // Decrease position
            if (gamepad1.dpad_down && !downWasPressed) {
                servoPos -= STEP;
                downWasPressed = true;
            }
            if (!gamepad1.dpad_down) downWasPressed = false;

            // Clip to valid range
            servoPos = Math.max(0.0, Math.min(1.0, servoPos));

            // Apply to servo
            servo.setPosition(servoPos);

            // Telemetry
            telemetry.addData("Servo Position", "%.3f", servoPos);
            telemetry.update();
        }
    }
}
