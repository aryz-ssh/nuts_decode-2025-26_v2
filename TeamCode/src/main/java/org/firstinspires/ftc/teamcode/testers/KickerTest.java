package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "KickerTest", group = "Test")
public class KickerTest extends LinearOpMode {

    private Servo kicker;

    // Servo positions
    private final double REST_POS = 0.50;
    private final double KICK_POS = 0.83;

    @Override
    public void runOpMode() {

        kicker = hardwareMap.get(Servo.class, "kickerServo");

        // Always start in the safe position
        kicker.setPosition(REST_POS);

        telemetry.addLine("Kicker Test Ready");
        telemetry.addLine("Press A to kick");
        telemetry.addLine("Release A to return to rest");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                kicker.setPosition(KICK_POS);
            } else {
                kicker.setPosition(REST_POS);
            }

            telemetry.addData("Servo Position", kicker.getPosition());
            telemetry.update();
        }
    }
}
