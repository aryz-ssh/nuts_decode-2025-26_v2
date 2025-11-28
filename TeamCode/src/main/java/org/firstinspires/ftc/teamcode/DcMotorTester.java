package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "DC Motor Adjustable-Speed Test", group = "Test")
public class DcMotorTester extends LinearOpMode {

    private DcMotorEx motor;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- CHANGE NAME TO MATCH YOUR CONFIG ---
        motor = hardwareMap.get(DcMotorEx.class, "testMotor");
        // -----------------------------------------

        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        double power = 0.10;  // starting test power

        telemetry.addLine("DC Motor Adjustable-Speed Test");
        telemetry.addLine("A = forward");
        telemetry.addLine("B = backward");
        telemetry.addLine("X = stop");
        telemetry.addLine("Dpad Up/Down = adjust speed");
        telemetry.addData("Initial Power", power);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // -------- Speed Adjustment --------
            if (gamepad1.dpad_up) {
                power += 0.01;
                if (power > 1.0) power = 1.0;
            }
            if (gamepad1.dpad_down) {
                power -= 0.01;
                if (power < 0.0) power = 0.0;
            }

            // -------- Motor Control Buttons --------
            if (gamepad1.a) {
                motor.setPower(power);     // forward
            } else if (gamepad1.b) {
                motor.setPower(-power);    // backward
            } else if (gamepad1.x) {
                motor.setPower(0);         // stop
            }

            // -------- Telemetry Display --------
            telemetry.addData("Motor Power", motor.getPower());
            telemetry.addData("Selected Speed", power);
            telemetry.addData("Encoder Position", motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
