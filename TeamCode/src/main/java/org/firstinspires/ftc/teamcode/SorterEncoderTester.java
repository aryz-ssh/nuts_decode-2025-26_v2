package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Sorter Encoder Tester", group="Debug")
public class SorterEncoderTester extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx sorterMotor = hardwareMap.get(DcMotorEx.class, "sorterMotor");

        sorterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Sorter Encoder Tester Loaded");
        telemetry.addLine("Press A to spin forward");
        telemetry.addLine("Press B to spin reverse");
        telemetry.addLine("Press X to stop");
        telemetry.addLine("Encoder will print every loop");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                sorterMotor.setPower(0.3);
            } else if (gamepad1.b) {
                sorterMotor.setPower(-0.3);
            } else if (gamepad1.x) {
                sorterMotor.setPower(0);
            }

            telemetry.addData("Power", sorterMotor.getPower());
            telemetry.addData("Encoder", sorterMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
