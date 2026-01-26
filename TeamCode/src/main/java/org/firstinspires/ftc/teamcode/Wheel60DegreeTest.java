package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Sorter 60 Degree Encoder Test", group="Debug")
public class Wheel60DegreeTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx sorterMotor = hardwareMap.get(DcMotorEx.class, "sorterMotor");

        // Reset encoder to zero
        sorterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Sorter 60° Encoder Test Loaded");
        telemetry.addLine("Hold RIGHT BUMPER to rotate motor until wheel moves 60°");
        telemetry.addLine("Press A to reset encoder to 0");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Spin motor forward slowly while holding RB
            if (gamepad1.right_bumper) {
                sorterMotor.setPower(0.2);
            } else {
                sorterMotor.setPower(0);
            }

            // Reset encoder
            if (gamepad1.a) {
                sorterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                sorterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            // Show encoder ticks
            telemetry.addData("Encoder Ticks", sorterMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
