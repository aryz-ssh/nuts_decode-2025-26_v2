package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Odo Pod Identifier", group = "Diagnostics")
public class OdoPodChecker extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Try to map all 4 possible motor ports (0–3)
        DcMotor motor0 = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor motor1 = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor motor2 = hardwareMap.get(DcMotor.class, "rightBack");
        DcMotor motor3 = hardwareMap.get(DcMotor.class, "leftBack");

        // Ensure zero power behavior is safe (not required but good practice)
        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Reset encoders (optional)
        motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("ODOMETRY IDENTIFIER");
        telemetry.addLine("Move each odometry pod and watch encoder values change.");
        telemetry.addLine("Press PLAY to begin...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("---- Live Encoder Values ----");
            telemetry.addData("Port 0 (motor0)", motor0.getCurrentPosition());
            telemetry.addData("Port 1 (motor1)", motor1.getCurrentPosition());
            telemetry.addData("Port 2 (motor2)", motor2.getCurrentPosition());
            telemetry.addData("Port 3 (motor3)", motor3.getCurrentPosition());
            telemetry.update();
        }
    }
}
