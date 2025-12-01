package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Wheel Diagnostic Test", group="Debug")
public class WheelDiagnosticTest extends LinearOpMode {

    DcMotorEx frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft  = hardwareMap.get(DcMotorEx.class, "leftFront");
        backLeft   = hardwareMap.get(DcMotorEx.class, "leftBack");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backRight  = hardwareMap.get(DcMotorEx.class, "rightBack");

        // Use the SAME directions you use in TeleOp
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("READY — Press:");
        telemetry.addLine("  A → frontLeft");
        telemetry.addLine("  B → frontRight");
        telemetry.addLine("  X → backLeft");
        telemetry.addLine("  Y → backRight");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double fl = 0, fr = 0, bl = 0, br = 0;

            if (gamepad1.a) fl = 0.3;
            if (gamepad1.b) fr = 0.3;
            if (gamepad1.x) bl = 0.3;
            if (gamepad1.y) br = 0.3;

            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);

            telemetry.addData("frontLeft",  fl);
            telemetry.addData("frontRight", fr);
            telemetry.addData("backLeft",   bl);
            telemetry.addData("backRight",  br);
            telemetry.update();
        }
    }
}
