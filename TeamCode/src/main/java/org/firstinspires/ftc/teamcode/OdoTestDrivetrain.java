package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Odometry Test (Drivetrain Encoders)", group = "Test")
public class OdoTestDrivetrain extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // YOUR ACTUAL WIRING:
        DcMotorEx leftFront   = hardwareMap.get(DcMotorEx.class, "leftFront");   // ← right odo pod
        DcMotorEx rightFront  = hardwareMap.get(DcMotorEx.class, "rightFront");  // ← left odo pod
        DcMotorEx leftBack    = hardwareMap.get(DcMotorEx.class, "leftBack");    // ← center odo pod

        // Reset encoders
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        int lastLF = 0, lastRF = 0, lastLB = 0;

        while (opModeIsActive()) {

            int LF = leftFront.getCurrentPosition();
            int RF = rightFront.getCurrentPosition();
            int LB = leftBack.getCurrentPosition();

            int dLF = LF - lastLF;
            int dRF = RF - lastRF;
            int dLB = LB - lastLB;

            lastLF = LF;
            lastRF = RF;
            lastLB = LB;

            telemetry.addData("LEFT FRONT (Right Odo)",  "%d (Δ %d)", LF, dLF);
            telemetry.addData("RIGHT FRONT (Left Odo)",  "%d (Δ %d)", RF, dRF);
            telemetry.addData("LEFT BACK (Center Odo)",  "%d (Δ %d)", LB, dLB);

            // ---- Direction Classification ----

            if (Math.abs(dLF) > 5 || Math.abs(dRF) > 5) {
                if (dLF > 0 && dRF > 0)
                    telemetry.addLine("→ Robot MOVED FORWARD");
                if (dLF < 0 && dRF < 0)
                    telemetry.addLine("→ Robot MOVED BACKWARD");

                if (dLF > 0 && dRF < 0)
                    telemetry.addLine("→ Robot ROTATED LEFT (CCW)");
                if (dLF < 0 && dRF > 0)
                    telemetry.addLine("→ Robot ROTATED RIGHT (CW)");
            }

            if (Math.abs(dLB) > 5) {
                if (dLB > 0) telemetry.addLine("→ Robot STRAFED RIGHT");
                if (dLB < 0) telemetry.addLine("→ Robot STRAFED LEFT");
            }

            telemetry.update();
        }
    }
}
