package org.firstinspires.ftc.teamcode.testers;
import org.firstinspires.ftc.teamcode.AprilTagLimelight;

import static org.firstinspires.ftc.teamcode.Mechanisms.RAMP_ANGLE_MIN_POS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Mechanisms;

@TeleOp(name = "Regression Test")
public class Regression extends LinearOpMode {

    public static double speed = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        boolean outtakeActive = false;
        double manualOuttakeSpeed = 0.7;
        double RAMP_ANGLE_MIN_POS = 0.24;
        double RAMP_ANGLE_MAX_POS = 0.99;
        double rampAngleTarget = RAMP_ANGLE_MIN_POS;
        final double RAMP_STEP_FRACTION = 0.20; // 20%
        final double RAMP_RANGE = RAMP_ANGLE_MAX_POS - RAMP_ANGLE_MIN_POS;
        final double RAMP_STEP = RAMP_RANGE * RAMP_STEP_FRACTION;
        boolean lastLB2 = false;
        boolean lastRB2 = false;


        Mechanisms mechanisms = new Mechanisms();
        AprilTagLimelight limelight = new AprilTagLimelight(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.right_trigger>0.1){
                mechanisms.engageOuttake(speed);
            }

            // Ramp adjustments
            boolean rb2 = gamepad2.right_bumper;
            boolean lb2 = gamepad2.left_bumper;
            if (rb2 && !lastRB2) mechanisms.adjustOuttakeAngle(true);
            if (lb2 && !lastLB2) mechanisms.adjustOuttakeAngle(false);
            lastRB2 = rb2;
            lastLB2 = lb2;

            telemetry.addData("CurrentSpeed", mechanisms.outtakeMotor.getVelocity());
            telemetry.addData("Distance", limelight.getDistance());
            telemetry.addData("Ramp Angle", "%.2f / %.2f", mechanisms.getRampAngleCurrent(), mechanisms.getRampAngleTarget());




        }
    }
}