package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Flywheel Max Velocity Test", group = "Test")
public class outtakemaxer extends OpMode {

    private DcMotorEx outtakeMotor;

    private boolean motorOn = false;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastY = false;

    private double maxVelocity = 0;

    @Override
    public void init() {
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");

        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("Flywheel Max Velocity Test Ready");
        telemetry.addLine("A = ON | B = OFF | Y = Reset Max");
        telemetry.update();
    }

    @Override
    public void loop() {

        // ---------- TOGGLE MOTOR ----------
        if (gamepad1.a && !lastA) {
            motorOn = true;
        }
        if (gamepad1.b && !lastB) {
            motorOn = false;
        }
        lastA = gamepad1.a;
        lastB = gamepad1.b;

        // ---------- RESET MAX ----------
        if (gamepad1.y && !lastY) {
            maxVelocity = 0;
        }
        lastY = gamepad1.y;

        // ---------- APPLY MAX POWER ----------
        if (motorOn) {
            outtakeMotor.setPower(1.0);   // RAW POWER, NOT VELOCITY
        } else {
            outtakeMotor.setPower(0);
        }

        // ---------- MEASURE ----------
        double currentVelocity = outtakeMotor.getVelocity();
        maxVelocity = Math.max(maxVelocity, currentVelocity);

        // ---------- TELEMETRY ----------
        telemetry.addData("Motor ON", motorOn);
        telemetry.addData("Current Velocity (ticks/s)", "%.0f", currentVelocity);
        telemetry.addData("MAX Velocity (ticks/s)", "%.0f", maxVelocity);
        telemetry.update();
    }
}
