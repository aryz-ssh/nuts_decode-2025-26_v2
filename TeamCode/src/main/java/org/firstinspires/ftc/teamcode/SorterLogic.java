package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
    TREAT THIS AS ITS OWN CLASS TO BE INITIALIZED IN "Mechanisms.java"!
    SORTING LOGIC WILL GET COMPLEX.
*/

public class SorterLogic {
    Telemetry telemetry;
    DcMotorEx sorterMotor;
    ColorSensor opticalSorterHoming;

    // State flags
    boolean sorterHomed = false;

    // Threshold for detecting reflective tape
    int HOME_THRESHOLD = 5000;

    // other things
    double HOMING_VELOCITY = 100;

    // --- Pocket positions (in encoder ticks) ---
    public int B1_INTAKE  = 0;
    public int B1_OUTTAKE = -259;

    public int B2_INTAKE  = 176;
    public int B2_OUTTAKE = -80;

    public int B3_INTAKE  = 361;
    public int B3_OUTTAKE = 640;

    // --- Motion control state ---
    int targetPos = 0;                // Encoder ticks target
    boolean moving = false;           // Whether carousel is rotating
    int POSITION_TOLERANCE = 5;       // acceptable error in ticks

    int FAST_VEL = 350;               // ticks/sec (main speed)
    int SLOW_VEL = 120;               // final approach speed
    int SLOW_ZONE = 40;               // when |error| < this, switch to slow

    public void init(HardwareMap hwMap, Telemetry tel) {
        this.telemetry = tel;

        sorterMotor = hwMap.get(DcMotorEx.class, "sorterMotor");
        opticalSorterHoming = hwMap.get(ColorSensor.class, "opticalSorterHoming");

        sorterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        sorterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sorterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void homeSorter() {
        if (sorterHomed) return; // Only home once

        int alpha = opticalSorterHoming.alpha();

        // allow free movement during homing
        sorterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // Move slowly clockwise
        sorterMotor.setVelocity(HOMING_VELOCITY);

        if (alpha > HOME_THRESHOLD) {
            // STOP IMMEDIATELY
            sorterMotor.setVelocity(0);

            // tiny delay to let motor settle
            try { Thread.sleep(20); } catch (Exception e) {}

            // REVERSE TO COUNTER OVERSHOOT
            // reverse at 60–100 ticks/sec (very low speed)
            sorterMotor.setVelocity(-80);

            // reverse for 40–60ms
            try { Thread.sleep(45); } catch (Exception e) {}

            // ensure complete stop
            sorterMotor.setVelocity(0);

            // tiny settle delay
            try { Thread.sleep(20); } catch (Exception e) {}

            // ZERO ENCODER IN FINAL POSITION
            sorterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            sorterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            // RESTORE BRAKE MODE
            sorterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            sorterHomed = true;
        }
    }

    public boolean isHomed() {
        return sorterHomed;
    }

    // VELOCITY-BASED MOVE COMMANDS
    public void goToPosition(int pos) {
        if (!sorterHomed) return; // safety
        targetPos = pos;
        moving = true;
    }

    public void goToIntake(int n) {
        if (n == 1) goToPosition(B1_INTAKE);
        if (n == 2) goToPosition(B2_INTAKE);
        if (n == 3) goToPosition(B3_INTAKE);
    }

    public void goToOuttake(int n) {
        if (n == 1) goToPosition(B1_OUTTAKE);
        if (n == 2) goToPosition(B2_OUTTAKE);
        if (n == 3) goToPosition(B3_OUTTAKE);
    }


    // UPDATE LOOP, call this every OpMode loop
    public void update() {
        if (!moving) return;

        int current = sorterMotor.getCurrentPosition();
        int error = targetPos - current;

        // When close, stop
        if (Math.abs(error) <= POSITION_TOLERANCE) {
            sorterMotor.setVelocity(0);
            moving = false;
            return;
        }

        // Decide velocity based on error sign
        int vel;

        if (Math.abs(error) < SLOW_ZONE)
            vel = SLOW_VEL;
        else
            vel = FAST_VEL;

        if (error < 0) vel *= -1;

        sorterMotor.setVelocity(vel);

        // Debug (optional)
        telemetry.addData("Target", targetPos);
        telemetry.addData("Pos", current);
        telemetry.addData("Err", error);
    }

    public void init_loop() {

        homeSorter();

        telemetry.addData("Homing", sorterHomed ? "DONE" : "HOMING...");
        telemetry.addData("Alpha", opticalSorterHoming.alpha());
        telemetry.addData("Encoder", sorterMotor.getCurrentPosition());
        telemetry.update();
    }

}
