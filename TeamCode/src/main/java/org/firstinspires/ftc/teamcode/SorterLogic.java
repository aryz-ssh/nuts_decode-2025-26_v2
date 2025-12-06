package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SorterLogic {

    // ---------- Hardware ----------
    private DcMotorEx sorterMotor;
    private ColorSensor homingSensor;
    private NormalizedColorSensor colorSensor;
    private Telemetry telemetry;

    // ---------- Sorter State ----------
    public enum DetectedColor { PURPLE, GREEN, UNKNOWN }
    private DetectedColor[] sorterSlots = {DetectedColor.UNKNOWN, DetectedColor.UNKNOWN, DetectedColor.UNKNOWN};
    // Index 0 = bottom, 1 = top-left, 2 = top-right

    private boolean sorterHomed = false;
    public int targetPos = 0;
    public boolean moving = false;

    private ElapsedTime updateTimer = new ElapsedTime();
    private static final double UPDATE_PERIOD_MS = 10.0;
    private static final int MAX_VEL = 600;

    // ---------- Encoder Positions ----------
    public static int B1_INTAKE = -50;
    public static int B2_INTAKE = 0;
    public static int B3_INTAKE = 50;

    public static int B1_OUTTAKE = 0;
    public static int B2_OUTTAKE = 100;
    public static int B3_OUTTAKE = -100;

    // ---------- INIT ----------
    public void init(HardwareMap hwMap, Telemetry tel) {
        telemetry = tel;
        sorterMotor = hwMap.get(DcMotorEx.class, "sorterMotor");
        homingSensor = hwMap.get(ColorSensor.class, "opticalSorterHoming");
        colorSensor = hwMap.get(NormalizedColorSensor.class, "sorterColorSensor");

        sorterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        sorterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sorterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    // ---------- LOOP INIT ----------
    public void init_loop() {
        // Can implement homing check or pre-run logic
        if (!sorterHomed) homeSorter();
    }

    // ---------- HOMING ----------
    public void homeSorter() {
        // Simplified homing logic
        sorterMotor.setVelocity(200); // spin slowly
        // TODO: detect homing sensor here
        sorterMotor.setVelocity(0);
        sorterHomed = true;
    }

    // ---------- GO TO POSITION ----------
    private void goToPosition(int pos) {
        targetPos = pos;
        moving = true;
    }

    public void goToIntake(int pocket) {
        switch (pocket) {
            case 1: goToPosition(B1_INTAKE); break;
            case 2: goToPosition(B2_INTAKE); break;
            case 3: goToPosition(B3_INTAKE); break;
        }
    }

    public void goToOuttake(int pocket) {
        switch (pocket) {
            case 1: goToPosition(B1_OUTTAKE); break;
            case 2: goToPosition(B2_OUTTAKE); break;
            case 3: goToPosition(B3_OUTTAKE); break;
        }
    }

    public void update() {
        if (!moving) return;
        if (updateTimer.milliseconds() < UPDATE_PERIOD_MS) return;
        updateTimer.reset();

        int current = sorterMotor.getCurrentPosition();
        int error = targetPos - current;

        if (Math.abs(error) < 5) { // tolerance
            sorterMotor.setVelocity(0);
            moving = false;
            return;
        }

        float vel = Math.signum(error) * MAX_VEL;
        sorterMotor.setVelocity(vel);
    }

    // ---------- BALL CONTROL ----------
    public void intakeBall(DetectedColor color) {
        // Find next empty slot
        int slot = -1;
        for (int i = 0; i < sorterSlots.length; i++) {
            if (sorterSlots[i] == DetectedColor.UNKNOWN) {
                slot = i;
                break;
            }
        }
        if (slot == -1) return; // carousel full
        sorterSlots[slot] = color;
    }

    public void movePurpleToTop() {
        moveBallToTop(DetectedColor.PURPLE);
    }

    public void moveGreenToTop() {
        moveBallToTop(DetectedColor.GREEN);
    }

    private void moveBallToTop(DetectedColor targetColor) {
        if (!sorterHomed) return;

        int targetSlot = -1;
        for (int i = 0; i < sorterSlots.length; i++) {
            if (sorterSlots[i] == targetColor) {
                targetSlot = i;
                break;
            }
        }
        if (targetSlot == -1) return; // ball not present

        int positionTicks = 0;
        switch (targetSlot) {
            case 0: positionTicks = B1_OUTTAKE; break;
            case 1: positionTicks = B2_OUTTAKE; break;
            case 2: positionTicks = B3_OUTTAKE; break;
        }

        goToPosition(positionTicks);

        // Update carousel state
        sorterSlots[targetSlot] = DetectedColor.UNKNOWN; // old slot empty
        sorterSlots[1] = targetColor; // place at top-left by default
    }

    // ---------- UTILITY ----------
    public DetectedColor[] getSorterSlots() {
        return sorterSlots.clone();
    }

    public boolean isBallPresent() {
        for (DetectedColor c : sorterSlots) {
            if (c != DetectedColor.UNKNOWN) return true;
        }
        return false;
    }

    public DetectedColor detectBallColor() {
        // Simplified detection: returns first detected ball
        for (DetectedColor c : sorterSlots) {
            if (c != DetectedColor.UNKNOWN) return c;
        }
        return DetectedColor.UNKNOWN;
    }

    public int getCurrentPos() {
        return sorterMotor.getCurrentPosition();
    }

    public int getTargetPos() {
        return targetPos;
    }
}
