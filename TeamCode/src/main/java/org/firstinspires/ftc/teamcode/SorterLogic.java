package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.testers.ColorSensorTester;

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

        ColorSensorTester.DetectedColor detectedColor = new ColorSensorTester();
    }

    // ---------- LOOP INIT ----------
    public void init_loop() {
        if (!sorterHomed) homeSorter();
    }

    // ---------- HOMING ----------
    public void homeSorter() {
        sorterMotor.setVelocity(200);
        // TODO: implement homing sensor logic here
        sorterMotor.setVelocity(0);
        sorterHomed = true;
    }

    // ---------- POSITION CONTROL ----------
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

        if (Math.abs(error) < 5) {
            sorterMotor.setVelocity(0);
            moving = false;
            return;
        }

        float vel = Math.signum(error) * MAX_VEL;
        sorterMotor.setVelocity(vel);
    }

    // ---------- COLOR SENSOR ----------
    public ColorSensorTester.DetectedColor getDetectedColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float normRed = colors.red / colors.alpha;
        float normGreen = colors.green / colors.alpha;
        float normBlue = colors.blue / colors.alpha;

        telemetry.addData("Red", normRed);
        telemetry.addData("Green", normGreen);
        telemetry.addData("Blue", normBlue);

        if (normGreen >= 0.0030 && normGreen <= 0.0055 && normGreen >= 0.01 && normGreen <= 0.0130 && normBlue >= 0.0080 && normBlue <= 0.015) {
            telemetry.addData("Color detected", "GREEN");
            return ColorSensorTester.DetectedColor.GREEN;
            //do 2nd thresh
        } else if (normGreen >= 0.0050 && normGreen <= 0.0055 && normGreen >= 0.01 && normGreen <= 0.0130 && normBlue >= 0.0080 && normBlue <= 0.015) {
            telemetry.addData("Color detected", "PURPLE");
            return ColorSensorTester.DetectedColor.PURPLE;
        }

        return ColorSensorTester.DetectedColor.UNKNOWN;
    }

    // ---------- BALL CONTROL ----------
    public void intakeBall() {

        if (detectedColor == DetectedColor.UNKNOWN) return;

        int slot = -1;
        for (int i = 0; i < sorterSlots.length; i++) {
            if (sorterSlots[i] == DetectedColor.UNKNOWN) {
                slot = i;
                break;
            }
        }
        if (slot == -1) return;

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
        if (targetSlot == -1) return;

        int positionTicks = 0;
        switch (targetSlot) {
            case 0: positionTicks = B1_OUTTAKE; break;
            case 1: positionTicks = B2_OUTTAKE; break;
            case 2: positionTicks = B3_OUTTAKE; break;
        }

        goToPosition(positionTicks);

        sorterSlots[targetSlot] = DetectedColor.UNKNOWN; // old slot empty
        sorterSlots[1] = targetColor; // top-left by default
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
        return getDetectedColor();
    }

    public int getCurrentPos() {
        return sorterMotor.getCurrentPosition();
    }

    public int getTargetPos() {
        return targetPos;
    }

    public boolean isMoving() {
        return moving;
    }
}
