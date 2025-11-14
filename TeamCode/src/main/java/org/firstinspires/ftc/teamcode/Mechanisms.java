package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class Mechanisms {

    // ---------- INTAKE SYSTEM ----------
    public DcMotorEx intakeMotor;
    public Servo intakeServo;
    private static final double TICKS_PER_REV_1150 = 145.1;
    private static final double MAX_TICKS_PER_SEC_1150 = (TICKS_PER_REV_1150 * 1150) / 60.0; // ≈ 2786 t/s

    // ---------- SORTER SYSTEM ----------
    public DcMotor sortingMotor;
    private static final int TICKS_PER_REV = 384;   // 435 RPM motor
    private static final int TICKS_120 = TICKS_PER_REV / 3;
    private static final int TICKS_240 = TICKS_120 * 2;

    // Bottom color sensor
    public NormalizedColorSensor bottomColorSensor;

    // ---------- OUTTAKE SYSTEM ----------
    public DcMotorEx outtakeMotorLeft;
    public DcMotorEx outtakeMotorRight;
    private static final double TICKS_PER_REV_6000 = 28.0;
    private static final double MAX_TICKS_PER_SEC_6000 = (TICKS_PER_REV_6000 * 6000) / 60.0; // ≈ 2800 t/s

    // ---------- TELEMETRY ----------
    private Telemetry telemetry;

    // ---------- INIT METHODS ----------
    public void initMechanisms(HardwareMap hw, Telemetry telemetry) {
        this.telemetry = telemetry;
        initIntake(hw);
        initOuttake(hw);
        initSorter(hw);
    }

    private void initIntake(HardwareMap hw) {
        intakeMotor = hw.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intakeServo = hw.get(Servo.class, "intakeServo");
        intakeServo.setPosition(0.5);
    }

    private void initOuttake(HardwareMap hw) {
        outtakeMotorLeft = hw.get(DcMotorEx.class, "outtakeMotorLeft");
        outtakeMotorRight = hw.get(DcMotorEx.class, "outtakeMotorRight");

        outtakeMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        outtakeMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        outtakeMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initSorter(HardwareMap hw) {
        sortingMotor = hw.get(DcMotor.class, "sortingMotor");
        sortingMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        sortingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sortingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sortingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bottomColorSensor = hw.get(NormalizedColorSensor.class, "colorSensor");
    }

    // ---------- INTAKE METHODS ----------
    public void engageIntake(double power, boolean intakeDirectionFlip) {
        double direction = intakeDirectionFlip ? 1.0 : -1.0;
        double servoPosition = 0.5 + (direction * power * 0.5);
        servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));

        intakeServo.setPosition(servoPosition);
        intakeMotor.setVelocity(direction * power * MAX_TICKS_PER_SEC_1150);
    }

    public void disengageIntake() {
        intakeMotor.setPower(0.0);
        intakeServo.setPosition(0.5);
    }

    // ---------- OUTTAKE METHODS ----------
    public void engageOuttake(double outtakeSpeed) {
        outtakeMotorLeft.setVelocity(outtakeSpeed * MAX_TICKS_PER_SEC_6000);
        outtakeMotorRight.setVelocity(outtakeSpeed * MAX_TICKS_PER_SEC_6000);

        telemetry.addData("Outtake Velocity", outtakeMotorLeft.getVelocity());
        telemetry.update();
    }

    public void disengageOuttake() {
        outtakeMotorLeft.setVelocity(0.0);
        outtakeMotorRight.setVelocity(0.0);

        telemetry.addData("Outtake Velocity", outtakeMotorLeft.getVelocity());
        telemetry.update();
    }

    // ---------- SORTER METHODS ----------
    public void rotateCarousel(int steps, double power) {
        int targetTicks = 0;
        switch (steps) {
            case 0: return;
            case 1: targetTicks = TICKS_120; break;
            case 2: targetTicks = TICKS_240; break;
        }

        int newTarget = sortingMotor.getCurrentPosition() + targetTicks;
        sortingMotor.setTargetPosition(newTarget);
        sortingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sortingMotor.setPower(power);

        while (sortingMotor.isBusy()) { /* optional telemetry */ }

        sortingMotor.setPower(0);
        sortingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int stepsToAlign(String desiredColor, ArrayList<String> intakeOrder) {
        if (intakeOrder.get(0).equals(desiredColor)) return 0;
        else if (intakeOrder.get(1).equals(desiredColor)) return 1;
        else return 2;
    }

    public void removeTopBall(ArrayList<String> intakeOrder) {
        if (!intakeOrder.isEmpty()) intakeOrder.remove(0);
    }

    // ---------- COLOR SENSOR METHOD ----------
    public String getBottomBallColor() {
        NormalizedRGBA colors = bottomColorSensor.getNormalizedColors();
        float r = colors.red / colors.alpha;
        float g = colors.green / colors.alpha;
        float b = colors.blue / colors.alpha;

        if (g > 0.05 && g > r && g > b) return "green";
        if (b > 0.1 && b > r && b > g) return "purple";

        return "unknown";
    }
}
