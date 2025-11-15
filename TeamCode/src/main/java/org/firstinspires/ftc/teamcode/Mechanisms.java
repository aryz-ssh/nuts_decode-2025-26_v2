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
    public Servo intakeServoFirst;
    public Servo intakeServoSecond;
    private static final double TICKS_PER_REV_1150 = 145.1;
    private static final double MAX_TICKS_PER_SEC_1150 = (TICKS_PER_REV_1150 * 1150) / 60.0;

    // ---------- SORTER SYSTEM ----------
    public DcMotor sortingMotor;
    private static final int TICKS_PER_REV = 384;   // 435 RPM motor
    private static final int TICKS_120 = TICKS_PER_REV / 3;

    public NormalizedColorSensor bottomColorSensor;

    // ---------- OUTTAKE SYSTEM ----------
    public DcMotorEx outtakeMotorLeft;
    public DcMotorEx outtakeMotorRight;
    private static final double TICKS_PER_REV_6000 = 28.0;
    private static final double MAX_TICKS_PER_SEC_6000 = (TICKS_PER_REV_6000 * 6000) / 60.0;

    // ---------- TELEMETRY ----------
    private Telemetry telemetry;

    // ---------- MANUAL OUTTAKE ----------
    private double manualOuttakeSpeed = 0.7; // default speed

    // ---------- RACK-AND-PINION ----------
    public Servo rackServo;
    private static final double RACK_RETRACTED_POS = 0.0;
    private static final double RACK_PUSHED_POS = 1.0;

    // ---------- INIT ----------
    public void initMechanisms(HardwareMap hw, Telemetry telemetry) {
        this.telemetry = telemetry;
        initIntake(hw);
        initOuttake(hw);
        initSorter(hw);
        initRackAndPinion(hw);
    }

    private void initIntake(HardwareMap hw) {
        intakeMotor = hw.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intakeServoFirst = hw.get(Servo.class, "intakeServoFirst");
        intakeServoFirst.setPosition(0.5);

        intakeServoSecond = hw.get(Servo.class, "intakeServoSecond");
        intakeServoSecond.setDirection(Servo.Direction.FORWARD);
        intakeServoSecond.setPosition(0.5);
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

    private void initRackAndPinion(HardwareMap hw) {
        rackServo = hw.get(Servo.class, "rackServo");
        rackServo.setDirection(Servo.Direction.FORWARD);
        rackServo.setPosition(RACK_RETRACTED_POS); // initial retracted
    }

    // ---------- INTAKE ----------
    public void engageIntake(double power, boolean intakeDirectionFlip) {
        double direction = intakeDirectionFlip ? 1.0 : -1.0;

        double servo1Pos = 0.5 + (direction * power * 0.5);
        servo1Pos = Math.max(0.0, Math.min(1.0, servo1Pos));
        double servo2Pos = 0.5 - (direction * power * 0.5);
        servo2Pos = Math.max(0.0, Math.min(1.0, servo2Pos));

        intakeServoFirst.setPosition(servo1Pos);
        intakeServoSecond.setPosition(servo2Pos);

        intakeMotor.setVelocity(direction * power * MAX_TICKS_PER_SEC_1150);

        telemetry.addData("Intake Velocity", intakeMotor.getVelocity());
        telemetry.addData("Servo1 Pos", servo1Pos);
        telemetry.addData("Servo2 Pos", servo2Pos);
        telemetry.update();
    }

    public void disengageIntake() {
        intakeMotor.setPower(0.0);
        intakeServoFirst.setPosition(0.5);
        intakeServoSecond.setPosition(0.5);
    }

    // ---------- OUTTAKE ----------
    public void engageOuttake(double speed) {
        outtakeMotorLeft.setVelocity(speed * MAX_TICKS_PER_SEC_6000);
        outtakeMotorRight.setVelocity(speed * MAX_TICKS_PER_SEC_6000);

        telemetry.addData("Outtake Velocity", outtakeMotorLeft.getVelocity());
        telemetry.update();
    }

    public void disengageOuttake() {
        outtakeMotorLeft.setVelocity(0.0);
        outtakeMotorRight.setVelocity(0.0);

        telemetry.addData("Outtake Velocity", outtakeMotorLeft.getVelocity());
        telemetry.update();
    }

    // ---------- SHOOTER ----------
    public void fireLauncher(double speed) {
        outtakeMotorLeft.setVelocity(speed * MAX_TICKS_PER_SEC_6000);
        outtakeMotorRight.setVelocity(speed * MAX_TICKS_PER_SEC_6000);
    }

    public void stopLauncher() {
        outtakeMotorLeft.setVelocity(0);
        outtakeMotorRight.setVelocity(0);
    }

    // ---------- SORTER ----------
    public void rotateCarouselStep(double power) {
        String bottomColor = getBottomBallColor();
        if (!bottomColor.equals("unknown")) {
            int newTarget = sortingMotor.getCurrentPosition() + TICKS_120;
            sortingMotor.setTargetPosition(newTarget);
            sortingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sortingMotor.setPower(power);
        }
    }

    public String getBottomBallColor() {
        NormalizedRGBA colors = bottomColorSensor.getNormalizedColors();
        float r = colors.red / colors.alpha;
        float g = colors.green / colors.alpha;
        float b = colors.blue / colors.alpha;

        if (g > 0.05 && g > r && g > b) return "green";
        if (b > 0.1 && b > r && b > g) return "purple";
        return "unknown";
    }

    public void removeTopBall(double power) {
        int newTarget = sortingMotor.getCurrentPosition() + TICKS_120; // one step
        sortingMotor.setTargetPosition(newTarget);
        sortingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sortingMotor.setPower(power);

        telemetry.addData("Removing Top Ball", "Target Position: " + newTarget);
        telemetry.update();
    }

    // ---------- MANUAL OUTTAKE CONTROL ----------
    public void manualOuttake(boolean turnOn) {
        if (turnOn) {
            outtakeMotorLeft.setVelocity(manualOuttakeSpeed * MAX_TICKS_PER_SEC_6000);
            outtakeMotorRight.setVelocity(manualOuttakeSpeed * MAX_TICKS_PER_SEC_6000);
        } else {
            outtakeMotorLeft.setVelocity(0);
            outtakeMotorRight.setVelocity(0);
        }
    }

    public void increaseOuttakeSpeed(double delta) {
        manualOuttakeSpeed = Math.min(4.0, manualOuttakeSpeed + delta);
    }

    public void decreaseOuttakeSpeed(double delta) {
        manualOuttakeSpeed = Math.max(0.0, manualOuttakeSpeed - delta);
    }

    public double getManualOuttakeSpeed() {
        return manualOuttakeSpeed;
    }

    // ---------- RACK-AND-PINION ----------
    public void pushBall() {
        rackServo.setPosition(RACK_PUSHED_POS);
    }

    public void retractRack() {
        rackServo.setPosition(RACK_RETRACTED_POS);
    }
}
