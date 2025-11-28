package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Mechanisms {

    // ---------- INTAKE SYSTEM ----------
    public DcMotorEx intakeMotor;
    public Servo intakeServoFirst;
    public Servo intakeServoSecond;
    private static final double TICKS_PER_REV_1150 = 145.1;
    private static final double MAX_TICKS_PER_SEC_1150 = (TICKS_PER_REV_1150 * 1150) / 60.0;

    // ---------- SORTER SYSTEM ----------
    public Servo sortingMotor;
    private static final int TICKS_PER_REV = 384;   // 435 RPM motor
    private static final double TICKS_PER_DEGREE = ((double) TICKS_PER_REV) / 360.0;

    // ---------- OUTTAKE SYSTEM ----------
    public DcMotorEx outtakeMotor;
    private static final double TICKS_PER_REV_6000 = 28.0;
    private static final double MAX_TICKS_PER_SEC_6000 = (TICKS_PER_REV_6000 * 6000) / 60.0;

    // ---------- TELEMETRY ----------
    private Telemetry telemetry;

    // ---------- MANUAL OUTTAKE ----------
    private double manualOuttakeSpeed = 0.7; // default speed

    // ---------- KICKER ----------
    public Servo kickerServo;
    private static final double RACK_RETRACTED_POS = 0.0;
    private static final double RACK_PUSHED_POS = 0.7;

    // ---------- INIT ----------
    public void initMechanisms(HardwareMap hw, Telemetry telemetry) {
        this.telemetry = telemetry;
        initIntake(hw);
        initOuttake(hw);
        initSorter(hw);
        initKicker(hw);
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
        outtakeMotor = hw.get(DcMotorEx.class, "outtakeMotor");
        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    private void initSorter(HardwareMap hw) {
        sortingMotor = hw.get(Servo.class, "intakeServoSecond");
        sortingMotor.setDirection(Servo.Direction.FORWARD);
        sortingMotor.setPosition(0.5);
    }

    private void initKicker(HardwareMap hw) {
        kickerServo = hw.get(Servo.class, "rackServo");
        kickerServo.setPosition(RACK_RETRACTED_POS);
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
        pushBall();
        outtakeMotor.setVelocity(speed * MAX_TICKS_PER_SEC_6000);

        telemetry.addData("Outtake Velocity", outtakeMotor.getVelocity());
        telemetry.update();
    }

    public void disengageOuttake() {
        retractRack();
        outtakeMotor.setVelocity(0.0);

        telemetry.addData("Outtake Velocity", outtakeMotor.getVelocity());
        telemetry.update();
    }

    // ---------- SHOOTER ----------
    public void fireLauncher(double speed) {
        outtakeMotor.setVelocity(speed * MAX_TICKS_PER_SEC_6000);
    }

    public void stopLauncher() {
        outtakeMotor.setVelocity(0);
    }

    // ---------- MANUAL OUTTAKE CONTROL ----------
    public void manualOuttake(float power) {
            outtakeMotor.setPower(power);
    }

    public void rackMove(boolean turnOn){
        if(turnOn) {
            pushBall();
        }
        else {
            retractRack();
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
        kickerServo.setPosition(RACK_PUSHED_POS);
    }

    public void retractRack() {
        kickerServo.setPosition(RACK_RETRACTED_POS);
    }
}
