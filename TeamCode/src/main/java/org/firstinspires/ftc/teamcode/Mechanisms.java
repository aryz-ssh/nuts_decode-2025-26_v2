package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Mechanisms {

    // --------- SORTER ----------
    public SorterLogic sorterLogic;
    private ElapsedTime sorterRate = new ElapsedTime();
    private static final double SORTER_PERIOD_MS = 60;

    // ---------- INTAKE SYSTEM ----------
    public DcMotorEx intakeMotor;
    public Servo intakeServoFirst;
    public Servo intakeServoSecond;
    private static final double INTAKE_SERVO_SECOND_RESTING_POS = 0.9;
    private static final double INTAKE_SERVO_SECOND_PUSH_POS = 0.17;

    // ---------- OUTTAKE SYSTEM ----------
    public DcMotorEx outtakeMotor;
    private static final double MAX_TICKS_PER_SEC_6000 = 28.0 * 6000 / 60.0;
    private double manualOuttakeSpeed = 0.7;
    public Servo rampAngleAdjust;
    private static final double RAMP_ANGLE_MIN_POS = 0.07;
    private static final double RAMP_ANGLE_MAX_POS = 0.7;
    private double rampAngleTarget = RAMP_ANGLE_MIN_POS;
    private static final double RAMP_STEP = 0.15;
    public Servo kickerServo;
    private static final double KICKER_RESTING_POS = 0.50;
    private static final double KICKER_EJECT_POS = 0.83;

    // ---------- TELEMETRY ----------
    private Telemetry telemetry;

    // ---------- MECHANISM STATES ----------
    private boolean intakeActive = false;
    private double intakePowerRequested = 0;
    private boolean intakeDirectionFlipRequested = true;

    private boolean outtakeActive = false;

    private boolean kickerActive = false;
    private ElapsedTime kickerTimer = new ElapsedTime();
    private static final double KICK_DURATION = 0.30;

    private boolean pusherActive = false;

    // ---------- BALL DETECTION ----------
    private ElapsedTime intakeCheckTimer = new ElapsedTime();
    private static final double INTAKE_CHECK_PERIOD = 50; // milliseconds

    // ---------- INIT ----------
    public void initMechanisms(HardwareMap hw, Telemetry telemetry) {
        this.telemetry = telemetry;
        initIntake(hw);
        initOuttake(hw);

        sorterLogic = new SorterLogic();
        sorterLogic.init(hw, telemetry);
    }

    private void initIntake(HardwareMap hw) {
        intakeMotor = hw.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        intakeServoFirst = hw.get(Servo.class, "intakeServoFirst");
        intakeServoFirst.setDirection(Servo.Direction.REVERSE);
        intakeServoFirst.setPosition(0.5);

        intakeServoSecond = hw.get(Servo.class, "intakeServoSecond");
        intakeServoSecond.setDirection(Servo.Direction.FORWARD);
        intakeServoSecond.setPosition(INTAKE_SERVO_SECOND_RESTING_POS);
    }

    private void initOuttake(HardwareMap hw) {
        outtakeMotor = hw.get(DcMotorEx.class, "outtakeMotor");
        rampAngleAdjust = hw.get(Servo.class, "rampAngle");
        kickerServo = hw.get(Servo.class, "kickerServo");

        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        rampAngleAdjust.setDirection(Servo.Direction.FORWARD);
        rampAngleAdjust.setPosition(RAMP_ANGLE_MIN_POS);

        kickerServo.setDirection(Servo.Direction.FORWARD);
        kickerServo.setPosition(KICKER_RESTING_POS);
    }

    // ---------- INTAKE ----------
    public void engageIntake(double power, boolean intakeDirectionFlip) {
        intakeActive = true;
        intakePowerRequested = power;
        intakeDirectionFlipRequested = intakeDirectionFlip;
    }

    public void disengageIntake() {
        intakeActive = false;
    }

    // ---------- SORTER WRAPPERS ----------
    public void sorterInitLoop() {
        if (sorterRate.milliseconds() >= SORTER_PERIOD_MS) {
            sorterLogic.init_loop();
            sorterRate.reset();
        }
    }

    public void sorterGoToIntake(int pocket) { sorterLogic.goToIntake(pocket); }
    public void sorterGoToOuttake(int pocket) { sorterLogic.goToOuttake(pocket); }
    public boolean sorterBallPresent() { return sorterLogic.isBallPresent(); }
    public SorterLogic.DetectedColor sorterDetectColor() { return sorterLogic.detectBallColor(); }
    public int getSorterCurrentPosition() { return sorterLogic.getCurrentPos(); }
    public int getSorterTargetPosition() { return sorterLogic.getTargetPos(); }
    public boolean sorterIsMoving() { return sorterLogic.moving; }
    public String getSorterStateText(int pocket) { return "UNKNOWN"; }

    // ---------- OUTTAKE ----------
    public void engageOuttake(double speed) {
        outtakeActive = true;
        manualOuttakeSpeed = speed;
    }

    public void disengageOuttake() {
        outtakeActive = false;
    }

    public void adjustOuttakeAngle(boolean increase, boolean decrease) {
        if (increase) rampAngleTarget += RAMP_STEP;
        if (decrease) rampAngleTarget -= RAMP_STEP;
        rampAngleTarget = Math.max(RAMP_ANGLE_MIN_POS, Math.min(RAMP_ANGLE_MAX_POS, rampAngleTarget));
    }

    public void ejectBall() {
        if (!kickerActive) {
            kickerActive = true;
            kickerTimer.reset();
        }
    }

    public void pushBallToSorter(boolean pressed) { pusherActive = pressed; }

    public void increaseOuttakeSpeed(double delta) { manualOuttakeSpeed = Math.min(1.0, manualOuttakeSpeed + delta); }
    public void decreaseOuttakeSpeed(double delta) { manualOuttakeSpeed = Math.max(0.0, manualOuttakeSpeed - delta); }
    public double getManualOuttakeSpeed() { return manualOuttakeSpeed; }
    public double getRampAngleTarget() { return rampAngleTarget; }
    public double getRampAngleCurrent() { return rampAngleAdjust.getPosition(); }

    private double rampTo(double current, double target, double rate) {
        double delta = target - current;
        if (Math.abs(delta) > rate) return current + Math.signum(delta) * rate;
        return target;
    }

    // ---------- CHECK COLOR SENSOR AND INTAKE BALL ----------
    private void checkAndIntakeBall() {
        if (intakeCheckTimer.milliseconds() < INTAKE_CHECK_PERIOD) return;
        intakeCheckTimer.reset();
        if (!intakeActive) return;

        // Detect ball from sensor
        SorterLogic.DetectedColor color = sorterLogic.detectBallColor();
        if (color != SorterLogic.DetectedColor.UNKNOWN) {
            sorterLogic.intakeBall(color);
        }
    }

    // ---------- MAIN UPDATE LOOP ----------
    public void updateMechanisms() {

        // --- INTAKE ---
        if (intakeActive) {
            double direction = intakeDirectionFlipRequested ? 1.0 : -1.0;
            intakeMotor.setPower(direction * intakePowerRequested);
            double servo1Pos = 0.5 + (direction * intakePowerRequested * 0.5);
            intakeServoFirst.setPosition(Math.max(0.0, Math.min(1.0, servo1Pos)));
        } else {
            intakeMotor.setPower(0);
            intakeServoFirst.setPosition(0.5);
        }

        // --- PUSHER ---
        intakeServoSecond.setPosition(pusherActive ? INTAKE_SERVO_SECOND_PUSH_POS : INTAKE_SERVO_SECOND_RESTING_POS);

        // --- SORTER ---
        sorterLogic.update();

        // --- OUTTAKE ---
        outtakeMotor.setVelocity(outtakeActive ? manualOuttakeSpeed * MAX_TICKS_PER_SEC_6000 : 0);

        // --- RAMP SERVO ---
        rampAngleAdjust.setPosition(rampTo(rampAngleAdjust.getPosition(), rampAngleTarget, 0.01));

        // --- KICKER ---
        if (kickerActive) {
            kickerServo.setPosition(KICKER_EJECT_POS);
            if (kickerTimer.seconds() > KICK_DURATION) {
                kickerServo.setPosition(KICKER_RESTING_POS);
                kickerActive = false;
            }
        }

        // --- CHECK COLOR SENSOR AND INTAKE BALL ---
        checkAndIntakeBall();
    }

    // ---------- D-PAD HELPERS ----------
    public void sorterMovePurpleToTop() { sorterLogic.movePurpleToTop(); }
    public void sorterMoveGreenToTop() { sorterLogic.moveGreenToTop(); }
}
