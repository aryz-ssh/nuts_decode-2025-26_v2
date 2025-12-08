package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Mechanisms {

    // --------- SORTER ----------
    public SorterLogicV2 sorterLogic;
    private ElapsedTime sorterRate = new ElapsedTime();
    private static final double SORTER_PERIOD_MS = 60;

    // ---------- INTAKE SYSTEM ----------
    public DcMotorEx intakeMotor;
    public Servo intakeServoFirst;
    public Servo intakeServoSecond;
    private static final double TICKS_PER_REV_1150 = 145.1;
    private static final double MAX_TICKS_PER_SEC_1150 = (TICKS_PER_REV_1150 * 1150) / 60.0;
    private static final double INTAKE_SERVO_SECOND_RESTING_POS = 0.9;
    private static final double INTAKE_SERVO_SECOND_PUSH_POS = 0.17;

    // ---------- OUTTAKE SYSTEM ----------
    public DcMotorEx outtakeMotor;
    private static final double TICKS_PER_REV_6000 = 28.0;
    private static final double MAX_TICKS_PER_SEC_6000 = (TICKS_PER_REV_6000 * 6000) / 60.0;

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
    public IMU imu;

    // ---------- INIT ----------
    public void initMechanisms(HardwareMap hw, Telemetry telemetry) {
        this.telemetry = telemetry;
        initIntake(hw);
        initOuttake(hw);
        initIMU(hw);

        sorterLogic = new SorterLogicV2();
        sorterLogic.init(hw, telemetry);
    }


    private void initIntake(HardwareMap hw) {
        intakeMotor = hw.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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
        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rampAngleAdjust.setDirection(Servo.Direction.FORWARD);
        rampAngleAdjust.setPosition(RAMP_ANGLE_MIN_POS);

        kickerServo.setDirection(Servo.Direction.FORWARD);
        kickerServo.setPosition(KICKER_RESTING_POS);
    }

    private void initIMU(HardwareMap hw) {
        // ---------------- IMU INIT ----------------
        imu = hw.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(parameters);
        imu.resetYaw();   // start at zero heading
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

    //    public void sorterHome() { sorterLogic.homeSorter(); }
//    public boolean sorterIsHomed() { return sorterLogic.isHomed(); }
    public void sorterGoToIntake(int n) { sorterLogic.goToIntake(n); }
    public void sorterGoToOuttake(int n) { sorterLogic.goToOuttake(n); }
    public boolean sorterBallPresent() { return sorterLogic.isBallPresent(); }
    public SorterLogicV2.BallColor sorterDetectColor() { return sorterLogic.detectBallColor(); }

    public int getSorterCurrentPosition() { return sorterLogic.getCurrentPos();}
    public int getSorterTargetPosition() { return sorterLogic.getTargetPos(); }
    public boolean sorterIsMoving() { return sorterLogic.moving; }

    public String getSorterStateText(int pocket) {
        int targetIntake, targetOuttake;

        if (pocket == 1) {
            targetIntake  = sorterLogic.B1_INTAKE;
            targetOuttake = sorterLogic.B1_OUTTAKE;
        } else if (pocket == 2) {
            targetIntake  = sorterLogic.B2_INTAKE;
            targetOuttake = sorterLogic.B2_OUTTAKE;
        } else {
            targetIntake  = sorterLogic.B3_INTAKE;
            targetOuttake = sorterLogic.B3_OUTTAKE;
        }

        if (sorterLogic.targetPos == targetIntake)  return "INTAKE";
        if (sorterLogic.targetPos == targetOuttake) return "OUTTAKE";
        return "UNKNOWN";
    }

    // ---------- OUTTAKE ----------
    public void engageOuttake(double speed) {
        outtakeActive = true;
        manualOuttakeSpeed = speed;
    }

    public void disengageOuttake() {
        outtakeActive = false;
    }

    public void adjustOuttakeAngle(boolean increase, boolean decrease) {
        if (increase)  rampAngleTarget += RAMP_STEP;
        if (decrease)  rampAngleTarget -= RAMP_STEP;

        rampAngleTarget = Math.max(RAMP_ANGLE_MIN_POS,
                Math.min(RAMP_ANGLE_MAX_POS, rampAngleTarget));
    }

    public void ejectBall() {
        if (!kickerActive) {
            kickerActive = true;
            kickerTimer.reset();
        }
    }

    public void pushBallToSorter(boolean pressed) {
        pusherActive = pressed;
    }

    public void increaseOuttakeSpeed(double delta) {
        manualOuttakeSpeed = Math.min(1.0, manualOuttakeSpeed + delta);
    }

    public void decreaseOuttakeSpeed(double delta) {
        manualOuttakeSpeed = Math.max(0.0, manualOuttakeSpeed - delta);
    }

    public double getManualOuttakeSpeed() {
        return manualOuttakeSpeed;
    }

    public double getRampAngleTarget() {
        return rampAngleTarget;
    }

    public double getRampAngleCurrent() {
        return rampAngleAdjust.getPosition();
    }

    private double rampTo(double current, double target, double rate) {
        double delta = target - current;
        if (Math.abs(delta) > rate)
            return current + Math.signum(delta) * rate;
        return target;
    }

    public double getHeadingRadians() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.RADIANS);
    }

    // ---------- RESET ----------
    public void resetHeading() {
        imu.resetYaw();
    }

    // ---------- MAIN UPDATE LOOP ----------
    public void updateMechanisms() {

        // ========================
        // INTAKE
        // ========================
        if (intakeActive) {
            double direction = intakeDirectionFlipRequested ? 1.0 : -1.0;

            double servo1Pos = 0.5 + (direction * intakePowerRequested * 0.5);
            servo1Pos = Math.max(0.0, Math.min(1.0, servo1Pos));

            intakeServoFirst.setPosition(servo1Pos);

            intakeMotor.setPower(direction * intakePowerRequested);
        } else {
            intakeMotor.setPower(0);
            intakeServoFirst.setPosition(0.5);
        }

        // ========================
        // PUSHER (immediate return)
        // ========================
        if (pusherActive)
            intakeServoSecond.setPosition(INTAKE_SERVO_SECOND_PUSH_POS);
        else
            intakeServoSecond.setPosition(INTAKE_SERVO_SECOND_RESTING_POS);

        // ========================
        // SORTER UPDATE
        // ========================
        sorterLogic.update();

        // ========================
        // OUTTAKE
        // ========================
        if (outtakeActive)
            outtakeMotor.setVelocity(manualOuttakeSpeed * MAX_TICKS_PER_SEC_6000);
        else
            outtakeMotor.setVelocity(0);

        // ========================
        // RAMP ANGLE SERVO (smooth)
        // ========================
        double currentRamp = rampAngleAdjust.getPosition();
        double smoothedRamp = rampTo(currentRamp, rampAngleTarget, 0.01);
        rampAngleAdjust.setPosition(smoothedRamp);

        // ========================
        // KICKER
        // ========================
        if (kickerActive) {
            kickerServo.setPosition(KICKER_EJECT_POS);

            if (kickerTimer.seconds() > KICK_DURATION) {
                kickerServo.setPosition(KICKER_RESTING_POS);
                kickerActive = false;
            }
        }
    }
    // ---------- NEW SORTER CODE ----------
    //First, we need to initialize the servo
    //Create a colorsensor object
    //After that
}