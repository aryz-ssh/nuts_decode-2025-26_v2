package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
public class Mechanisms {

    // ---------- SORTER ----------
    public FinalSorter sorter;
    private DcMotorEx sorterMotor;
    private static int TICKS_PER_60 = 1000; // set this to your motor ticks per 60 degrees

    // ---------- INTAKE SYSTEM ----------
    public DcMotorEx intakeMotor;
    public Servo intakeServoFirst;
    public Servo intakeServoSecond;
    private boolean intakeActive = false;
    private double intakePowerRequested = 0;
    private boolean intakeDirectionFlipRequested = true;

    // ---------- OUTTAKE SYSTEM ----------
    public DcMotorEx outtakeMotor;
    public Servo rampAngleAdjust;
    public Servo kickerServo;
    private DigitalChannel outtakeBeamBreak;
    private boolean outtakeActive = false;
    private double manualOuttakeSpeed = 0.7;

    private static double RAMP_ANGLE_MIN_POS = 0.24;
    private static double RAMP_ANGLE_MAX_POS = 0.99;
    private double rampAngleTarget = RAMP_ANGLE_MIN_POS;
    private static final double RAMP_STEP = 0.1;

    public static double KICKER_RESTING_POS = 0.45;
    public static double KICKER_EJECT_POS = 0.83;
    private boolean kickerActive = false;
    private ElapsedTime kickerTimer = new ElapsedTime();
    public static double KICK_DURATION = 0.25;

    // ---------- IMU ----------
    public IMU imu;

    // ---------- TELEMETRY ----------
    private Telemetry telemetry;

    // ---------- INIT ----------
    public void initMechanisms(HardwareMap hw, Telemetry telemetry, boolean isAuto) {
        this.telemetry = telemetry;

        initIntake(hw);
        initOuttake(hw);

        if (!isAuto) initIMU(hw);

        // Init FinalSorter
        sorterMotor = hw.get(DcMotorEx.class, "sorterMotor");
        sorterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sorterMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        sorterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        sorter = new FinalSorter();
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
        intakeServoSecond.setPosition(0.5);
    }

    private void initOuttake(HardwareMap hw) {
        outtakeMotor = hw.get(DcMotorEx.class, "outtakeMotor");
        rampAngleAdjust = hw.get(Servo.class, "rampAngle");
        kickerServo = hw.get(Servo.class, "kickerServo");
        outtakeBeamBreak = hw.get(DigitalChannel.class, "outtakeBeamBreak");

        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rampAngleAdjust.setDirection(Servo.Direction.FORWARD);
        rampAngleAdjust.setPosition(RAMP_ANGLE_MIN_POS);

        kickerServo.setDirection(Servo.Direction.FORWARD);
        kickerServo.setPosition(KICKER_RESTING_POS);

        outtakeBeamBreak.setMode(DigitalChannel.Mode.INPUT);
    }

    private void initIMU(HardwareMap hw) {
        imu = hw.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));
        imu.resetYaw();
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

    // ---------- SORTER ----------
    public void recordBall(boolean isGreen) {
        sorter.markPendingBall(isGreen ? FinalSorter.BallColor.GREEN : FinalSorter.BallColor.PURPLE);
        sorter.rotateIntakeSlot();
    }

    public void moveBallToTop(boolean isGreen) {
        sorter.rotateClosestToTop(isGreen ? FinalSorter.BallColor.GREEN : FinalSorter.BallColor.PURPLE);
    }

    public boolean isSorterBusy() {
        return sorterMotor.isBusy();
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
        rampAngleTarget = Math.max(RAMP_ANGLE_MIN_POS, Math.min(RAMP_ANGLE_MAX_POS, rampAngleTarget));
    }

    public void setRampAngle(double target) {
        rampAngleTarget = Math.max(RAMP_ANGLE_MIN_POS, Math.min(RAMP_ANGLE_MAX_POS, target));
    }

    public void ejectBall() {
        if (!kickerActive) {
            kickerActive = true;
            kickerTimer.reset();
        }
    }

    public void increaseOuttakeSpeed(double delta) {
        manualOuttakeSpeed = Math.min(1.0, manualOuttakeSpeed + delta);
    }

    public void decreaseOuttakeSpeed(double delta) {
        manualOuttakeSpeed = Math.max(0.0, manualOuttakeSpeed - delta);
    }

    public double getManualOuttakeSpeed() { return manualOuttakeSpeed; }
    public double getRampAngleTarget() { return rampAngleTarget; }
    public double getRampAngleCurrent() { return rampAngleAdjust.getPosition(); }

    public double getHeadingRadians() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.RADIANS);
    }

    public void resetHeading() { imu.resetYaw(); }

    // ---------- MAIN UPDATE LOOP ----------
    public void updateMechanisms() {
        // INTAKE
        intakeMotor.setPower(intakeActive ? (intakeDirectionFlipRequested ? 1.0 : -1.0) * intakePowerRequested : 0);

        // OUTTAKE
        outtakeMotor.setVelocity(outtakeActive ? manualOuttakeSpeed * 28.0 * 6000 / 60.0 : 0);

        // RAMP SERVO
        double currentRamp = rampAngleAdjust.getPosition();
        double smoothedRamp = currentRamp + Math.signum(rampAngleTarget - currentRamp) * 0.01;
        if (Math.abs(rampAngleTarget - currentRamp) < 0.01) smoothedRamp = rampAngleTarget;
        rampAngleAdjust.setPosition(smoothedRamp);

        // KICKER
        if (kickerActive) {
            kickerServo.setPosition(KICKER_EJECT_POS);
            if (kickerTimer.seconds() > KICK_DURATION) {
                kickerServo.setPosition(KICKER_RESTING_POS);
                kickerActive = false;
            }
        }
    }
}
