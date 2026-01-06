package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;


import com.qualcomm.hardware.limelightvision.Limelight3A;
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


@Config
public class Mechanisms {
    // --------- la la la limelight ---------
    public Limelight3A limelight;

    // --------- SORTER ----------
    public SorterLogicColor sorterLogic;

    private ElapsedTime sorterRate = new ElapsedTime();
    private static final double SORTER_PERIOD_MS = 60;

    // ---------- INTAKE SYSTEM ----------
    public DcMotorEx intakeMotor;
    public Servo intakeServoFirst;
    public Servo intakeServoSecond;
    private static final double TICKS_PER_REV_1150 = 145.1;
    private static final double MAX_TICKS_PER_SEC_1150 = (TICKS_PER_REV_1150 * 1150) / 60.0;

    // ---------- OUTTAKE SYSTEM ----------
    public DcMotorEx outtakeMotor;
    private static final double TICKS_PER_REV_6000 = 28.0;
    private static final double MAX_TICKS_PER_SEC_6000 = (TICKS_PER_REV_6000 * 6000) / 60.0;

    private double manualOuttakeSpeed = 0.7;

    public Servo rampAngleAdjust;
    public static double RAMP_ANGLE_MIN_POS = 0.3;
    private static final double RAMP_ANGLE_MAX_POS = 1;
    private double rampAngleTarget = RAMP_ANGLE_MIN_POS;
    private static final double RAMP_STEP = 0.1;

    public Servo kickerServo;
    public static double KICKER_RESTING_POS = 0.45;
    public static double KICKER_EJECT_POS = 0.83;

    // ---------- TELEMETRY -----------
    private Telemetry telemetry;

    // ---------- MECHANISM STATES ----------
    private boolean intakeActive = false;
    private double intakePowerRequested = 0;
    private boolean intakeDirectionFlipRequested = true;

    private boolean outtakeActive = false;

    private boolean kickerActive = false;
    private ElapsedTime kickerTimer = new ElapsedTime();
    public static double KICK_DURATION = 0.25;
    private int lastShotPocket = 1;

    public IMU imu;

    // ---------- INIT ----------
    public void initMechanisms(HardwareMap hw, Telemetry telemetry) {
        this.telemetry = telemetry;
        initIntake(hw);
        initOuttake(hw);
        initIMU(hw);

        limelight = hw.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        sorterLogic = new SorterLogicColor();
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
        intakeServoSecond.setPosition(0.5);
    }

    public void initOuttake(HardwareMap hw) {
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

    public boolean isSorterHomed() {
        return sorterLogic.isHomed();
    }

    //    public void sorterHome() { sorterLogic.homeSorter(); }
//    public boolean sorterIsHomed() { return sorterLogic.isHomed(); }
    public void sorterGoToIntake(int n) { sorterLogic.goToIntake(n); }
    public void sorterGoToOuttake(int n) { sorterLogic.goToOuttake(n); }

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

    public void setRampAngle(double target) {
        rampAngleTarget = Math.max(RAMP_ANGLE_MIN_POS,
                Math.min(RAMP_ANGLE_MAX_POS, target));
    }

    public void ejectBall() {
        if (!kickerActive) {
            kickerActive = true;
            kickerTimer.reset();
            sorterLogic.clearPocket(lastShotPocket);
        }
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

    public void setShotPocket(int pocket) {
        lastShotPocket = pocket;
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

    public boolean isSorterMoving() {
        return sorterLogic.isMoving();
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

            // double intakeServoPos = 0.5 + (direction * intakePowerRequested * 0.5);
            // intakeServoPos = Math.max(0.0, Math.min(1.0, intakeServoPos));

            // intakeServoFirst.setPosition(intakeServoPos);
            // intakeServoSecond.setPosition(intakeServoPos);

            intakeMotor.setPower(direction * intakePowerRequested);
        } else {
            intakeMotor.setPower(0);
            // intakeServoFirst.setPosition(0.5);
            // intakeServoSecond.setPosition(0.5);
        }

        // ========================
        // SORTER UPDATE
        // ========================
        sorterLogic.update();

        if (!sorterLogic.isMoving() && sorterLogic.isAtIntakePosition()) {
            SorterLogicColor.BallColor detected = sorterLogic.detectBallColor();

            if (detected != SorterLogicColor.BallColor.UNKNOWN)
                sorterLogic.storeColorForCurrentPocket(detected);
        }

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
}