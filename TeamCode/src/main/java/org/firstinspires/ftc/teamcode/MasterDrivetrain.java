package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class MasterDrivetrain {

    // ---------------- Motors ----------------
    private DcMotorEx frontLeft, backLeft, frontRight, backRight;

    // ---------------- IMU ----------------
    private IMU imu;

    // ---------------- Tuning ----------------
    public static double BRAKE_MULT = 0.5;
    public static double RAMP_RATE = 0.4;
    public static double MIN_POWER = 0.15;
    public static double KICK_MULT = 1.4;
    public static long KICK_TIME_MS = 100;

    public static double FL_SCALE = 1.0;
    public static double FR_SCALE = 1.0;
    public static double BL_SCALE = 1.0;
    public static double BR_SCALE = 1.0;

    // ---------------- Ramp State ----------------
    private double curFL = 0, curFR = 0, curBL = 0, curBR = 0;

    private boolean wasMoving = false;
    private long kickStartTime = 0;

    // -------- Continuous heading (still needed for field-centric) --------
    private double lastImuYawDeg = 0.0;
    private double continuousHeadingDeg = 0.0;
    private boolean imuInitialized = false;

    public MasterDrivetrain() {}

    // ----------------------------------------------------------
    // INIT
    // ----------------------------------------------------------
    public void init(HardwareMap hardwareMap) {

        frontLeft  = hardwareMap.get(DcMotorEx.class, "leftFront");
        backLeft   = hardwareMap.get(DcMotorEx.class, "leftBack");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backRight  = hardwareMap.get(DcMotorEx.class, "rightBack");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(params);
        imu.resetYaw();
    }

    // ----------------------------------------------------------
    // ROBOT-CENTRIC DRIVE (NO HEADING HOLD)
    // ----------------------------------------------------------
    public void driveRobotCentric(double x, double y, double turn, boolean brake) {
        updateContinuousHeading();

        if (brake) {
            x *= BRAKE_MULT;
            y *= BRAKE_MULT;
            turn *= BRAKE_MULT;
        }

        // No heading hold — driver or auto-turn always controls rotation
        driveMecanum(x, y, turn);
    }

    // ----------------------------------------------------------
    // FIELD-CENTRIC DRIVE (still works)
    // ----------------------------------------------------------
    public void driveFieldCentric(double x, double y, double turn) {
        updateContinuousHeading();

        double headingRad = Math.toRadians(continuousHeadingDeg);

        double cosA = Math.cos(headingRad);
        double sinA = Math.sin(headingRad);

        double fieldX = x * cosA - y * sinA;
        double fieldY = x * sinA + y * cosA;

        driveMecanum(fieldX, fieldY, turn);
    }

    // ----------------------------------------------------------
    // CORE MECANUM
    // ----------------------------------------------------------
    private void driveMecanum(double x, double y, double turn) {

        boolean translating = Math.abs(x) > 1e-3 || Math.abs(y) > 1e-3;
        boolean isMoving = translating || Math.abs(turn) > 1e-3;

        // Kick only on translation, not rotation
        if (translating && !wasMoving) {
            kickStartTime = System.currentTimeMillis();
        }
        wasMoving = isMoving;

        boolean inKick =
                translating &&
                        (System.currentTimeMillis() - kickStartTime < KICK_TIME_MS);

        // ---------------- Mecanum math ----------------
        double fl = y + x + turn;
        double fr = y - x - turn;
        double bl = y - x + turn;
        double br = y + x - turn;

        // ---------------- Strafe detection ----------------
        boolean isStrafing = Math.abs(x) > Math.abs(y) * 1.2;

        if (isStrafing) {
            bl *= BL_SCALE;
            br *= BR_SCALE;
        }

        // ---------------- Normalize ----------------
        double max = Math.max(1.0,
                Math.max(Math.abs(fl),
                        Math.max(Math.abs(fr),
                                Math.max(Math.abs(bl), Math.abs(br)))));

        fl /= max;
        fr /= max;
        bl /= max;
        br /= max;

        // ---------------- Min power ----------------
        if (translating) {
            fl = applyMinPower(fl);
            fr = applyMinPower(fr);
            bl = applyMinPower(bl);
            br = applyMinPower(br);
        }

        // ---------------- Kick ----------------
        if (inKick) {
            fl *= KICK_MULT;
            fr *= KICK_MULT;
            bl *= KICK_MULT;
            br *= KICK_MULT;
        }

        // ---------------- Ramp ----------------
        curFL = ramp(curFL, fl);
        curFR = ramp(curFR, fr);
        curBL = ramp(curBL, bl);
        curBR = ramp(curBR, br);

        frontLeft.setPower(curFL);
        frontRight.setPower(curFR);
        backLeft.setPower(curBL);
        backRight.setPower(curBR);
    }

    // ----------------------------------------------------------
    // AUTO DRIVE
    // ----------------------------------------------------------
    public void runAutoDrive(double[] p) {

        double fl = p[0];
        double bl = p[1];
        double fr = p[2];
        double br = p[3];

        double max = Math.max(1.0,
                Math.max(Math.abs(fl),
                        Math.max(Math.abs(fr),
                                Math.max(Math.abs(bl), Math.abs(br)))));

        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }

    // ----------------------------------------------------------
    // IMU + HEADING
    // ----------------------------------------------------------
    private void updateContinuousHeading() {
        double rawYaw = imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.DEGREES);

        if (!imuInitialized) {
            lastImuYawDeg = rawYaw;
            continuousHeadingDeg = rawYaw;
            imuInitialized = true;
            return;
        }

        double delta = rawYaw - lastImuYawDeg;

        if (delta > 180)  delta -= 360;
        if (delta < -180) delta += 360;

        continuousHeadingDeg += delta;
        lastImuYawDeg = rawYaw;
    }

    public void resetImuYaw() {
        imu.resetYaw();
        imuInitialized = false;
        lastImuYawDeg = 0.0;
        continuousHeadingDeg = 0.0;
    }

    // ----------------------------------------------------------
    // HELPERS
    // ----------------------------------------------------------
    private double ramp(double cur, double target) {
        double delta = target - cur;
        if (Math.abs(delta) > RAMP_RATE)
            return cur + Math.signum(delta) * RAMP_RATE;
        return target;
    }

    private double applyMinPower(double val) {
        if (Math.abs(val) < 1e-4) return 0;
        return Math.signum(val) *
                (MIN_POWER + (1.0 - MIN_POWER) * Math.abs(val));
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private double clamp(double v) {
        return clamp(v, -1.0, 1.0);
    }

    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
