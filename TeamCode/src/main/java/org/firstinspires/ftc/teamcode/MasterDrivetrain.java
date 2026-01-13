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

    // -------- Continuous heading --------
    private double lastImuYawDeg = 0.0;
    private double continuousHeadingDeg = 0.0;
    private boolean imuInitialized = false;

    // ---------------- ZPB ----------------
    private boolean wasMovingZPB = false;

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

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

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
    // ROBOT-CENTRIC DRIVE
    // ----------------------------------------------------------
    public void driveRobotCentric(double x, double y, double turn) {
        updateContinuousHeading();
        driveMecanum(x, y, turn);
    }

    // ----------------------------------------------------------
    // FIELD-CENTRIC DRIVE
    // ----------------------------------------------------------
    public void driveFieldCentric(double x, double y, double turn) {
        driveMecanum(x, y, turn);
    }

    // ----------------------------------------------------------
    // CORE MECANUM
    // ----------------------------------------------------------
    private void driveMecanum(double x, double y, double turn) {

        boolean isMoving =
                Math.abs(x) > 1e-3 ||
                        Math.abs(y) > 1e-3 ||
                        Math.abs(turn) > 1e-3;

        // ----- ZERO POWER BEHAVIOR SWITCH -----
        if (isMoving != wasMovingZPB) {
            DcMotorEx.ZeroPowerBehavior zpb =
                    isMoving ? DcMotorEx.ZeroPowerBehavior.FLOAT : DcMotorEx.ZeroPowerBehavior.FLOAT;

            frontLeft.setZeroPowerBehavior(zpb);
            frontRight.setZeroPowerBehavior(zpb);
            backLeft.setZeroPowerBehavior(zpb);
            backRight.setZeroPowerBehavior(zpb);

            wasMovingZPB = isMoving;
        }

        boolean translating = Math.abs(x) > 1e-3 || Math.abs(y) > 1e-3;

        // Kick detection
        if (isMoving && !wasMoving) {
            kickStartTime = System.currentTimeMillis();
        }
        wasMoving = isMoving;

        boolean inKick = isMoving && (System.currentTimeMillis() - kickStartTime < KICK_TIME_MS);

        // ---------------- Mecanum math ----------------
        double fl = y + x + turn;
        double fr = y - x - turn;
        double bl = y - x + turn;
        double br = y + x - turn;

        // ---------------- Strafe scaling ----------------
        boolean isStrafing = Math.abs(fl + br - fr - bl) > Math.abs(fl + fr + bl + br) * 0.3;
        if (isStrafing) {
            bl *= BL_SCALE;
            br *= BR_SCALE;
        }

        // ---------------- Normalize ----------------
        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        fl /= max; fr /= max; bl /= max; br /= max;

        // ---------------- Min power ----------------
        if (translating) {
            fl = applyMinPower(fl);
            fr = applyMinPower(fr);
            bl = applyMinPower(bl);
            br = applyMinPower(br);
        }

        // ---------------- Kick ----------------
        if (inKick) {
            fl *= KICK_MULT; fr *= KICK_MULT; bl *= KICK_MULT; br *= KICK_MULT;
        }

        fl = clamp(fl); fr = clamp(fr); bl = clamp(bl); br = clamp(br);

        // ---------------- Ramp ----------------
        curFL = ramp(curFL, fl); curFR = ramp(curFR, fr); curBL = ramp(curBL, bl); curBR = ramp(curBR, br);

        frontLeft.setPower(curFL);
        frontRight.setPower(curFR);
        backLeft.setPower(curBL);
        backRight.setPower(curBR);
    }

    // ----------------------------------------------------------
    // IMU ACCESS
    // ----------------------------------------------------------
    public double getHeadingRad() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public double getHeadingDeg() {
        return continuousHeadingDeg;
    }

    // ----------------------------------------------------------
    // HELPERS
    // ----------------------------------------------------------
    private double ramp(double cur, double target) {
        double delta = target - cur;
        if (Math.abs(delta) > RAMP_RATE) return cur + Math.signum(delta) * RAMP_RATE;
        return target;
    }

    private double applyMinPower(double val) {
        if (Math.abs(val) < 1e-4) return 0;
        return Math.signum(val) * (MIN_POWER + (1.0 - MIN_POWER) * Math.abs(val));
    }

    private double clamp(double v) {
        return Math.max(-1.0, Math.min(1.0, v));
    }

    private void updateContinuousHeading() {
        double rawYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        if (!imuInitialized) {
            lastImuYawDeg = rawYaw;
            continuousHeadingDeg = rawYaw;
            imuInitialized = true;
            return;
        }

        double delta = rawYaw - lastImuYawDeg;
        if (delta > 180) delta -= 360;
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

    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
