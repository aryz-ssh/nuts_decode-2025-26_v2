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
    public static double MIN_POWER = 0.15;   // 0.10–0.18 typical
    public static double KICK_MULT = 1.4;    // 1.2–1.5
    public static long KICK_TIME_MS = 100; // 60–120 ms


    public static double FL_SCALE = 1.0;
    public static double FR_SCALE = 1.0;
    public static double BL_SCALE = 1.0;
    public static double BR_SCALE = 1.0;

    // ---------------- Heading Hold ----------------
    private double lockedHeadingDeg = 0.0;

    public static double HEADING_KP = 0.0;      // turn per degree 0.04
    public static double MAX_HEADING_CORR = 0.35;
    public static double TURN_DEADBAND = 0.08;
    public static double TURN_CORRECT_SPEED_FLOOR = 0.2;
    private long lastTurnTimeMs = 0;
    public static long TURN_SETTLE_MS = 150; // 100–200ms sweet spot
    private boolean wasMovingZPB = false;


    // ---------------- Ramp State ----------------
    private double curFL = 0, curFR = 0, curBL = 0, curBR = 0;

    private boolean wasMoving = false;
    private long kickStartTime = 0;

    // -------- Continuous heading --------
    private double lastImuYawDeg = 0.0;
    private double continuousHeadingDeg = 0.0;
    private boolean imuInitialized = false;
    private boolean wasTranslating = false;

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
    // ROBOT-CENTRIC DRIVE (BRAKE ALLOWED)
    // ----------------------------------------------------------
    public void driveRobotCentric(double x, double y, double turn, boolean brake) {
        updateContinuousHeading();

        if (brake) {
            x *= BRAKE_MULT;
            y *= BRAKE_MULT;
            turn *= BRAKE_MULT;
        }

        // --------------------------------------------------
        // HEADING HOLD (robot-centric ONLY)
        // --------------------------------------------------

        double currentHeadingDeg = continuousHeadingDeg;

        boolean translating =
                Math.abs(x) > 1e-3 ||
                        Math.abs(y) > 1e-3;

        boolean driverTurning =
                Math.abs(turn) > TURN_DEADBAND;

        // --------------------------------------------------
        // HEADING LOCK LOGIC (EDGE-BASED)
        // --------------------------------------------------

        // Lock heading ONCE when translation starts
        if (translating && !wasTranslating && !driverTurning) {
            lockedHeadingDeg = currentHeadingDeg;
        }

        // If driver manually turns, release and re-lock immediately
        if (driverTurning) {
            lockedHeadingDeg = currentHeadingDeg;
        }

        // Track translation state
        wasTranslating = translating;

        double finalTurn;

// track last time the driver actively turned
        if (driverTurning) {
            lastTurnTimeMs = System.currentTimeMillis();
        }

// are we still in the post-turn settle window?
        boolean inTurnSettle =
                (System.currentTimeMillis() - lastTurnTimeMs) < TURN_SETTLE_MS;

        if (driverTurning) {
            // Driver has full control
            finalTurn = turn;
            lockedHeadingDeg = currentHeadingDeg;

        } else if (!inTurnSettle) {
            // Heading hold is allowed (even if x/y = 0)
            double errorDeg = wrapDegrees(lockedHeadingDeg - currentHeadingDeg);

            // FULL correction at all times
            double speedScale = 1.0;

            finalTurn = -HEADING_KP * speedScale * errorDeg;

            finalTurn = Math.max(
                    -MAX_HEADING_CORR,
                    Math.min(MAX_HEADING_CORR, finalTurn)
            );


        } else {
            // Settling period — do NOTHING
            finalTurn = 0.0;
            lockedHeadingDeg = currentHeadingDeg;
        }

        driveMecanum(x, y, finalTurn);
    }

    // ----------------------------------------------------------
    // FIELD-CENTRIC DRIVE (NO BRAKE)
    // ----------------------------------------------------------
    public void driveFieldCentric(double x, double y, double turn) {
        driveMecanum(x, y, turn);
    }

    // ----------------------------------------------------------
    // CORE MECANUM (shared)
    // ----------------------------------------------------------
    private void driveMecanum(double x, double y, double turn) {

        boolean isMoving =
                Math.abs(x) > 1e-3 ||
                        Math.abs(y) > 1e-3 ||
                        Math.abs(turn) > 1e-3;

        // ----- ZERO POWER BEHAVIOR SWITCH (STATE-BASED) -----
        if (isMoving != wasMovingZPB) {
            DcMotorEx.ZeroPowerBehavior zpb =
                    isMoving
                            ? DcMotorEx.ZeroPowerBehavior.FLOAT
                            : DcMotorEx.ZeroPowerBehavior.BRAKE;

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

        boolean inKick =
                isMoving &&
                        (System.currentTimeMillis() - kickStartTime < KICK_TIME_MS);

        // ---------------- Mecanum math ----------------
        double fl = y + x + turn;
        double fr = y - x - turn;
        double bl = y - x + turn;
        double br = y + x - turn;

        // ---------------- Strafe dominance detection ----------------
        // Lateral vs forward comparison (Pedro-style)
        boolean isStrafing =
                Math.abs(fl + br - fr - bl) >
                        Math.abs(fl + fr + bl + br) * 0.3;

        // ---------------- Apply scaling ONLY for strafe ----------------
        if (isStrafing) {
            bl *= BL_SCALE;
            br *= BR_SCALE;
            // fronts intentionally untouched
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

        fl = clamp(fl);
        fr = clamp(fr);
        bl = clamp(bl);
        br = clamp(br);

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
    // IMU ACCESS
    // ----------------------------------------------------------
    public double getHeadingRad() {
        return imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.RADIANS);
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

    private double clamp(double v) {
        return Math.max(-1.0, Math.min(1.0, v));
    }

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

        // unwrap across ±180
        if (delta > 180)  delta -= 360;
        if (delta < -180) delta += 360;

        continuousHeadingDeg += delta;
        lastImuYawDeg = rawYaw;
    }

    private double wrapDegrees(double deg) {
        while (deg > 180)  deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }

    public void resetImuYaw() {
        imu.resetYaw();

        imuInitialized = false;
        lastImuYawDeg = 0.0;
        continuousHeadingDeg = 0.0;

        lockedHeadingDeg = 0.0;
    }

    public double getHeadingDeg() {
        return continuousHeadingDeg;
    }

    public double getLockedHeadingDeg() {
        return lockedHeadingDeg;
    }

    public double getHeadingErrorDeg() {
        return wrapDegrees(lockedHeadingDeg - continuousHeadingDeg);
    }

    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
