package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.PostNut.DEADZONE;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class MasterDrivetrain {

    // ---------------- Dashboard Tuning ----------------
    public boolean isRedAlliance = false;

    public static double STRAFE_THRESHOLD = 0.05;
    public static double STRAFE_KP = 0.015;

    public static double FL_SCALE = 1.0;
    public static double FR_SCALE = 1.0;
    public static double BL_SCALE = 0.64;
    public static double BR_SCALE = 0.64;

    public static double BRAKE_HOLD_TORQUE = 0.18;  // Adjustable

    // ---------------- Motors ----------------
    public DcMotorEx frontLeft, backLeft, frontRight, backRight;

    // ---------------- Ramp Smoothing ----------------
    public static double rampRate = 0.25;
    private double currentFL = 0, currentFR = 0, currentBL = 0, currentBR = 0;

    // ---------------- Brake Assist Toggle ----------------
    public boolean brakeAssist = true;

    // ---------------- Strafe Heading ----------------
    private double savedHeading = 0;
    private boolean wasStrafing = false;
    public double startOffsetRadians = 0;
    private double targetHeading;
    public boolean fieldCentricEnabled = false;


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

        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        stop();
    }

    // ----------------------------------------------------------
    // TELEOP DRIVE — ROBOT CENTRIC
    // ----------------------------------------------------------
    public void driveRobotCentric(double x, double y, double turn, double headingRadians) {

        boolean noInput =
                Math.abs(x) < DEADZONE &&
                        Math.abs(y) < DEADZONE &&
                        Math.abs(turn) < DEADZONE;

        // ------------------------------------------------------
        // BRAKE ASSIST (HOLD POSITION)
        // ------------------------------------------------------
        if (brakeAssist && noInput) {
            frontLeft.setPower( BRAKE_HOLD_TORQUE);
            backLeft.setPower(  BRAKE_HOLD_TORQUE);
            frontRight.setPower(-BRAKE_HOLD_TORQUE);
            backRight.setPower( -BRAKE_HOLD_TORQUE);
            return;
        }

        // ------------------------------------------------------
        // STRAFE IMU DRIFT CORRECTION
        // ------------------------------------------------------
        boolean isStrafing = Math.abs(x) > Math.abs(y) + STRAFE_THRESHOLD;

        if (isStrafing && !wasStrafing) {
            savedHeading = headingRadians;
        }
        wasStrafing = isStrafing;

        double correction = 0;
        if (isStrafing) {
            double error = savedHeading - headingRadians;
            error = Math.atan2(Math.sin(error), Math.cos(error));
            correction = -(error * STRAFE_KP);
        }

        double finalTurn = turn + correction;

        // ------------------------------------------------------
        // MECANUM MATH
        // ------------------------------------------------------
        double fl = y + x + finalTurn;
        double fr = y - x - finalTurn;
        double bl = y - x + finalTurn;
        double br = y + x - finalTurn;

        // ------------------------------------------------------
        // WHEEL SCALING (only for strafe)
        // ------------------------------------------------------
        if (isStrafing) {
            bl *= BL_SCALE;
            br *= BR_SCALE;
        }

        // ------------------------------------------------------
        // NORMALIZATION + RAMP
        // ------------------------------------------------------
        double max = Math.max(1.0,
                Math.max(Math.abs(fl),
                        Math.max(Math.abs(fr),
                                Math.max(Math.abs(bl), Math.abs(br)))));

        fl /= max;
        fr /= max;
        bl /= max;
        br /= max;

        currentFL = ramp(currentFL, fl);
        currentFR = ramp(currentFR, fr);
        currentBL = ramp(currentBL, bl);
        currentBR = ramp(currentBR, br);

        frontLeft.setPower(currentFL);
        frontRight.setPower(currentFR);
        backLeft.setPower(currentBL);
        backRight.setPower(currentBR);
    }

    // ----------------------------------------------------------
    // FIELD CENTRIC
    // ----------------------------------------------------------
    public void driveFieldCentric(double x, double y, double turn, double headingRadians) {

        headingRadians -= startOffsetRadians;

        if (isRedAlliance)
            headingRadians += Math.PI;

        double rotatedX = x * Math.cos(headingRadians) - y * Math.sin(headingRadians);
        double rotatedY = x * Math.sin(headingRadians) + y * Math.cos(headingRadians);

        driveRobotCentric(rotatedX, rotatedY, turn, headingRadians);
    }

    public void resetHeadingFromFollower(double heading) {
        savedHeading = heading;   // for strafe correction
    }

    // ----------------------------------------------------------
    // Ramp function
    // ----------------------------------------------------------
    private double ramp(double cur, double target) {
        double delta = target - cur;
        if (Math.abs(delta) > rampRate)
            return cur + Math.signum(delta) * rampRate;
        return target;
    }

    // ----------------------------------------------------------
    // AUTO DRIVE (required by Pedro Pathing)
    // ----------------------------------------------------------
    public void runAutoDrive(double[] p) {

        double fl = p[0] * FL_SCALE;
        double bl = p[1] * BL_SCALE;
        double fr = p[2] * FR_SCALE;
        double br = p[3] * BR_SCALE;

        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr),
                        Math.max(Math.abs(bl), Math.abs(br)))));

        fl /= max;
        fr /= max;
        bl /= max;
        br /= max;

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    // ----------------------------------------------------------
    // STOP
    // ----------------------------------------------------------
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
