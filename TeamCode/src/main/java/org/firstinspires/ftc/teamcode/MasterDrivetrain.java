package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class MasterDrivetrain {

    // ---------------- Dashboard Tuning ----------------
    public static double FL_SCALE = 1.0;
    public static double FR_SCALE = 1.0;
    public static double BL_SCALE = 0.64;
    public static double BR_SCALE = 0.64;

    public static double BRAKE_HOLD_TORQUE = 0.18;     // resisting torque
    public static double BRAKE_INPUT_DAMPING = 0.6;    // slow inputs in brake mode

    // ---------------- Motors ----------------
    public DcMotorEx frontLeft, backLeft, frontRight, backRight;

    // ---------------- Ramp Smoothing ----------------
    private double rampRate = 0.35;
    private double currentFL = 0, currentFR = 0, currentBL = 0, currentBR = 0;

    // ---------------- Brake Assist Toggle ----------------
    public boolean brakeAssist = false;

    public MasterDrivetrain() {}

    // ----------------------------------------------------------
    // INIT
    // ----------------------------------------------------------
    public void init(HardwareMap hardwareMap) {

        frontLeft  = hardwareMap.get(DcMotorEx.class, "leftFront");
        backLeft   = hardwareMap.get(DcMotorEx.class, "leftBack");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backRight  = hardwareMap.get(DcMotorEx.class, "rightBack");

        // Motor directions
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // TeleOp mode
        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        stop();
    }

    // ----------------------------------------------------------
    // TELEOP DRIVE (robot-centric mecanum)
    // ----------------------------------------------------------
    public void driveRobotCentric(double x, double y, double turn) {

        // Brake mode input damping
        if (brakeAssist) {
            x *= BRAKE_INPUT_DAMPING;
            y *= BRAKE_INPUT_DAMPING;
            turn *= BRAKE_INPUT_DAMPING;
        }

        // Basic mecanum wheel mix
        double fl = y + x + turn;
        double fr = y - x - turn;
        double bl = y - x + turn;
        double br = y + x - turn;

        // Brake Assist — add resistive torque & enable ZPB=BRAKE
        if (brakeAssist) {
            fl += Math.signum(fl) * BRAKE_HOLD_TORQUE;
            fr += Math.signum(fr) * BRAKE_HOLD_TORQUE;
            bl += Math.signum(bl) * BRAKE_HOLD_TORQUE;
            br += Math.signum(br) * BRAKE_HOLD_TORQUE;

            frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        } else {
            frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        }

        // Wheel scaling
        fl *= FL_SCALE;
        fr *= FR_SCALE;
        bl *= BL_SCALE;
        br *= BR_SCALE;

        // Normalize
        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

        fl /= max;
        fr /= max;
        bl /= max;
        br /= max;

        // Ramp smoothing
        currentFL = ramp(currentFL, fl);
        currentFR = ramp(currentFR, fr);
        currentBL = ramp(currentBL, bl);
        currentBR = ramp(currentBR, br);

        // Apply final power
        frontLeft.setPower(currentFL);
        frontRight.setPower(currentFR);
        backLeft.setPower(currentBL);
        backRight.setPower(currentBR);
    }

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
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

        fl /= max; fr /= max; bl /= max; br /= max;

        // No ramping, no brake assist — Pedro handles motion
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
