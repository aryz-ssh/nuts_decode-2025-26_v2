package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class MasterDrivetrain {

    // Dashboard-tunable compensation multipliers
    public static double FL_SCALE = 1.0;
    public static double FR_SCALE = 1.0;
    public static double BL_SCALE = 0.64;
    public static double BR_SCALE = 0.64;

    // Motors
    public DcMotorEx frontLeft, backLeft, frontRight, backRight;

    // Ramping (TeleOp only)
    private double currentFL = 0, currentFR = 0, currentBL = 0, currentBR = 0;
    private double rampRate = 0.25;

    public boolean brakeAssist = false;

    private static final double BRAKE_DAMPING = 0.4;       // How "stiff" driving feels
    private static final double BRAKE_HOLD_POWER = 0.10;    // Resistive torque


    LinearOpMode opMode;

    public MasterDrivetrain() {}

    public void init(HardwareMap hardwareMap) {

        frontLeft  = hardwareMap.get(DcMotorEx.class, "leftFront");
        backLeft   = hardwareMap.get(DcMotorEx.class, "leftBack");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backRight  = hardwareMap.get(DcMotorEx.class, "rightBack");

        // Motor directions (same as your original)
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // TeleOp uses RUN_WITHOUT_ENCODER
        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        stop();
    }

    // TeleOp Ramping
    private double rampToTarget(double current, double target) {
        double delta = target - current;

        if (Math.abs(delta) > rampRate)
            return current + Math.signum(delta) * rampRate;

        return target;
    }

    // TeleOp Robot Centric Ramping
    public void driveRobotCentric(double x, double y, double turn) {
        // ------------------------------------
        // BRAKE MODE ACTIVE → DAMP INPUT
        // ------------------------------------
        if (brakeAssist) {
            x *= BRAKE_DAMPING;
            y *= BRAKE_DAMPING;
            turn *= BRAKE_DAMPING;

            // Motors should resist when power = 0, so set ZeroPowerBehavior to BRAKE
            frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        } else {
            // Normal free movement
            frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        }

        // ------------------------------------
        // STANDARD MECANUM MATH
        // ------------------------------------
        double fl = y + x + turn;
        double fr = y - x - turn;
        double bl = y - x + turn;
        double br = y + x - turn;

        // ------------------------------------
        // ADD RESISTIVE TORQUE WHEN BRAKING
        // ------------------------------------
        if (brakeAssist) {
            fl += Math.signum(fl) * BRAKE_HOLD_POWER;
            fr += Math.signum(fr) * BRAKE_HOLD_POWER;
            bl += Math.signum(bl) * BRAKE_HOLD_POWER;
            br += Math.signum(br) * BRAKE_HOLD_POWER;
        }

        // Apply wheel scaling
        fl *= FL_SCALE;
        fr *= FR_SCALE;
        bl *= BL_SCALE;
        br *= BR_SCALE;

        // Normalize output
        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        fl /= max; fr /= max; bl /= max; br /= max;

        // RAMPING FOR TELEOP
        currentFL = rampToTarget(currentFL, fl);
        currentFR = rampToTarget(currentFR, fr);
        currentBL = rampToTarget(currentBL, bl);
        currentBR = rampToTarget(currentBR, br);

        frontLeft.setPower(currentFL);
        frontRight.setPower(currentFR);
        backLeft.setPower(currentBL);
        backRight.setPower(currentBR);
    }

    // AUTO (PEDRO) USES DIRECT POWER — NO RAMPING
    public void runAutoDrive(double[] p) {
        // p = {fl, bl, fr, br} EXACT ORDER from Pedro
        double fl = p[0] * FL_SCALE;
        double bl = p[1] * BL_SCALE;
        double fr = p[2] * FR_SCALE;
        double br = p[3] * BR_SCALE;

        // Normalize wheel outputs
        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        fl /= max; fr /= max; bl /= max; br /= max;

        // NO RAMPING → DIRECT OUTPUT
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    public void applyBrake(boolean brakeRequested, double x, double y, double turn) {

        boolean driverIsMoving = (Math.abs(x) > 0.05 || Math.abs(y) > 0.05 || Math.abs(turn) > 0.05);

        if (brakeRequested && !driverIsMoving) {
            // Hold the robot still
            frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            stop();  // Set motor power = 0 while brake mode holds
        } else {
            // Return to smooth driving mode
            frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        }
    }

    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
