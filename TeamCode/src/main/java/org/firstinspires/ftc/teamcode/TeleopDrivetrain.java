package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class TeleopDrivetrain {

    public DcMotorEx frontLeft;
    public DcMotorEx backLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backRight;

    // Minimal ramping—much smaller, very responsive
    private double currentFL = 0, currentFR = 0, currentBL = 0, currentBR = 0;
    private double rampRate = 0.25;   // you previously had 0.3 or 0.05

    LinearOpMode opMode;

    public TeleopDrivetrain(LinearOpMode op) {
        opMode = op;
    }

    public void initDriveTrain(com.qualcomm.robotcore.hardware.HardwareMap hardwareMap) {

        frontLeft  = hardwareMap.get(DcMotorEx.class, "leftFront");
        backLeft   = hardwareMap.get(DcMotorEx.class, "leftBack");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backRight  = hardwareMap.get(DcMotorEx.class, "rightBack");

        // ----------------------------------------------------
        // ORIGINAL MECHANUM MOTOR DIRECTION CONFIGURATION
        // (This is what drove correctly before odometry changes)
        // ----------------------------------------------------
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // No encoders for TeleOp
        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Brakes for control
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Remove all velocity PID
        frontLeft.setVelocityPIDFCoefficients(0,0,0,0);
        frontRight.setVelocityPIDFCoefficients(0,0,0,0);
        backLeft.setVelocityPIDFCoefficients(0,0,0,0);
        backRight.setVelocityPIDFCoefficients(0,0,0,0);

        stopMotors();
    }

    // ---------- Minimal Smoothing ----------
    private double rampToTarget(double current, double target) {
        double delta = target - current;

        if (Math.abs(delta) > rampRate)
            return current + Math.signum(delta) * rampRate;

        return target;
    }

    // ---------- Main Drive Control ----------
    public void updateDrive(double targetFL, double targetFR, double targetBL, double targetBR) {

        currentFL = rampToTarget(currentFL, targetFL);
        currentFR = rampToTarget(currentFR, targetFR);
        currentBL = rampToTarget(currentBL, targetBL);
        currentBR = rampToTarget(currentBR, targetBR);

        frontLeft.setPower(currentFL);
        frontRight.setPower(currentFR);
        backLeft.setPower(currentBL);
        backRight.setPower(currentBR);
    }

    public void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
