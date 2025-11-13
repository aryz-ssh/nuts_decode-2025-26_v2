package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.util.ElapsedTime;


public class TeleopDrivetrain {
    public DcMotorEx frontLeft;
    public DcMotorEx backLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backRight;
    public IMU imu;
    public DcMotorEx odoLeft;
    public DcMotorEx odoRight;
    public DcMotorEx odoCenter;
    private static final double ODO_TICKS_PER_REV = 8192.0;     // REV Through-Bore typical
    private static final double ODO_WHEEL_DIAM_IN = 2.0;        // inches
    private static final double ODO_WHEEL_CIRCUMFERENCE = Math.PI * ODO_WHEEL_DIAM_IN;
    private static final double ODO_TICKS_TO_INCHES = ODO_WHEEL_CIRCUMFERENCE / ODO_TICKS_PER_REV;
    private static final double TRACK_WIDTH_IN = 10.5;          // distance between left/right odo wheels
    private int lastLeftOdo = 0;
    private int lastRightOdo = 0;
    private int lastCenterOdo = 0;

    private double currentFL = 0, currentFR = 0, currentBL = 0, currentBR = 0;
    private double rampRate = 0.05; // change per loop (tune this value)

    public double targetFL = 0, targetFR = 0, targetBL = 0, targetBR = 0;
    public double targetHeading = 0.0;   // degrees, used for heading hold


    private ElapsedTime runtime = new ElapsedTime();

    LinearOpMode opMode;

    // 435 rpm motor

    private static final double TICKS_PER_REV = 383.6;
    private static final double MAX_RPM = 435.0;
    private static final double MAX_TICKS_PER_SEC = (TICKS_PER_REV * MAX_RPM) / 60.0; // ≈ 2786 t/s
    public TeleopDrivetrain(LinearOpMode op) {
        opMode = op;
    }

    public double getHeadingDriftCorrection() {
        // read raw odo tick positions
        int leftNow = odoLeft.getCurrentPosition();
        int rightNow = odoRight.getCurrentPosition();

        // convert to inches
        double dL = (leftNow - lastLeftOdo) * ODO_TICKS_TO_INCHES;
        double dR = (rightNow - lastRightOdo) * ODO_TICKS_TO_INCHES;

        // update stored tick counts for next loop
        lastLeftOdo = leftNow;
        lastRightOdo = rightNow;

        // compute the difference in distance traveled
        double dTheta = (dR - dL) / TRACK_WIDTH_IN;   // radians since last frame

        // proportional gain — tune between 0.2–1.0
        double kP = 0.5;

        // drift correction value: positive means robot veered right
        double correction = -kP * dTheta;

        return correction;  // to be added to your rotation term (rx)
    }

    public double getLateralDriftCorrection() {
        int centerNow = odoCenter.getCurrentPosition();

        // convert ticks to inches of sideways motion
        double dC = (centerNow - lastCenterOdo) * ODO_TICKS_TO_INCHES;
        lastCenterOdo = centerNow;

        // proportional gain for lateral correction — tune 0.05–0.2
        double kP_strafe = 0.1;

        // correction value: positive means drifting right
        double correction = -kP_strafe * dC;

        return correction;  // to be added to your strafe input (x)
    }


    public void initDriveTrain(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,      // adjust if hub mounted differently
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );


        imu.initialize(parameters);

        frontLeft  = hardwareMap.get(DcMotorEx.class, "leftFront");
        backLeft   = hardwareMap.get(DcMotorEx.class, "leftBack");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backRight  = hardwareMap.get(DcMotorEx.class, "rightBack");

        odoLeft   = hardwareMap.get(DcMotorEx.class, "leftFront");   // shares encoder with backLeft motor power
        odoRight  = hardwareMap.get(DcMotorEx.class, "rightRear");  // shares encoder with backRight motor power
        odoCenter = hardwareMap.get(DcMotorEx.class, "rightFront");  // example; adjust to your wiring


        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        odoLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoCenter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        odoLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoCenter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stopMotors();
    }

    private double rampToTarget(double current, double target) {
        double delta = target - current;
        // Limit change per loop to +/- rampRate
        if (Math.abs(delta) > rampRate) {
            current += Math.signum(delta) * rampRate;
        } else {
            current = target;
        }
        return current;
    }

    public void updateDrive(double targetFL, double targetFR, double targetBL, double targetBR) {
        // Ramp each motor toward its target
        currentFL = rampToTarget(currentFL, targetFL);
        currentFR = rampToTarget(currentFR, targetFR);
        currentBL = rampToTarget(currentBL, targetBL);
        currentBR = rampToTarget(currentBR, targetBR);

        // Apply smoothed powers
        frontLeft.setPower(currentFL);
        frontRight.setPower(currentFR);
        backLeft.setPower(currentBL);
        backRight.setPower(currentBR);
    }


    public void stopMotors(){
        opMode.telemetry.addData("Status", "Stopped");
        opMode.telemetry.update();
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

/*    public void moveForward(double power) {
        runtime.reset();

        opMode.telemetry.addData("Status", "Moving Forward");
        opMode.telemetry.update();

        double velocity = power * MAX_TICKS_PER_SEC;

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);


       // opMode.sleep(targetInSeconds);
    }

    public void moveBackwards(double power) {
        opMode.telemetry.addData("Status", "Moving Backward");
        opMode.telemetry.update();

        double velocity = power * MAX_TICKS_PER_SEC;

        frontLeft.setPower(-power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(-power);

     //   while(runtime.milliseconds() < targetInMillis) {
            //Keeps on looping until target is reached
       // }
    }

    public void strafeLeft(double power) {
        runtime.reset();

        opMode.telemetry.addData("Status", "Moving Left");
        opMode.telemetry.update();

        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(-power);

*//*        double velocity = power * MAX_TICKS_PER_SEC;

        frontLeft.setPower(velocity);
        frontRight.setPower(-velocity);
        backLeft.setPower(-velocity);
        backRight.setPower(velocity);*//*

     //   while(runtime.milliseconds() < targetTimeMillis) {
            //keeps on looping until target is reached
       // }
    }

    public void strafeRight(double power){
        runtime.reset();

        opMode.telemetry.addData("Status", "Moving Right");
        opMode.telemetry.update();

        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(power);

*//*        double velocity = power * MAX_TICKS_PER_SEC;

        frontLeft.setPower(-velocity);
        frontRight.setPower(velocity);
        backLeft.setPower(velocity);
        backRight.setPower(-velocity);*//*

       // while(runtime.milliseconds() < targetTimeMillis) {

       // }
    }*/


    public void moveForward(double power) {
        opMode.telemetry.addData("Status", "Moving Forward");
        opMode.telemetry.update();

        targetFL = power;
        targetFR = power;
        targetBL = power;
        targetBR = power;
    }

    public void moveBackwards(double power) {
        opMode.telemetry.addData("Status", "Moving Backward");
        opMode.telemetry.update();

        targetFL = -power;
        targetFR = -power;
        targetBL = -power;
        targetBR = -power;
    }

    public void strafeLeft(double power) {
        opMode.telemetry.addData("Status", "Strafing Left");
        opMode.telemetry.update();

        targetFL = -power;
        targetFR = power;
        targetBL = power;
        targetBR = -power;
    }

    public void strafeRight(double power) {
        opMode.telemetry.addData("Status", "Strafing Right");
        opMode.telemetry.update();

        targetFL = power;
        targetFR = -power;
        targetBL = power;
        targetBR = -power;
    }
    public void rotateLeft(double power) {
        opMode.telemetry.addData("Status", "Rotating");
        opMode.telemetry.update();

        frontLeft.setPower(-power);
        backLeft.setPower(-power);
        frontRight.setPower(power);
        backRight.setPower(power);

        // opMode.sleep(targetInMilis);
    }

    public void rotateRight(double power) {
        opMode.telemetry.addData("Status", "Rotating");
        opMode.telemetry.update();

        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(-power);

        //opMode.sleep(targetInMilis);
    }

/*    public void rotateToAngle(double targetAngle, double power) {
        double tolerance = 1.0; // Tolerance in degrees for accuracy
        double currentAngle = imu.getRobotYawPitchRollAngles().getYaw();

        // Normalize target angle to be within 0-360 range
        targetAngle = ((targetAngle % 360) + 360) % 360;

        // Loop until within tolerance of target angle
        while (opMode.opModeIsActive()) {
            currentAngle = imu.getRobotYawPitchRollAngles().getYaw();

            // Calculate the difference between current and target angles
            double angleDiff = (targetAngle - currentAngle + 360) % 360;

            // If the angle difference is greater than 180, it's shorter to rotate in the opposite direction
            if (angleDiff > 180) {
                angleDiff -= 360; // Rotate counterclockwise instead of clockwise
            }

            // Check if we're within the target tolerance
            if (Math.abs(angleDiff) < tolerance) {
                break;
            }

            // Determine the rotation direction based on angle difference
            double direction = angleDiff > 0 ? 1 : -1; // Clockwise if positive, counterclockwise if negative

            // Rotate with the calculated power and direction
         //   rotate(direction * power, 50.0); // Rotate for a small increment (e.g., 50 ms)

            // Telemetry for debugging
            opMode.telemetry.addData("Target Angle", targetAngle);
            opMode.telemetry.addData("Current Angle", currentAngle);
            opMode.telemetry.addData("Angle Difference", angleDiff);
            opMode.telemetry.update();
        }

        // Stop the motors once the target angle is reached
        stopMotors();
    }*/

//    private int convertInchesToTicks(double inches) {
//        double ticksPerRevolution = 1538; // Example: Neverest 20 motor
//        double wheelCircumference = 4 * Math.PI; // 4-inch diameter wheel
//        return (int) ((inches / wheelCircumference) * ticksPerRevolution);
//    }
//
//    public void moveForward(double distanceInInches, double power) {
//        int targetTicks = convertInchesToTicks(distanceInInches);
//        //resetEncoders();
//
//
//        frontLeft.setPower(power);
//        frontRight.setPower(power);
//        backLeft.setPower(power);
//        backRight.setPower(power); // Forward power
//        while (Math.abs(getEncoderPosition()) < targetTicks) {
//            // Wait until the target is reached
//        }
//        stopMotors();
//    }





   /*
    public void initGyuro(HardwareMap hardwareMap) {
        parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu = hardwareMap.get(IMU.class, "imu");
        ypr = imu.getRobotYawPitchRollAngles();
        imu.initialize(parameters);


    }

    */

    public void presetL2() {
    }

    public void presetR2() {

    }
}

