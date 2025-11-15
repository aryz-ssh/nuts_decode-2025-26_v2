package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Mechanisms {

    // ---------- INTAKE SYSTEM ----------
    public DcMotorEx intakeMotor;
    public Servo intakeServoFirst;
    public Servo intakeServoSecond;
    private static final double TICKS_PER_REV_1150 = 145.1;
    private static final double MAX_TICKS_PER_SEC_1150 = (TICKS_PER_REV_1150 * 1150) / 60.0;

    // ---------- SORTER SYSTEM ----------
    public DcMotor sortingMotor;


    // Using your old constant for ticks/rev — adjust if you know the exact motor
    private static final int TICKS_PER_REV = 384;   // e.g., 435 RPM motor
    // You can calculate ticks per degree:
    private static final double TICKS_PER_DEGREE = ((double) TICKS_PER_REV) / 360.0;

    // ---------- OUTTAKE SYSTEM ----------
    public DcMotorEx outtakeMotorLeft;
    public DcMotorEx outtakeMotorRight;
    private static final double TICKS_PER_REV_6000 = 28.0;
    private static final double MAX_TICKS_PER_SEC_6000 = (TICKS_PER_REV_6000 * 6000) / 60.0;

    // ---------- TELEMETRY ----------
    private Telemetry telemetry;

    // ---------- MANUAL OUTTAKE ----------
    private double manualOuttakeSpeed = 0.7; // default speed

    // ---------- RACK-AND-PINION ----------
    public Servo rackServo;
    private static final double RACK_RETRACTED_POS = 0.0;
    private static final double RACK_PUSHED_POS = 0.7;

    // ---------- INIT ----------
    public void initMechanisms(HardwareMap hw, Telemetry telemetry) {
        this.telemetry = telemetry;
        initIntake(hw);
        initOuttake(hw);
        initSorter(hw);
        initRackAndPinion(hw);
    }

    private void initIntake(HardwareMap hw) {
        intakeMotor = hw.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intakeServoFirst = hw.get(Servo.class, "intakeServoFirst");
        intakeServoFirst.setPosition(0.5);

        intakeServoSecond = hw.get(Servo.class, "intakeServoSecond");
        intakeServoSecond.setDirection(Servo.Direction.FORWARD);
        intakeServoSecond.setPosition(0.5);
    }

    private void initOuttake(HardwareMap hw) {
        outtakeMotorLeft = hw.get(DcMotorEx.class, "outtakeMotorLeft");
        outtakeMotorRight = hw.get(DcMotorEx.class, "outtakeMotorRight");

        outtakeMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        outtakeMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        outtakeMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        outtakeMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outtakeMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initSorter(HardwareMap hw) {
        sortingMotor = hw.get(DcMotor.class, "sortingMotor");
        sortingMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        sortingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sortingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sortingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    private void initRackAndPinion(HardwareMap hw) {
        rackServo = hw.get(Servo.class, "rackServo");
        rackServo.setPosition(RACK_RETRACTED_POS);
    }

    // ---------- INTAKE ----------
    public void engageIntake(double power, boolean intakeDirectionFlip) {
        double direction = intakeDirectionFlip ? 1.0 : -1.0;

        double servo1Pos = 0.5 + (direction * power * 0.5);
        servo1Pos = Math.max(0.0, Math.min(1.0, servo1Pos));
        double servo2Pos = 0.5 - (direction * power * 0.5);
        servo2Pos = Math.max(0.0, Math.min(1.0, servo2Pos));

        intakeServoFirst.setPosition(servo1Pos);
        intakeServoSecond.setPosition(servo2Pos);

        intakeMotor.setVelocity(direction * power * MAX_TICKS_PER_SEC_1150);

        telemetry.addData("Intake Velocity", intakeMotor.getVelocity());
        telemetry.addData("Servo1 Pos", servo1Pos);
        telemetry.addData("Servo2 Pos", servo2Pos);
        telemetry.update();
    }

    public void disengageIntake() {
        intakeMotor.setPower(0.0);
        intakeServoFirst.setPosition(0.5);
        intakeServoSecond.setPosition(0.5);
    }

    // ---------- OUTTAKE ----------
    public void engageOuttake(double speed) {
        pushBall();
        outtakeMotorLeft.setVelocity(speed * MAX_TICKS_PER_SEC_6000);
        outtakeMotorRight.setVelocity(speed * MAX_TICKS_PER_SEC_6000);

        telemetry.addData("Outtake Velocity", outtakeMotorLeft.getVelocity());
        telemetry.update();
    }

    public void disengageOuttake() {
        retractRack();
        outtakeMotorLeft.setVelocity(0.0);
        outtakeMotorRight.setVelocity(0.0);

        telemetry.addData("Outtake Velocity", outtakeMotorLeft.getVelocity());
        telemetry.update();
    }

    // ---------- SHOOTER ----------
    public void fireLauncher(double speed) {
        outtakeMotorLeft.setVelocity(speed * MAX_TICKS_PER_SEC_6000);
        outtakeMotorRight.setVelocity(speed * MAX_TICKS_PER_SEC_6000);
    }

    public void stopLauncher() {
        outtakeMotorLeft.setVelocity(0);
        outtakeMotorRight.setVelocity(0);
    }

    // ---------- SORTER (CAROUSEL) ----------

    /**
     * Rotate the carousel by a specified number of degrees (using encoders).
     * @param degrees how many degrees to rotate (e.g. 120)
     * @param power how fast to spin (0 to 1)
     */
    public void rotateCarouselDegrees(int degrees, double power) {
        if (sortingMotor == null) return;

        // Calculate how many encoder ticks corresponds to the desired degrees
        int targetTicks = sortingMotor.getCurrentPosition()
                + (int) Math.round(degrees * TICKS_PER_DEGREE);

        sortingMotor.setTargetPosition(targetTicks);
        sortingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sortingMotor.setPower(power);
    }

    /**
     * Call this periodically (each loop) in TeleOp to stop the carousel after reaching the target.
     */
    public void updateCarousel() {
        if (sortingMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            if (!sortingMotor.isBusy()) {
                // We've reached (or nearly reached) the target
                sortingMotor.setPower(0);
                sortingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }

    // ---------- COLOR DETECTION (optional telemetry) ----------
//    public String getBottomBallColor() {
//        NormalizedRGBA colors = bottomColorSensor.getNormalizedColors();
//        float r = colors.red / colors.alpha;
//        float g = colors.green / colors.alpha;
//        float b = colors.blue / colors.alpha;
//
//        // Example simple logic: more blue than red or green = purple
//        // More green = green = "green"
//        if (g > r && g > b) {
//            return "green";
//        }
//        if (b > r && b > g) {
//            return "purple";
//        }
//        return "unknown";
//    }

    // Old step method, but you can keep if you want:
    public void rotateCarouselStep(double power) {
        // 1. Degrees to rotate
        int degreesToRotate = 120;

        // 2. Motor encoder ticks per revolution
        // Adjust this value to match your motor specs!
        int ticksPerRevolution = 537;  // NeveRest 20 example

        // 3. Calculate target ticks
        int ticksToMove = (int)(degreesToRotate / 360.0 * ticksPerRevolution);

        // 4. Set target position relative to current position
        int target = sortingMotor.getCurrentPosition() + ticksToMove;

        // 5. Set motor to run to position
        sortingMotor.setTargetPosition(target);
        sortingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // 6. Set power
        sortingMotor.setPower(power);
    }


    // ---------- MANUAL OUTTAKE CONTROL ----------
    public void manualOuttake(boolean turnOn) {
        if (turnOn) {
            outtakeMotorLeft.setPower(manualOuttakeSpeed);
            outtakeMotorRight.setPower(manualOuttakeSpeed);
        } else {
            outtakeMotorLeft.setPower(0);
            outtakeMotorRight.setPower(0);
        }
    }

    public void rackMove(boolean turnOn){
        if(turnOn) {
            pushBall();
        }
        else {
            retractRack();
        }
    }

    public void increaseOuttakeSpeed(double delta) {
        manualOuttakeSpeed = Math.min(4.0, manualOuttakeSpeed + delta);
    }

    public void decreaseOuttakeSpeed(double delta) {
        manualOuttakeSpeed = Math.max(0.0, manualOuttakeSpeed - delta);
    }

    public double getManualOuttakeSpeed() {
        return manualOuttakeSpeed;
    }

    // ---------- RACK-AND-PINION ----------
    public void pushBall() {
        rackServo.setPosition(RACK_PUSHED_POS);
    }

    public void retractRack() {
        rackServo.setPosition(RACK_RETRACTED_POS);
    }
}
