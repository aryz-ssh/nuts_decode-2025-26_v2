package org.firstinspires.ftc.teamcode;

// import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Mechanisms {
    // INTAKE SYSTEM VARIABLES
    // 1150 RPM
    public DcMotorEx intakeMotor;
    public Servo intakeServo;
    private static final double TICKS_PER_REV_1150 = 145.1;
    private static final double MAX_TICKS_PER_SEC_1150 = (TICKS_PER_REV_1150 * 1150) / 60.0; // ≈ 2786 t/s

    // SORTING SYSTEM VARIABLES
    // ASSUMING 435 RPM
    public DcMotor sortingMotor;
    private static final double TICKS_PER_REV_435 = 384.5; // Encoder ticks per revolution
    private static final double MAX_TICKS_PER_SEC_435 = (TICKS_PER_REV_435 * 435) / 60.0; // ≈ 2786 t/s
    private static final int sorter_pos1 = 67; // Encoder ticks per revolution
    private static final int sorter_pos2 = 67; // Encoder ticks per revolution
    private static final int sorter_pos3 = 67; // Encoder ticks per revolution


    // OUTTAKE SYSTEM VARIABLES
    // 6000 RPM MOTORS
    public DcMotorEx outtakeMotorLeft;
    public DcMotorEx outtakeMotorRight;
    private static final double TICKS_PER_REV_6000 = 28.0;
    private static final double MAX_TICKS_PER_SEC_6000 = (TICKS_PER_REV_6000 * 6000) / 60.0; // ≈ 2800 t/s

    // ELEVATION SYSTEM VARIABLES
    // ASSUMING 30 RPM
    public DcMotorEx raiserLeft;
    public DcMotorEx raiserRight;

    // MISCELLANEOUS VARIABLES
    private Telemetry telemetry;


    public void initTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void initIntakeSystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor"); // 1150 rpm dc motor

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Simple standard servo
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        intakeServo.setPosition(0.5);
    }

    public void initOuttakeSystem(HardwareMap hardwareMap) {
        outtakeMotorLeft = hardwareMap.get(DcMotorEx.class, "outtakeMotorLeft"); // 1150 rpm dc motor
        outtakeMotorRight = hardwareMap.get(DcMotorEx.class, "outtakeMotorRight"); // 1150 rpm dc motor

        outtakeMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        outtakeMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void initSorter(HardwareMap hardwareMap) {
        sortingMotor = hardwareMap.get(DcMotor.class, "sortingMotor"); // 435 rpm dc motor

        sortingMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        sortingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sortingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sortingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

/*    public void initElevation(HardwareMap hardwareMap) {
        raiserLeft = hardwareMap.get(DcMotorEx.class, "raiserLeft"); // 30 rpm dc motor
        raiserRight = hardwareMap.get(DcMotorEx.class, "raiserRight"); // 30 rpm dc motor

        sortingMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        sortingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sortingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sortingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        raiserRight.setDirection(DcMotorSimple.Direction.FORWARD);
        raiserRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        raiserRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        raiserRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }*/

    public void initMechanisms(HardwareMap hw, Telemetry telemetry) {
        this.telemetry = telemetry;
        initIntakeSystem(hw);
        initOuttakeSystem(hw);
    }

    public void engageIntake(double power, boolean intakeDirectionFlip) {
        double direction = intakeDirectionFlip ? 1.0 : -1.0;
        double servoPosition = 0.5 + (direction * power * 0.5);

        servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));

        intakeServo.setPosition(servoPosition);
        intakeMotor.setVelocity(direction * power * MAX_TICKS_PER_SEC_1150);
    }

    public void disengageIntake() {
        // intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setPower(0.0);
        intakeServo.setPosition(0.5);
    }

    public void indexArtifacts() {
        sortingMotor.setTargetPosition(sorter_pos1);
        sortingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sortingMotor.setPower(0.5);
        // record color
        sortingMotor.setTargetPosition(sorter_pos2);
        sortingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sortingMotor.setPower(0.5);
        // record color
        sortingMotor.setTargetPosition(sorter_pos3);
        sortingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sortingMotor.setPower(0.5);
    }

    public void cycleArtifacts(int position) {
        if (position == 0) {
            sortingMotor.setTargetPosition(sorter_pos1);
            sortingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sortingMotor.setPower(0.5);
        } else if (position == 1) {
            sortingMotor.setTargetPosition(sorter_pos1);
            sortingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sortingMotor.setPower(0.5);
        } else if (position == 2) {
            sortingMotor.setTargetPosition(sorter_pos1);
            sortingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sortingMotor.setPower(0.5);
        } else {
            telemetry.addData("Status:", "in cycleArtifacts, invalid input");
        }

    }

    public void engageOuttake(double power) {
        outtakeMotorLeft.setVelocity(power * MAX_TICKS_PER_SEC_6000);
        outtakeMotorRight.setVelocity(power * MAX_TICKS_PER_SEC_6000);

        telemetry.addData("Outtake Velocity", outtakeMotorLeft.getVelocity());
        telemetry.update();
    }

    public void disengageOuttake() {
        outtakeMotorLeft.setVelocity(0.0);
        outtakeMotorRight.setVelocity(0.0);

        telemetry.addData("Outtake Velocity", outtakeMotorLeft.getVelocity());
        telemetry.update();
    }

}



