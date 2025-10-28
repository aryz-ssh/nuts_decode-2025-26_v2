package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


public class Mechanisms {
    // INTAKE SYSTEM VARIABLES
    // ASSUMING 1150 RPM
    public DcMotorEx intakeMotor;
    private static final double intakeTargetVelocity = 363;
    private static final double TICKS_PER_REV = 145.1;
    private static final double MAX_RPM = 1150;
    private static final double MAX_TICKS_PER_SEC = (TICKS_PER_REV * MAX_RPM) / 60.0; // ≈ 2786 t/s

    // SORTING SYSTEM VARIABLES
    // ASSUMING EXTERNALLY GEARED SERVO
    public DcMotor sortingMotor;
    private static final double sortingMotor_TPR = 384.5; // Encoder ticks per revolution
    private static final int sorter_pos1 = 67; // Encoder ticks per revolution
    private static final int sorter_pos2 = 67; // Encoder ticks per revolution
    private static final int sorter_pos3 = 67; // Encoder ticks per revolution

    // OUTTAKE SYSTEM VARIABLES
    // ASSUMING 1150 RPM MOTORS
    public DcMotorEx outtakeMotor1;
    public DcMotorEx outtakeMotor2;

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

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void initOuttakeSystem(HardwareMap hardwareMap) {
        outtakeMotor1 = hardwareMap.get(DcMotorEx.class, "outtakeMotor1"); // 1150 rpm dc motor
//        outtakeMotor2 = hardwareMap.get(DcMotorEx.class, "outtakeMotor2"); // 1150 rpm dc motor

        outtakeMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
//        outtakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        outtakeMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        outtakeMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void initFarisWheel(HardwareMap hardwareMap) {
        sortingMotor = hardwareMap.get(DcMotor.class, "sortingMotor"); // 435 rpm dc motor

        sortingMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        sortingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sortingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sortingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void initElevation(HardwareMap hardwareMap) {
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
    }

    public void initMechanisms(){
        //        initIntakeSystem(hardwareMap);
        initOuttakeSystem(hardwareMap);
        //        initFarisWheel(hardwareMap);
        //        initElevation(hardwareMap);
    }

    public void engageIntake(double power, boolean intakeDirectionFlip) {
        if (intakeDirectionFlip){
            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        intakeMotor.setVelocity(power*MAX_TICKS_PER_SEC);
    }

    public void disengageIntake(HardwareMap hardwareMap) {
        // intakeMotor  .setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setVelocity(0.0);
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

    public void outtakeMotorStart(double power) {
        outtakeMotor1.setVelocity(power*MAX_TICKS_PER_SEC);
        // outtakeMotor2.setVelocity(power*MAX_TICKS_PER_SEC);
        telemetry.addData("Velocity", outtakeMotor1.getVelocity());
        telemetry.update();
    }

    public void outtakeMotorStop() {
        outtakeMotor1.setPower(0.0);
        // outtakeMotor2.setPower(0.0);
        telemetry.addData("Velocity", outtakeMotor1.getVelocity());
        telemetry.update();
    }

}
