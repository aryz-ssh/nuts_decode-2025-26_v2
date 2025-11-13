package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;



@TeleOp(name = "TeleopMain")
public class TeleopMain extends LinearOpMode{

    //private static final double TICKS_PER_REVOLUTION = 1538.0;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean intakeDirectionFlip = true;
    private double targetHeading = 0.0;

    private double applyDeadband(double value) {
        return (Math.abs(value) > 0.05) ? value : 0;
    }

    public TeleopMain() {

    }

   // @Override
    public void runOpMode() throws InterruptedException {
        TeleopDrivetrain drivetrain = new TeleopDrivetrain(this);
        Mechanisms mech = new Mechanisms();
        //mech.initTelemetry(telemetry);

        drivetrain.initDriveTrain((hardwareMap));
        mech.initMechanisms(hardwareMap, telemetry);

        telemetry.addData("Status","READY TO NUT");

        waitForStart();
        drivetrain.targetHeading = drivetrain.imu
                .getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.DEGREES);

        //This is loop that checks the gamepad for inputs every iteration
        while (opModeIsActive()) {
            if (drivetrain.imu == null) {
                telemetry.addData("Warning", "IMU not initialized");
                telemetry.update();
                continue;  // skip the loop until IMU is ready
            }

// --- joystick inputs ---
            double y  = applyDeadband(gamepad1.left_stick_y);        // forward/back
            double x  = -applyDeadband(gamepad1.left_stick_x) * 1.1; // strafe
            double rx = -applyDeadband(gamepad1.right_stick_x);      // rotation

// --- get odometry drift corrections ---
            double headingDriftCorrection = drivetrain.getHeadingDriftCorrection();  // cancel veering
            double lateralDriftCorrection = drivetrain.getLateralDriftCorrection();  // cancel sideways slip

// --- get IMU heading hold correction ---
            YawPitchRollAngles orientation = drivetrain.imu.getRobotYawPitchRollAngles();
            double currentHeading = orientation.getYaw(AngleUnit.DEGREES);
            double headingError = (drivetrain.targetHeading - currentHeading + 540) % 360 - 180;

// smaller = smoother hold, larger = stronger lock
            double kP_headingHold = 0.02;
            double imuCorrection = kP_headingHold * headingError;

// --- apply corrections ---
            rx += headingDriftCorrection;  // short-term correction from odo L/R
            rx -= imuCorrection;           // long-term heading hold from IMU
            x  += lateralDriftCorrection;  // keep strafing straight

// --- mecanum math ---
            double fl = y + x + rx;
            double fr = y - x - rx;
            double bl = y - x + rx;
            double br = y + x - rx;

// normalize + scale
            double max = Math.max(1.0, Math.max(Math.abs(fl),
                    Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
            fl /= max; fr /= max; bl /= max; br /= max;

            double driveScale = 0.7;
            drivetrain.updateDrive(fl * driveScale, fr * driveScale, bl * driveScale, br * driveScale);

// --- allow heading reset ---
            if (gamepad1.b) {
                drivetrain.targetHeading = drivetrain.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            }

            if (gamepad2.dpad_left){
              //  mech.simplePivotLimit1();
                //mech.extendViperSlide("backward");

            }
            if (gamepad2.dpad_right){
              //  mech.simplePivotLimit1();
               // mech.extendViperSlide("forward");
            }

            if(gamepad2.dpad_down){
                //mech.pivotLimit1();
             //   mech.armMotorPivot("down");
            }
            if(gamepad2.dpad_up){
                //mech.pivotLimit1();
               // mech.armMotorPivot("up");
            }

            if (gamepad2.a) {
              //  mech.SpecimenScoringPosition();
            }
            //block pickup positions on wall
            if (gamepad2.b) {
            //    mech.SpecimenPickupPosition();
            }
            //block pickup position from floor
            if(gamepad2.y){
              //  mech.BlockPickupPosition();
            }
            if (gamepad2.x) {
                //mech.SpecimenViperPosition();
            }

            if(gamepad1.left_bumper) {
                intakeDirectionFlip = false;

            } else {
                intakeDirectionFlip = true;
            }

/*            if(gamepad2.right_trigger > 0.1){
                mech.outtakeMotorStart(gamepad2.right_trigger);
            } else {
                mech.outtakeMotorStop();
            }*/
            if(gamepad1.left_trigger > 0.1) {
                mech.engageIntake(gamepad2.left_trigger, intakeDirectionFlip);
            } else {
                mech.disengageIntake();
            }

            if(gamepad2.right_bumper) { // IN PROGRESS
                mech.engageOuttake(gamepad2.right_trigger);
            } else {
                mech.disengageOuttake();
            }


            telemetry.update();
            //sleep(100);
        }
    }



}

