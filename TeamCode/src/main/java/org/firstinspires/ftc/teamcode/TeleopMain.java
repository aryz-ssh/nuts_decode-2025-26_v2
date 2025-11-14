package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@TeleOp(name = "TeleopMain")
public class TeleopMain extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private boolean intakeDirectionFlip = true;
    private double targetHeading = 0.0;
    private double outtakeSpeed = 0.0;
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;

    private double applyDeadband(double value) {
        return (Math.abs(value) > 0.05) ? value : 0;
    }

    // ---------- Mechanisms ----------
    private Mechanisms mechanisms;
    private TeleopDrivetrain drivetrain;

    private ArrayList<String> intakeOrder = new ArrayList<>(); // Current balls in carousel

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize mechanisms
        mechanisms = new Mechanisms();
        mechanisms.initMechanisms(hardwareMap, telemetry);

        // drivetrain = new TeleopDrivetrain();
        drivetrain.initDriveTrain(hardwareMap);

        // Populate intake order for testing
        intakeOrder.add("red");
        intakeOrder.add("blue");
        intakeOrder.add("green");

        telemetry.addData("Status", "READY TO NUT");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
//            if (drivetrain.imu == null) {
//                telemetry.addData("Warning", "IMU not initialized");
//                telemetry.update();
//                continue;  // skip the loop until IMU is ready
//            }

// --- joystick inputs ---
            double y  = applyDeadband(gamepad1.left_stick_y);        // forward/back
            double x  = -applyDeadband(gamepad1.left_stick_x) * 1.1; // strafe
            double rx = -applyDeadband(gamepad1.right_stick_x);      // rotation

// --- mecanum math ---
            double fl = y + x + rx;
            double fr = y - x - rx;
            double bl = y - x + rx;
            double br = y + x - rx;

// normalize + scale
            double max = Math.max(1.0, Math.max(Math.abs(fl),
                    Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
            fl /= max; fr /= max; bl /= max; br /= max;

            double driveScale = 0.8;
            drivetrain.updateDrive(fl * driveScale, fr * driveScale, bl * driveScale, br * driveScale);

// --- allow heading reset ---
            if (gamepad1.b) {
             //   drivetrain.targetHeading = drivetrain.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
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

            // ---------------- Auto-Sorter using Bottom Color Sensor ----------------
            if (gamepad2.right_trigger > 0.1) {
                int steps = 1;
//                String bottomColor = mechanisms.getBottomBallColor();
//                int steps = mechanisms.stepsToAlign(bottomColor, intakeOrder);
/*            if(gamepad2.right_trigger > 0.1){
                mech.outtakeMotorStart(gamepad2.right_trigger);
            } else {
                mech.outtakeMotorStop();
            }*/
            if(gamepad1.left_trigger > 0.1) {
                mechanisms.engageIntake(gamepad1.left_trigger, intakeDirectionFlip);
            } else {
                mechanisms.disengageIntake();
            }
            // --- adjust outtake speed 0.0 → 1.0 in steps of 0.1 ---
            if (gamepad2.dpad_up && !dpadUpPressed) {
                outtakeSpeed += 0.1;
                telemetry.addData("Outtake Commanded Speed", outtakeSpeed);
                dpadUpPressed = true;
            } else if (!gamepad2.dpad_up) {
                dpadUpPressed = false;
            }

            if (gamepad2.dpad_down && !dpadDownPressed) {
                outtakeSpeed -= 0.1;
                telemetry.addData("Outtake Commanded Speed", outtakeSpeed);
                dpadDownPressed = true;
            } else if (!gamepad2.dpad_down) {
                dpadDownPressed = false;
            }

            // clamp to range 0–1
            outtakeSpeed = Math.max(0.0, Math.min(1.0, outtakeSpeed));

                // Rotate carousel to align the desired ball
                mechanisms.rotateCarousel(steps, 0.5);

                // Remove the ball after shooting
//                mechanisms.removeTopBall(intakeOrder);
            if(gamepad2.right_bumper) { // IN PROGRESS
                mechanisms.engageOuttake(outtakeSpeed);
            } else {
                mechanisms.disengageOuttake();
            }

//                telemetry.addData("Sorter", "Shot ball: %s", bottomColor);
            }

            telemetry.addData("IntakeOrder", intakeOrder.toString());
            telemetry.addData("Runtime", runtime.seconds());
            telemetry.update();
        }
    }

    // ---------------- Helper Functions ----------------
    private double applyDeadzone(double value, double threshold) {
        return Math.abs(value) < threshold ? 0 : value;
    }
}
