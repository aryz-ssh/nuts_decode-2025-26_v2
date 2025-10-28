package org.firstinspires.ftc.teamcode;

import static android.icu.lang.UProperty.MATH;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "TeleopGamePadStuff")
public class TeleopMain extends LinearOpMode{

    //private static final double TICKS_PER_REVOLUTION = 1538.0;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean intakeDirectionFlip = true;
    public TeleopMain() {

    }

   // @Override
    public void runOpMode() throws InterruptedException {
        TeleopDrivetrain drivetrain = new TeleopDrivetrain(this);
       // Mechanisms mech = new Mechanisms();
        //mech.initTelemetry(telemetry);

        drivetrain.initDriveTrain((hardwareMap));
        // mech.initMechanisms();

        telemetry.addData("Status","READY TO NUT");

        this.waitForStart();

        //This is loop that checks the gamepad for inputs every iteration
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;   // forward/back (negative because stick up is negative)
            double x = -gamepad1.left_stick_x;    // left/right strafe
            double rx = gamepad1.right_stick_x;  // rotation

            if (Math.abs(y) < 0.05) y = 0;
            if (Math.abs(x) < 0.05) x = 0;
            if (Math.abs(rx) < 0.05) rx = 0;
            System.out.println("s");
            double targetFL = y + x + rx;
            double targetFR = y - x - rx;
            double targetBL = y - x + rx;
            double targetBR = y + x - rx;

            double max = Math.max(1.0, Math.max(Math.abs(targetFL),
                    Math.max(Math.abs(targetFR),
                            Math.max(Math.abs(targetBL), Math.abs(targetBR)))));
            targetFL /= max;
            targetFR /= max;
            targetBL /= max;
            targetBR /= max;

            double driveScale = 0.7;
            targetFL *= driveScale;
            targetFR *= driveScale;
            targetBL *= driveScale;
            targetBR *= driveScale;

            drivetrain.updateDrive(targetFL, targetFR, targetBL, targetBR);

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

            if(gamepad2.right_bumper){
                //mech.setClawPivot("up");

            }
            if(gamepad2.left_bumper) {
                intakeDirectionFlip = false;

            } else {
                intakeDirectionFlip = true;
            }

/*            if(gamepad2.right_trigger > 0.1){
                mech.outtakeMotorStart(gamepad2.right_trigger);
            } else {
                mech.outtakeMotorStop();
            }*/
            if(gamepad2.left_trigger > 0.1) {
                mech.engageIntake(gamepad2.left_trigger, intakeDirectionFlip);
            } else {
                mech.disengageIntake();
            }


            telemetry.update();
            //sleep(100);
        }
    }



}

