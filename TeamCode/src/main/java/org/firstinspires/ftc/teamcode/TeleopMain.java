package org.firstinspires.ftc.teamcode;

import static android.icu.lang.UProperty.MATH;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "TeleopMain")
public class TeleopMain extends LinearOpMode{

    //private static final double TICKS_PER_REVOLUTION = 1538.0;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean intakeDirectionFlip = true;
    public TeleopMain() {

    }

    // @Override
    public void runOpMode() throws InterruptedException {
        TeleopDrivetrain drivetrain = new TeleopDrivetrain(this);
        Mechanisms mech = new Mechanisms();
        ColorSensorTester colorSensor = new ColorSensorTester();
        //mech.initTelemetry(telemetry);

        drivetrain.initDriveTrain((hardwareMap));
        mech.initMechanisms(hardwareMap, telemetry);

        telemetry.addData("Status","READY TO NUT");

        this.waitForStart();

        //This is loop that checks the gamepad for inputs every iteration
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;   // forward/back (negative because stick up is negative)
            double x = gamepad1.left_stick_x;    // left/right strafe
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

/*            // Apply back wheel compensation (slightly boost back wheels)
            double backWheelCompensation = 1.5;

            targetBL *= backWheelCompensation;
            targetBR *= backWheelCompensation;*/

            // Re-normalize if any motor exceeds ±1.0 after compensation
            max = Math.max(1.0, Math.max(Math.abs(targetFL),
                    Math.max(Math.abs(targetFR),
                            Math.max(Math.abs(targetBL), Math.abs(targetBR)))));
            targetFL /= max;
            targetFR /= max;
            targetBL /= max;
            targetBR /= max;

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
        //Code for sorting
        //Scan apriltag, get motif, get balls, while going to shooting position, shoot, rotate immediately to next ball, and then shoot, and onwards
        //gets first ball and turns 120 degrees  clockwise
        //gets second ball and turns 120 degrees  clockwise
        //gets third ball and turns 60 degrees  clockwise
        // use color sensor on the ball that is now on the top
        // if it is the first color in the motif then the thing stays still - situation a
        // if it is wrong then turn 120 degrees counter clockwise
        // use color sensor on new ball to check if it is the first color of the motif
        // if it is right - situation b
        // if it is wrong turn 120 degrees COUNTER clockwise
        // after moving 120 degrees counter clockwise, move to situation a

        //situation a:
        // shoots first ball as it is right color
        // then turn 120 degrees counter clockwise
        // check if the ball in the thingy is the second color in the motif
        // if the ball is the right color - situation a1
        // if the ball is the wrong color - situation a2

        //situation a1:
        // second ball is the right color
        // shoots second ball
        // turn 120 degrees COUNTer clockwise
        // shoots the third ball

        //situation a2:
        // second ball is the WRONG color
        // turns 120 degrees cOUNTEr clockwise
        // shoots the second ball
        // turns 120 degrees clockwise
        // shoots the third ball

        // situation b:
        // the new ball is the RIGHt color and matches the first thing on the motif
        // shoot the first ball
        // then turn 120 degrees counter clockwise
        // check if the ball in the thingy is the second color in the motif
        // if the ball is the right color - situation a1
        // if the ball is the wrong color - situation a2
        //branch test
    }



}

