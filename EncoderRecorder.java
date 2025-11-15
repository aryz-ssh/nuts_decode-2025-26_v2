package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@TeleOp(name = "EncoderRecorder", group = "Tools")
public class EncoderRecorder extends LinearOpMode {

    DcMotorEx motor;
    ArrayList<Integer> recordedPositions = new ArrayList<>();

    int replayIndex = 0;
    boolean aWasPressed = false;
    boolean bWasPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {

        motor = hardwareMap.get(DcMotorEx.class, "testMotor");

        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            int currentPos = motor.getCurrentPosition();

            // ---- RECORD POSITION ----
            if (gamepad1.a && !aWasPressed) {
                recordedPositions.add(currentPos);
                aWasPressed = true;
            }
            if (!gamepad1.a) aWasPressed = false;

            // ---- REPLAY TO NEXT POSITION ----
            if (gamepad1.b && !bWasPressed) {
                if (replayIndex < recordedPositions.size()) {
                    int target = recordedPositions.get(replayIndex);
                    goToPosition(target, 0.4, 2000);
                    replayIndex++;
                }
                bWasPressed = true;
            }
            if (!gamepad1.b) bWasPressed = false;

            // ---- TELEMETRY ----
            telemetry.addData("Current Pos", currentPos);
            telemetry.addData("Saved Positions", recordedPositions);
            telemetry.addData("Next Replay Index", replayIndex);
            telemetry.update();
        }
    }

    // Move to a given encoder target
    void goToPosition(int target, double power, long timeoutMs) {
        motor.setTargetPosition(target);
        motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motor.setPower(power);

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && motor.isBusy() && timer.milliseconds() < timeoutMs) {
            telemetry.addData("Moving to", target);
            telemetry.addData("Current", motor.getCurrentPosition());
            telemetry.update();
        }

        motor.setPower(0);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
}
