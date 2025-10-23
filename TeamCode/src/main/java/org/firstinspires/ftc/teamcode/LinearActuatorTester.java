
package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "LinearActuatorTester")
public class LinearActuatorTester extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException {

        Servo linearActuator = hardwareMap.get(Servo.class, "linearActuator");
        linearActuator.setDirection(FORWARD);

        // Reset the encoder to start from 0
      //  Servo.setMode(Servo.RunMode.STOP_AND_RESET_ENCODER);

        // Set motor to run using encoder (but no power for manual movement)
    //    Servo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //    motor.setPower(0); // Ensure no power is applied to the motor

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {

            while(gamepad2.b) {
                linearActuator.setPosition(1.0);
                sleep(2500);
                linearActuator.setPosition(0.0);
                sleep(2500);
            }
        };
    }
}

