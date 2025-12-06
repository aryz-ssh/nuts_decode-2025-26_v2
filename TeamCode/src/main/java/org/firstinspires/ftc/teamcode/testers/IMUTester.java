package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "IMU Test", group = "Test")
public class IMUTester extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // ----- GET IMU FROM HARDWARE MAP -----
        IMU imu = hardwareMap.get(IMU.class, "imu");

        // ----- SET CORRECT HUB ORIENTATION -----
        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,     // change if needed
                        RevHubOrientationOnRobot.UsbFacingDirection.UP  // change if needed
                )
        );

        imu.initialize(params);

        telemetry.addLine("IMU Initialized");
        telemetry.addLine("Press START to view heading");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double headingDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            telemetry.addData("Heading (deg)", headingDeg);
            telemetry.update();

            sleep(50);
        }
    }
}
