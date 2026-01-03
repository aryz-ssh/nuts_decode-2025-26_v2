package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "IMU Test V2", group = "Test")
public class IMUTest_V2 extends LinearOpMode{

    private IMU imu;
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {

        imu = hardwareMap.get(IMU.class, "imu");

        // IMPORTANT: Control Hub orientation
        RevHubOrientationOnRobot orientation =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                );

        IMU.Parameters params = new IMU.Parameters(orientation);
        imu.initialize(params);

        telemetry.addLine("IMU initialized");
        telemetry.addLine("USB: UP | Logo: RIGHT");
        telemetry.addLine("Press start");
        telemetry.update();

        waitForStart();

        timer.reset();

        while (opModeIsActive()) {

            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

            double yaw = angles.getYaw(AngleUnit.DEGREES);
            double pitch = angles.getPitch(AngleUnit.DEGREES);
            double roll = angles.getRoll(AngleUnit.DEGREES);

            telemetry.addData("Yaw (Heading)", "%.2f°", yaw);
            telemetry.addData("Pitch", "%.2f°", pitch);
            telemetry.addData("Roll", "%.2f°", roll);

            telemetry.addLine();
            telemetry.addLine("Rotate robot by hand:");
            telemetry.addLine("CCW = +Yaw | CW = -Yaw");

            telemetry.update();
        }
    }
}
