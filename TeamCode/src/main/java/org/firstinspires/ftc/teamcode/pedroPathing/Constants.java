package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.3398)
            .forwardZeroPowerAcceleration(-79.981519040563273)
            .lateralZeroPowerAcceleration(-68.253952903901287)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.12, 0, 0.002, 0.08))
            .headingPIDFCoefficients(new PIDFCoefficients(0.82, 0, 0.005, 0.05));
            // .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0,0.0001,0.6,0.01));
            // .centripetalScaling(0.0005);

    public static PathConstraints pathConstraints = new PathConstraints(0.99,
            100,
            0.85,
            1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftRearMotorName("leftBack")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(31.440613303778722)
            .yVelocity(22.32240864811224);
    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            .forwardTicksToInches(0.0044274088283232)
            .strafeTicksToInches(0.0044247984836949)
            .turnTicksToInches(0.0019740928037213)
            .leftPodY(7)
            .rightPodY(-7)
            .strafePodX(-1.5)
            .leftEncoder_HardwareMapName("rightBack")
            .rightEncoder_HardwareMapName("leftBack")
            .strafeEncoder_HardwareMapName("leftFront")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.REVERSE)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));

    public static Follower createFollower(HardwareMap hardwareMap) {
        CustomMecanumPedroDrivebase drivebase =
                new CustomMecanumPedroDrivebase(hardwareMap, driveConstants);

        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .setDrivetrain(drivebase)
                .threeWheelIMULocalizer(localizerConstants)
                .build();
    }
}




