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
            .mass(11.7934016);
            // .forwardZeroPowerAcceleration(-61.796424856692)
            // .lateralZeroPowerAcceleration(-87.550458680425)
            // .translationalPIDFCoefficients(new PIDFCoefficients(0.075, 0, 0.065, 0.005))
            // .headingPIDFCoefficients(new PIDFCoefficients(0.75, 0, 0.027, 0.003))
            // .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0,0.0001,0.6,0.01))
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
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
            // .xVelocity(72.344021142073)
            // .yVelocity(58.201999979257);

    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            .forwardTicksToInches(0.0044305534901019)
            .strafeTicksToInches(0.0046180983954515)
            .turnTicksToInches(0.001976225513021)
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




